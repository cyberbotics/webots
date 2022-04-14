// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbMesh.hpp"

#include "WbApplication.hpp"
#include "WbDownloader.hpp"
#include "WbGroup.hpp"
#include "WbMFString.hpp"
#include "WbNodeUtilities.hpp"
#include "WbResizeManipulator.hpp"
#include "WbTriangleMesh.hpp"
#include "WbUrl.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <QtCore/QEventLoop>

void WbMesh::init() {
  mUrl = findMFString("url");
  mName = findSFString("name");
  mMaterialIndex = findSFInt("materialIndex");
  mIsCollada = false;
  mResizeConstraint = WbWrenAbstractResizeManipulator::UNIFORM;
  mDownloader = NULL;
}

WbMesh::WbMesh(WbTokenizer *tokenizer) : WbTriangleMeshGeometry("Mesh", tokenizer) {
  init();
}

WbMesh::WbMesh(const WbMesh &other) : WbTriangleMeshGeometry(other) {
  init();
}

WbMesh::WbMesh(const WbNode &other) : WbTriangleMeshGeometry(other) {
  init();
}

WbMesh::~WbMesh() {
}

void WbMesh::downloadAssets() {
  if (mUrl->size() == 0)
    return;
  const QString &url(mUrl->item(0));
  const QString completeUrl = WbUrl::computePath(this, "url", url, false);
  if (WbUrl::isWeb(completeUrl)) {
    delete mDownloader;
    mDownloader = new WbDownloader(this);
    if (!WbWorld::instance()->isLoading())  // URL changed from the scene tree or supervisor
      connect(mDownloader, &WbDownloader::complete, this, &WbMesh::downloadUpdate);

    mDownloader->download(QUrl(completeUrl));
  }
}

void WbMesh::downloadUpdate() {
  updateUrl();
  WbWorld::instance()->viewpoint()->emit refreshRequired();
  const WbNode *ancestor = WbNodeUtilities::findTopNode(this);
  const WbGroup *group = dynamic_cast<const WbGroup *>(ancestor);
  if (group)
    group->recomputeBoundingSphere();
}

void WbMesh::preFinalize() {
  mIsCollada = (path().mid(path().lastIndexOf('.') + 1).toLower() == "dae");
  WbTriangleMeshGeometry::preFinalize();
  updateUrl();
}

void WbMesh::postFinalize() {
  WbTriangleMeshGeometry::postFinalize();

  connect(mUrl, &WbMFString::changed, this, &WbMesh::updateUrl);
  connect(mName, &WbSFString::changed, this, &WbMesh::updateName);
  connect(mMaterialIndex, &WbSFInt::changed, this, &WbMesh::updateMaterialIndex);
}

void WbMesh::createResizeManipulator() {
  mResizeManipulator = new WbRegularResizeManipulator(uniqueId(), WbWrenAbstractResizeManipulator::ResizeConstraint::X_EQUAL_Z);
}

bool WbMesh::checkIfNameExists(const aiScene *scene, const QString &name) const {
  std::list<aiNode *> queue;
  queue.push_back(scene->mRootNode);
  aiNode *node = NULL;
  while (!queue.empty()) {
    node = queue.front();
    queue.pop_front();
    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
      const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
      if (name == mesh->mName.data)
        return true;
    }
  }
  return false;
}

void WbMesh::updateTriangleMesh(bool issueWarnings) {
  const QString filePath(path());
  if (filePath.isEmpty())
    return;

  if (mDownloader && !mDownloader->error().isEmpty()) {
    warn(mDownloader->error());
    delete mDownloader;
    mDownloader = NULL;
    return;
  }

  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_CAMERAS | aiComponent_LIGHTS | aiComponent_BONEWEIGHTS |
                                                        aiComponent_ANIMATIONS | aiComponent_TEXTURES | aiComponent_COLORS);
  const aiScene *scene;
  unsigned int flags = aiProcess_ValidateDataStructure | aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                       aiProcess_JoinIdenticalVertices | aiProcess_OptimizeGraph | aiProcess_RemoveComponent |
                       aiProcess_FlipUVs;
  if (WbUrl::isWeb(filePath)) {
    if (mDownloader == NULL)
      downloadAssets();

    if (mDownloader->hasFinished()) {
      const QByteArray data = mDownloader->device()->readAll();
      const char *hint = filePath.mid(filePath.lastIndexOf('.') + 1).toUtf8().constData();
      scene = importer.ReadFileFromMemory(data.constData(), data.size(), flags, hint);
      delete mDownloader;
      mDownloader = NULL;
    } else
      return;
  } else
    scene = importer.ReadFile(filePath.toStdString().c_str(), flags);

  if (!scene) {
    warn(tr("Invalid data, please verify mesh file (bone weights, normals, ...): %1").arg(importer.GetErrorString()));
    return;
  } else if (!scene->HasMeshes()) {
    warn(tr("This file doesn't contain any mesh."));
    return;
  }

  if (mIsCollada && mName->value() != "" && !checkIfNameExists(scene, mName->value())) {
    warn(tr("Geometry with the name \"%1\" doesn't exist in the mesh.").arg(mName->value()));
    return;
  }

  if (mIsCollada && mMaterialIndex->value() >= (int)scene->mNumMaterials) {
    warn(tr("Geometry with color index \"%1\" doesn't exist in the mesh.").arg(mMaterialIndex->value()));
    return;
  }

  // Assimp fix for up_axis, adapted from https://github.com/assimp/assimp/issues/849
  if (mIsCollada)  // rotate around X by 90° to swap Y and Z axis
    scene->mRootNode->mTransformation =
      aiMatrix4x4(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1) * scene->mRootNode->mTransformation;

  // count total number of vertices and faces
  int totalVertices = 0;
  int totalFaces = 0;
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh *mesh = scene->mMeshes[i];
    if (mIsCollada && !mName->value().isEmpty() && mName->value() != mesh->mName.data)
      continue;

    if (mIsCollada && mMaterialIndex->value() >= 0 && mMaterialIndex->value() != (int)mesh->mMaterialIndex)
      continue;

    totalVertices += mesh->mNumVertices;
    totalFaces += mesh->mNumFaces;
  }

  // create the arrays
  int currentCoordIndex = 0;
  double *const coordData = new double[3 * totalVertices];
  int currentNormalIndex = 0;
  double *const normalData = new double[3 * totalVertices];
  int currentTexCoordIndex = 0;
  double *const texCoordData = new double[2 * totalVertices];
  int currentIndexIndex = 0;
  unsigned int *const indexData = new unsigned int[3 * totalFaces];

  // loop over all the node to find meshes
  std::list<aiNode *> queue;
  queue.push_back(scene->mRootNode);
  aiNode *node = NULL;
  unsigned int indexOffset = 0;
  while (!queue.empty()) {
    node = queue.front();
    queue.pop_front();

    // compute absolute transform of this node from all the parents
    aiMatrix4x4 transform;
    aiNode *current = node;
    while (current != NULL) {
      transform *= current->mTransformation;
      current = current->mParent;
    }

    // merge all the meshes of this node
    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
      const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
      if (mIsCollada && mName->value() != "" && mName->value() != mesh->mName.data)
        continue;

      if (mIsCollada && mMaterialIndex->value() >= 0 && mMaterialIndex->value() != (int)mesh->mMaterialIndex)
        continue;

      for (size_t j = 0; j < mesh->mNumVertices; ++j) {
        // extract the coordinate
        const aiVector3D vertice = transform * mesh->mVertices[j];
        coordData[currentCoordIndex++] = vertice[0];
        coordData[currentCoordIndex++] = vertice[1];
        coordData[currentCoordIndex++] = vertice[2];
        // extract the normal
        const aiVector3D normal = transform * mesh->mNormals[j];
        normalData[currentNormalIndex++] = normal[0];
        normalData[currentNormalIndex++] = normal[1];
        normalData[currentNormalIndex++] = normal[2];
        // extract the texture coordinate
        if (mesh->HasTextureCoords(0)) {
          texCoordData[currentTexCoordIndex++] = mesh->mTextureCoords[0][j].x;
          texCoordData[currentTexCoordIndex++] = mesh->mTextureCoords[0][j].y;
        } else {
          texCoordData[currentTexCoordIndex++] = 0.5;
          texCoordData[currentTexCoordIndex++] = 0.5;
        }
      }

      // create the index array
      for (size_t j = 0; j < mesh->mNumFaces; ++j) {
        const aiFace face = mesh->mFaces[j];
        if (face.mNumIndices < 3)  // we want to skip lines
          continue;
        assert(face.mNumIndices == 3);
        indexData[currentIndexIndex++] = face.mIndices[0] + indexOffset;
        indexData[currentIndexIndex++] = face.mIndices[1] + indexOffset;
        indexData[currentIndexIndex++] = face.mIndices[2] + indexOffset;
      }

      indexOffset += mesh->mNumVertices;
    }

    // add all the children of this node to the queue
    for (size_t i = 0; i < node->mNumChildren; ++i)
      queue.push_back(node->mChildren[i]);
  }

  if (!node) {
    warn(tr("This file doesn't contain any mesh."));
    delete[] coordData;
    delete[] normalData;
    delete[] texCoordData;
    delete[] indexData;
    return;
  }

  mTriangleMeshError = mTriangleMesh->init(coordData, normalData, texCoordData, indexData, totalVertices, currentIndexIndex);

  if (issueWarnings) {
    foreach (QString warning, mTriangleMesh->warnings())
      warn(warning);

    if (!mTriangleMeshError.isEmpty())
      warn(tr("Cannot create IndexedFaceSet because: \"%1\".").arg(mTriangleMeshError));
  }

  delete[] coordData;
  delete[] normalData;
  delete[] texCoordData;
  delete[] indexData;
}

uint64_t WbMesh::computeHash() const {
  const QString meshPathNameIndex = path() + (mIsCollada ? mName->value() + QString::number(mMaterialIndex->value()) : "");
  return WbTriangleMeshCache::sipHash13x(meshPathNameIndex.toUtf8().constData(), meshPathNameIndex.size());
}

void WbMesh::exportNodeContents(WbVrmlWriter &writer) const {
  if (!writer.isVrml()) {
    WbTriangleMeshGeometry::exportNodeContents(writer);
    return;
  }
  // export the content as IndexedFaceSet in VRML
  const int n = mTriangleMesh->numberOfTriangles();
  const int n3 = n * 3;
  int *const coordIndex = new int[n3];
  int *const normalIndex = new int[n3];
  int *const texCoordIndex = new int[n3];
  double *const vertex = new double[n * 9];
  double *const normal = new double[n * 9];
  double *const texture = new double[n * 6];
  int indexCount = 0;
  int vertexCount = 0;
  int normalCount = 0;
  int textureCount = 0;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double x = mTriangleMesh->vertex(i, j, 0);
      const double y = mTriangleMesh->vertex(i, j, 1);
      const double z = mTriangleMesh->vertex(i, j, 2);
      bool found = false;
      for (int l = 0; l < vertexCount; ++l) {
        const int k = 3 * l;
        if (vertex[k] == x && vertex[k + 1] == y && vertex[k + 2] == z) {
          coordIndex[indexCount] = l;
          found = true;
          break;
        }
      }
      if (!found) {
        const int v = 3 * vertexCount;
        vertex[v] = x;
        vertex[v + 1] = y;
        vertex[v + 2] = z;
        coordIndex[indexCount] = vertexCount;
        ++vertexCount;
      }
      const double nx = mTriangleMesh->normal(i, j, 0);
      const double ny = mTriangleMesh->normal(i, j, 1);
      const double nz = mTriangleMesh->normal(i, j, 2);
      found = false;
      for (int l = 0; l < normalCount; ++l) {
        const int k = 3 * l;
        if (normal[k] == nx && normal[k + 1] == ny && normal[k + 2] == nz) {
          normalIndex[indexCount] = l;
          found = true;
          break;
        }
      }
      if (!found) {
        const int v = 3 * normalCount;
        normal[v] = nx;
        normal[v + 1] = ny;
        normal[v + 2] = nz;
        normalIndex[indexCount] = normalCount;
        ++normalCount;
      }

      const double tu = mTriangleMesh->textureCoordinate(i, j, 0);
      const double tv = mTriangleMesh->textureCoordinate(i, j, 1);
      found = false;
      for (int l = 0; l < textureCount; ++l) {
        const int k = 2 * l;
        if (texture[k] == tu && texture[k + 1] == tv) {
          texCoordIndex[indexCount] = l;
          found = true;
          break;
        }
      }
      if (!found) {
        const int v = 2 * textureCount;
        texture[v] = tu;
        texture[v + 1] = tv;
        texCoordIndex[indexCount] = textureCount;
        ++textureCount;
      }
      ++indexCount;
    }
  }

  writer.indent();
  writer << "coord Coordinate {\n";
  writer.increaseIndent();
  writer.indent();
  writer << "point [\n";
  writer.increaseIndent();
  writer.indent();
  for (int i = 0; i < vertexCount; ++i) {
    if (i != 0)
      writer << ", ";
    const int j = 3 * i;
    writer << vertex[j] << " " << vertex[j + 1] << " " << vertex[j + 2];
  }
  writer << "\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "]\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "}\n";

  writer.indent();
  writer << "normal Normal {\n";
  writer.increaseIndent();
  writer.indent();
  writer << "point [\n";
  writer.increaseIndent();
  writer.indent();
  for (int i = 0; i < normalCount; ++i) {
    if (i != 0)
      writer << ", ";
    const int j = 3 * i;
    writer << normal[j] << " " << normal[j + 1] << " " << normal[j + 2];
  }
  writer << "\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "]\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "}\n";

  writer.indent();
  writer << "texCoord TextureCoordinate {\n";
  writer.increaseIndent();
  writer.indent();
  writer << "point [\n";
  writer.increaseIndent();
  writer.indent();
  for (int i = 0; i < textureCount; ++i) {
    if (i != 0)
      writer << ", ";
    const int j = 2 * i;
    writer << texture[j] << " " << texture[j + 1];
  }
  writer << "\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "]\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "}\n";

  writer.indent();
  writer << "coordIndex [\n";
  writer.increaseIndent();
  writer.indent();
  for (int i = 0; i < indexCount; ++i) {
    if (i != 0) {
      writer << " ";
      if (i % 3 == 0)
        writer << "-1 ";
    }
    writer << coordIndex[i];
  }
  writer << " -1\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "]\n";

  writer.indent();
  writer << "normalIndex [\n";
  writer.increaseIndent();
  writer.indent();
  for (int i = 0; i < indexCount; ++i) {
    if (i != 0) {
      writer << " ";
      if (i % 3 == 0)
        writer << "-1 ";
    }
    writer << normalIndex[i];
  }
  writer << " -1\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "]\n";

  writer.indent();
  writer << "texCoordIndex [\n";
  writer.increaseIndent();
  writer.indent();
  for (int i = 0; i < indexCount; ++i) {
    if (i != 0) {
      writer << " ";
      if (i % 3 == 0)
        writer << "-1 ";
    }
    writer << texCoordIndex[i];
  }
  writer << " -1\n";
  writer.decreaseIndent();
  writer.indent();
  writer << "]\n";

  delete[] coordIndex;
  delete[] normalIndex;
  delete[] texCoordIndex;
  delete[] vertex;
  delete[] normal;
  delete[] texture;
}

void WbMesh::updateUrl() {
  // check url validity
  if (path().isEmpty())
    return;

  mIsCollada = (path().mid(path().lastIndexOf('.') + 1).toLower() == "dae");

  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  const int n = mUrl->size();
  for (int i = 0; i < n; i++) {
    QString item = mUrl->item(i);
    mUrl->setItem(i, item.replace("\\", "/"));
  }

  if (n > 0 && !WbWorld::instance()->isLoading() && WbUrl::isWeb(mUrl->item(0)) && mDownloader == NULL) {
    // url was changed from the scene tree or supervisor
    downloadAssets();
    return;
  }

  if (areWrenObjectsInitialized()) {
    buildWrenMesh(true);
    if (n > 0)
      emit wrenObjectsCreated();  // throw signal to update pickable state
  }

  if (isAValidBoundingObject())
    applyToOdeData();

  if (isPostFinalizedCalled())
    emit changed();
}

void WbMesh::updateName() {
  if (!mIsCollada)
    return;

  if (areWrenObjectsInitialized())
    buildWrenMesh(true);

  if (isPostFinalizedCalled())
    emit changed();
}

void WbMesh::updateMaterialIndex() {
  if (!mIsCollada)
    return;

  if (areWrenObjectsInitialized())
    buildWrenMesh(true);

  if (isPostFinalizedCalled())
    emit changed();
}

QString WbMesh::path() const {
  return WbUrl::computePath(this, "url", mUrl, 0);
}
