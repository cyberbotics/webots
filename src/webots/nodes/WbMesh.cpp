// Copyright 1996-2022 Cyberbotics Ltd.
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
#include "WbField.hpp"
#include "WbGroup.hpp"
#include "WbMFString.hpp"
#include "WbNetwork.hpp"
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
#include <QtCore/QFile>
#include <QtCore/QIODevice>

void WbMesh::init() {
  mUrl = findMFString("url");
  mCcw = findSFBool("ccw");
  mName = findSFString("name");
  mMaterialIndex = findSFInt("materialIndex");
  mIsCollada = false;
  mResizeConstraint = WbWrenAbstractResizeManipulator::UNIFORM;
  mDownloader = NULL;
  setCcw(mCcw->value());
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

  const QString &completeUrl = WbUrl::computePath(this, "url", mUrl, 0);
  if (!WbUrl::isWeb(completeUrl) || WbNetwork::instance()->isCachedWithMapUpdate(completeUrl))
    return;

  if (mDownloader != NULL && mDownloader->hasFinished())
    delete mDownloader;

  mDownloader = new WbDownloader(this);
  if (!WbWorld::instance()->isLoading())  // URL changed from the scene tree or supervisor
    connect(mDownloader, &WbDownloader::complete, this, &WbMesh::downloadUpdate);

  mDownloader->download(QUrl(completeUrl));
}

void WbMesh::downloadUpdate() {
  updateUrl();
  WbWorld::instance()->viewpoint()->emit refreshRequired();
  const WbNode *ancestor = WbNodeUtilities::findTopNode(this);
  WbGroup *group = dynamic_cast<WbGroup *>(const_cast<WbNode *>(ancestor));
  if (group)
    group->recomputeBoundingSphere();
}

void WbMesh::preFinalize() {
  const QString &completeUrl = WbUrl::computePath(this, "url", mUrl, 0);
  mIsCollada = (completeUrl.mid(completeUrl.lastIndexOf('.') + 1).toLower() == "dae");
  WbTriangleMeshGeometry::preFinalize();
  updateUrl();
}

void WbMesh::postFinalize() {
  WbTriangleMeshGeometry::postFinalize();

  connect(mUrl, &WbMFString::changed, this, &WbMesh::updateUrl);
  connect(mCcw, &WbSFBool::changed, this, &WbMesh::updateCcw);
  connect(mName, &WbSFString::changed, this, &WbMesh::updateName);
  connect(mMaterialIndex, &WbSFInt::changed, this, &WbMesh::updateMaterialIndex);
}

void WbMesh::createResizeManipulator() {
  mResizeManipulator = new WbRegularResizeManipulator(uniqueId(), WbWrenAbstractResizeManipulator::ResizeConstraint::X_EQUAL_Y);
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
  const QString &filePath = WbUrl::computePath(this, "url", mUrl, 0);
  if (filePath.isEmpty()) {
    mTriangleMesh->init(NULL, NULL, NULL, NULL, 0, 0);
    if (mUrl->size() > 0 && mUrl->item(0) != filePath)
      warn(tr("File '%1' could not be found.").arg(mUrl->item(0)));
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
    if (!WbNetwork::instance()->isCachedWithMapUpdate(filePath)) {
      if (mDownloader == NULL)  // never attempted to download it, try now
        downloadAssets();
      return;
    }

    QFile file(WbNetwork::instance()->get(filePath));
    if (!file.open(QIODevice::ReadOnly)) {
      warn(tr("Mesh file could not be read: '%1'").arg(filePath));
      return;
    }
    const QByteArray data = file.readAll();
    const char *hint = filePath.mid(filePath.lastIndexOf('.') + 1).toUtf8().constData();
    scene = importer.ReadFileFromMemory(data.constData(), data.size(), flags, hint);
  } else
    scene = importer.ReadFile(filePath.toUtf8().constData(), flags);

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
  const QString &completeUrl = WbUrl::computePath(this, "url", mUrl, 0);
  const QString meshPathNameIndex = completeUrl + (mIsCollada ? mName->value() + QString::number(mMaterialIndex->value()) : "");
  const QByteArray key = meshPathNameIndex.toUtf8();
  const uint64_t hash = WbTriangleMeshCache::sipHash13x(key.constData(), key.size());
  return hash;
}

void WbMesh::updateUrl() {
  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  const int n = mUrl->size();
  for (int i = 0; i < n; i++) {
    QString item = mUrl->item(i);
    mUrl->blockSignals(true);
    mUrl->setItem(i, item.replace("\\", "/"));
    mUrl->blockSignals(false);
  }

  if (n > 0) {
    const QString &completeUrl = WbUrl::computePath(this, "url", mUrl->item(0));
    mIsCollada = (completeUrl.mid(completeUrl.lastIndexOf('.') + 1).toLower() == "dae");
    if (WbUrl::isWeb(completeUrl)) {
      if (mDownloader && !mDownloader->error().isEmpty()) {
        warn(mDownloader->error());  // failure downloading or file does not exist (404)
        deleteWrenRenderable();
        wr_static_mesh_delete(mWrenMesh);
        delete mDownloader;
        mDownloader = NULL;
        mWrenMesh = NULL;
        return;
      }

      if (!WbNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
        if (mDownloader && mDownloader->hasFinished()) {
          delete mDownloader;
          mDownloader = NULL;
        }

        downloadAssets();  // URL was changed from the scene tree or supervisor
        return;
      }
    }
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

void WbMesh::updateCcw() {
  setCcw(mCcw->value());

  if (areWrenObjectsInitialized())
    buildWrenMesh(true);

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

void WbMesh::exportNodeFields(WbWriter &writer) const {
  if (!(writer.isX3d() || writer.isProto()))
    return;

  if (mUrl->size() == 0)
    return;

  WbField urlFieldCopy(*findField("url", true));
  for (int i = 0; i < mUrl->size(); ++i) {
    const QString &completeUrl = WbUrl::computePath(this, "url", mUrl, i);
    WbMFString *urlFieldValue = dynamic_cast<WbMFString *>(urlFieldCopy.value());
    if (WbUrl::isLocalUrl(completeUrl))
      urlFieldValue->setItem(i, WbUrl::computeLocalAssetUrl(completeUrl, writer.isX3d()));
    else if (WbUrl::isWeb(completeUrl))
      urlFieldValue->setItem(i, completeUrl);
    else {
      if (writer.isWritingToFile())
        urlFieldValue->setItem(i, WbUrl::exportMesh(this, mUrl, i, writer));
      else
        urlFieldValue->setItem(i, WbUrl::expressRelativeToWorld(completeUrl));
    }
  }

  urlFieldCopy.write(writer);

  findField("ccw", true)->write(writer);
  findField("materialIndex", -1)->write(writer);
  if (!mName->value().isEmpty()) {
    QString dirtyName = mName->value();
    dirtyName.replace("\'", "&apos;", Qt::CaseInsensitive);
    dirtyName.replace("\"", "&quot;", Qt::CaseInsensitive);
    dirtyName.replace(">", "&gt;", Qt::CaseInsensitive);
    dirtyName.replace("<", "&lt;", Qt::CaseInsensitive);
    writer << " name='" << dirtyName.replace("&", "&amp;", Qt::CaseInsensitive) << "'";
  }
}
