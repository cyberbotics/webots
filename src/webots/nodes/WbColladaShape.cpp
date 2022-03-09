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

#include "WbColladaShape.hpp"

#include "WbAbstractAppearance.hpp"
#include "WbBoundingSphere.hpp"
#include "WbDownloader.hpp"
#include "WbSFString.hpp"
#include "WbTriangleMesh.hpp"
#include "WbUrl.hpp"
#include "WbWrenShaders.hpp"

#include <QtCore/QFileInfo>

#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

void WbColladaShape::init() {
  mUrl = findSFString("url");

  mBoundingSphere = NULL;
}

WbColladaShape::WbColladaShape(WbTokenizer *tokenizer) : WbBaseNode("ColladaShape", tokenizer) {
  init();
}

WbColladaShape::WbColladaShape(const WbColladaShape &other) : WbBaseNode(other) {
  init();
}

WbColladaShape::WbColladaShape(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbColladaShape::~WbColladaShape() {
  if (areWrenObjectsInitialized())
    deleteWrenMeshes();

  delete mBoundingSphere;
}

void WbColladaShape::downloadAssets() {
}

void WbColladaShape::preFinalize() {
  WbBaseNode::preFinalize();

  mBoundingSphere = new WbBoundingSphere(this);
}

void WbColladaShape::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mUrl, &WbSFString::changed, this, &WbColladaShape::updateUrl);

  updateUrl();
}

void WbColladaShape::updateUrl() {
  if (!isPostFinalizedCalled())
    return;

  if (!mUrl->value().isEmpty()) {
    // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
    QString url = mUrl->value();
    mUrl->blockSignals(true);
    mUrl->setValue(url.replace("\\", "/"));
    mUrl->blockSignals(false);

    const QFileInfo fi(mUrl->value());
    const QStringList supportedExtensions = {"DAE"};
    if (!supportedExtensions.contains(fi.completeSuffix(), Qt::CaseInsensitive)) {
      warn(tr("Invalid url '%1'. Supported formats are: '%2'.").arg(mUrl->value()).arg(supportedExtensions.join("', '")));
      return;
    }

    // TODO: if remote, download it

    updateShape();
  }
}

void WbColladaShape::updateShape() {
  createWrenMeshes();
  // update appearance?
}

void WbColladaShape::createWrenMeshes() {
  deleteWrenMeshes();

  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                              aiComponent_CAMERAS | aiComponent_LIGHTS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS);
  const aiScene *scene =
    importer.ReadFile(colladaPath().toStdString().c_str(), aiProcess_ValidateDataStructure | aiProcess_Triangulate |
                                                             aiProcess_JoinIdenticalVertices | aiProcess_RemoveComponent);
  if (!scene) {
    warn(tr("Invalid data, please verify collada file: %1").arg(importer.GetErrorString()));
    return;
  }

  // Assimp fix for up_axis
  // Adapted from https://github.com/assimp/assimp/issues/849
  int upAxis = 1, upAxisSign = 1, frontAxis = 2, frontAxisSign = 1, coordAxis = 0, coordAxisSign = 1;
  double unitScaleFactor = 1.0;
  if (scene->mMetaData) {
    scene->mMetaData->Get<int>("UpAxis", upAxis);
    scene->mMetaData->Get<int>("UpAxisSign", upAxisSign);
    scene->mMetaData->Get<int>("FrontAxis", frontAxis);
    scene->mMetaData->Get<int>("FrontAxisSign", frontAxisSign);
    scene->mMetaData->Get<int>("CoordAxis", coordAxis);
    scene->mMetaData->Get<int>("CoordAxisSign", coordAxisSign);
    scene->mMetaData->Get<double>("UnitScaleFactor", unitScaleFactor);
  }

  aiVector3D upVec, forwardVec, rightVec;
  upVec[upAxis] = upAxisSign * (float)unitScaleFactor;
  forwardVec[frontAxis] = frontAxisSign * (float)unitScaleFactor;
  rightVec[coordAxis] = coordAxisSign * (float)unitScaleFactor;

  // aiMatrix4x4 mat(rightVec.x, rightVec.y, rightVec.z, 0.0f, upVec.x, upVec.y, upVec.z, 0.0f, forwardVec.x, forwardVec.y,
  //                forwardVec.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  // node->mTransformation = mat;

  std::list<aiNode *> queue;
  queue.push_back(scene->mRootNode);

  aiNode *node;
  while (!queue.empty()) {
    node = queue.front();
    queue.pop_front();

    printf("node %s has %d meshes and %d children \n", node->mName.C_Str(), node->mNumMeshes, node->mNumChildren);

    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
      const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
      const aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
      printf("  mesh %s (%p) has %d vertices and material index %d\n", mesh->mName.data, mesh, mesh->mNumVertices,
             mesh->mMaterialIndex);

      if (mesh->mNumVertices > 100000)
        warn(tr("mesh '%1' has more than 100'000 vertices, it is recommended to reduce the number of vertices.")
               .arg(mesh->mName.C_Str()));

      aiMatrix4x4 transform;
      aiNode *current = node;
      while (current != NULL) {
        transform *= current->mTransformation;
        current = current->mParent;
      }

      // compute absolute transform of this node from all the parents
      const int totalVertices = mesh->mNumVertices;
      const int totalFaces = mesh->mNumFaces;

      // create the arrays
      int currentCoordIndex = 0;
      double *const coordData = new double[3 * totalVertices];
      int currentNormalIndex = 0;
      double *const normalData = new double[3 * totalVertices];
      int currentTexCoordIndex = 0;
      double *const texCoordData = new double[2 * totalVertices];
      int currentIndexIndex = 0;
      unsigned int *const indexData = new unsigned int[3 * totalFaces];

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
        indexData[currentIndexIndex++] = face.mIndices[0];  // + indexOffset;
        indexData[currentIndexIndex++] = face.mIndices[1];  // + indexOffset;
        indexData[currentIndexIndex++] = face.mIndices[2];  // + indexOffset;
      }

      WbTriangleMesh *mTriangleMesh = new WbTriangleMesh();
      bool issueWarnings = true;  // TMP

      QString mTriangleMeshError =
        mTriangleMesh->init(coordData, normalData, texCoordData, indexData, totalVertices, currentIndexIndex);

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

    // add all the children of this node to the queue
    for (size_t i = 0; i < node->mNumChildren; ++i)
      queue.push_back(node->mChildren[i]);
  }

  /*
  for (unsigned int i = 0; i < node->mNumChildren; ++i) {
    printf("node %d (%s) has %d meshes (and %d children) || %d\n", i, node->mChildren[i]->mName.C_Str(),
           node->mChildren[i]->mNumMeshes, node->mChildren[i]->mNumChildren);
    if (node->mChildren[i]->HasMeshes())

    const aiMesh *mesh = scene->mMeshes[i];
    const aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
    printf("mesh %d: %s (%p) has %d vertices\n", i, mesh->mName.C_Str(), mesh, mesh->mNumVertices);
    printf("  material %d: (%p)\n", i, material);

    if (mesh->mNumVertices > 100000)
      warn(tr("mesh '%1' has more than 100'000 vertices, it is recommended to reduce the number of vertices.")
             .arg(mesh->mName.C_Str()));

    const int totalVertices = mesh->mNumVertices;
    const int totalFaces = mesh->mNumFaces;


    // create the arrays
    int currentCoordIndex = 0;
    double *const coordData = new double[3 * totalVertices];
    int currentNormalIndex = 0;
    double *const normalData = new double[3 * totalVertices];
    int currentTexCoordIndex = 0;
    double *const texCoordData = new double[2 * totalVertices];
    int currentIndexIndex = 0;
    unsigned int *const indexData = new unsigned int[3 * totalFaces];

    aiMatrix4x4 transform;
    aiNode *current = node;
    while (current != NULL) {
      transform *= current->mTransformation;
      current = current->mParent;
    }

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
  }
  */

  /*
  for (unsigned int i = 0; i < node->mNumChildren; ++i) {
    printf("node %d (%s) has %d meshes (and %d children) || %d\n", i, node->mChildren[i]->mName.C_Str(),
           node->mChildren[i]->mNumMeshes, node->mChildren[i]->mNumChildren);

    // count total number of vertices and faces of the current node

    int totalVertices = 0;
    int totalFaces = 0;

    for (unsigned int j = 0; j < node->mChildren[i]->mNumMeshes; ++j) {
      const aiMesh *mesh = scene->mMeshes[i];
      totalVertices += mesh->mNumVertices;
      totalFaces += mesh->mNumFaces;
    }

    if (node->mChildren[i]->mNumMeshes == 0)
      continue;

    for (unsigned int i = 0; i < node->mChildren[i]->mNumMeshes; ++i) {
      printf("  mesh %d ", i )
      const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
      const aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];

      if (mesh->mNumVertices > 100000)
        warn(tr("mesh '%1' has more than 100'000 vertices, it is recommended to reduce the number of vertices.")
               .arg(mesh->mName.C_Str()));
    }
  }
  */
}

void WbColladaShape::deleteWrenMeshes() {
  for (WrRenderable *renderable : mRenderables) {
    wr_material_delete(wr_renderable_get_material(renderable, "picking"));
    wr_node_delete(WR_NODE(renderable));
  }

  for (WrStaticMesh *mesh : mMeshes)
    wr_static_mesh_delete(mesh);

  for (WrMaterial *material : mMaterials)
    wr_material_delete(material);

  mRenderables.clear();
  mMeshes.clear();
  mMaterials.clear();
}

QString WbColladaShape::colladaPath() const {
  if (mUrl->value().isEmpty())
    return QString();
  return WbUrl::computePath(this, "url", mUrl->value());
}