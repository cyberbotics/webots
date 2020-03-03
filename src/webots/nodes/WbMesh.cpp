// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "WbAffinePlane.hpp"
#include "WbBoundingSphere.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbMFString.hpp"
#include "WbNodeUtilities.hpp"
#include "WbRay.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSimulationState.hpp"
#include "WbTransform.hpp"
#include "WbTriangleMesh.hpp"
#include "WbUrl.hpp"
#include "WbVector2.hpp"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <cmath>

void WbMesh::init() {
  mUrl = findMFString("url");

  mResizeConstraint = WbWrenAbstractResizeManipulator::UNIFORM;
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
  wr_static_mesh_delete(mWrenMesh);
}

void WbMesh::preFinalize() {
  WbTriangleMeshGeometry::preFinalize();

  updateUrl();
}

void WbMesh::postFinalize() {
  WbTriangleMeshGeometry::postFinalize();

  connect(mUrl, &WbMFString::changed, this, &WbMesh::updateUrl);
}

void WbMesh::createResizeManipulator() {
  mResizeManipulator = new WbRegularResizeManipulator(uniqueId(), WbWrenAbstractResizeManipulator::ResizeConstraint::X_EQUAL_Z);
}

void WbMesh::updateTriangleMesh(bool issueWarnings) {
  const QString filePath(path());
  if (filePath.isEmpty())
    return;

  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(
    filePath.toStdString().c_str(), aiProcess_ValidateDataStructure | aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                                      aiProcess_JoinIdenticalVertices | aiProcess_OptimizeGraph
    /* |  |
           | aiProcess_TransformUVCoords |
          aiProcess_FlipUVs */);

  if (!scene) {
    warn(tr("Invalid data, please verify mesh file (bone weights, normals, ...): %1").arg(importer.GetErrorString()));
    return;
  } else if (!scene->HasMeshes()) {
    warn(tr("This file doesn't contain any mesh."));
    return;
  }

  // count total number of vertices and faces
  int totalVertices = 0;
  int totalFaces = 0;
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    totalVertices += scene->mMeshes[i]->mNumVertices;
    totalFaces += scene->mMeshes[i]->mNumFaces;
  }
  // create arrays
  int current_coord_index = 0;
  double coord_data[3 * totalVertices];
  int current_normal_index = 0;
  double normal_data[3 * totalVertices];
  int current_tex_coord_index = 0;
  double tex_coord_data[2 * totalVertices];
  int current_index_index = 0;
  unsigned int index_data[3 * totalFaces];

  // global transformation
  aiMatrix4x4 globalInvTransform = scene->mRootNode->mTransformation;
  globalInvTransform.Inverse();

  // loop over all the node to find meshes
  std::list<aiNode *> queue;
  queue.push_back(scene->mRootNode);
  aiNode *node = NULL;
  while (!queue.empty()) {
    node = queue.front();
    queue.pop_front();

    // compute absolute transform of this node from all the parent
    aiMatrix4x4 transform = node->mTransformation;
    aiNode *current = node->mParent;
    while (current != scene->mRootNode && current != NULL) {
      transform = transform * node->mTransformation;
      current = current->mParent;
    }
    transform = (globalInvTransform * transform).Transpose();

    // merge all the meshes of this node
    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
      const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];

      for (size_t j = 0; j < mesh->mNumVertices; ++j) {
        const aiVector3D vertice = transform * mesh->mVertices[j];
        coord_data[current_coord_index++] = vertice[0];
        coord_data[current_coord_index++] = vertice[1];
        coord_data[current_coord_index++] = vertice[2];
        const aiVector3D normal = transform * mesh->mNormals[j];
        normal_data[current_normal_index++] = normal[0];
        normal_data[current_normal_index++] = normal[1];
        normal_data[current_normal_index++] = normal[2];
        if (mesh->HasTextureCoords(0)) {
          tex_coord_data[current_tex_coord_index++] = mesh->mTextureCoords[0][j].x;
          tex_coord_data[current_tex_coord_index++] = mesh->mTextureCoords[0][j].y;
        } else {
          // TODO: what should we do if not defined?
          tex_coord_data[current_tex_coord_index++] = 0.5;
          tex_coord_data[current_tex_coord_index++] = 0.5;
        }
      }

      for (size_t j = 0; j < mesh->mNumFaces; ++j) {
        const aiFace face = mesh->mFaces[j];
        assert(face.mNumIndices == 3);
        index_data[current_index_index++] = face.mIndices[0];
        index_data[current_index_index++] = face.mIndices[1];
        index_data[current_index_index++] = face.mIndices[2];
      }
    }

    // add all the childrens of this node to the queue
    for (size_t i = 0; i < node->mNumChildren; ++i)
      queue.push_back(node->mChildren[i]);
  }

  if (!node) {
    warn(tr("This file doesn't contain any mesh."));
    return;
  }

  mTriangleMeshError = mTriangleMesh->init(coord_data, normal_data, tex_coord_data, index_data, totalVertices, 3 * totalFaces);

  if (issueWarnings) {
    foreach (QString warning, mTriangleMesh->warnings())
      warn(warning);

    if (!mTriangleMeshError.isEmpty())
      warn(tr("Cannot create IndexedFaceSet because: \"%1\".").arg(mTriangleMeshError));
  }
}

uint64_t WbMesh::computeHash() const {
  const QByteArray meshPath = path().toUtf8();
  return WbTriangleMeshCache::sipHash13x(meshPath.constData(), meshPath.size());
}

void WbMesh::rescale(const WbVector3 &scale) {
}

void WbMesh::updateUrl() {
  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  int n = mUrl->size();
  for (int i = 0; i < n; i++) {
    QString item = mUrl->item(i);
    mUrl->setItem(i, item.replace("\\", "/"));
  }

  if (areWrenObjectsInitialized())
    buildWrenMesh(true);

  if (isPostFinalizedCalled())
    emit changed();
}

QString WbMesh::path() const {
  return WbUrl::computePath(this, "url", mUrl, 0);
}
