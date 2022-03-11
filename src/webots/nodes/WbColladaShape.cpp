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
#include "WbAppearance.hpp"
#include "WbBoundingSphere.hpp"
#include "WbDownloader.hpp"
#include "WbSFString.hpp"
#include "WbUrl.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <QtCore/QFileInfo>

#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

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
    deleteWrenObjects();

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
  createWrenObjects();
  // update appearance?
  // WbWorld::instance()->viewpoint()->emit refreshRequired();
}

void WbColladaShape::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  deleteWrenObjects();  // TODO: create and delete?

  // Assimp::Importer importer;
  // importer.SetPropertyInteger(
  //  AI_CONFIG_PP_RVC_FLAGS,
  //  aiComponent_CAMERAS | aiComponent_LIGHTS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS);  // TODO: needed?

  // const aiScene *scene =
  //  importer.ReadFile(colladaPath().toStdString().c_str(), aiProcess_ValidateDataStructure | aiProcess_Triangulate |
  //                                                           aiProcess_JoinIdenticalVertices | aiProcess_RemoveComponent);

  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_CAMERAS | aiComponent_LIGHTS | aiComponent_BONEWEIGHTS |
                                                        aiComponent_ANIMATIONS | aiComponent_TEXTURES | aiComponent_COLORS);
  unsigned int flags = aiProcess_ValidateDataStructure | aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                       aiProcess_JoinIdenticalVertices | aiProcess_OptimizeGraph | aiProcess_RemoveComponent |
                       aiProcess_FlipUVs;
  const aiScene *scene = importer.ReadFile(colladaPath().toStdString().c_str(), flags);

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

  aiMatrix4x4 mat(rightVec.x, rightVec.y, rightVec.z, 0.0f, upVec.x, upVec.y, upVec.z, 0.0f, forwardVec.x, forwardVec.y,
                  forwardVec.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  scene->mRootNode->mTransformation = mat;

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
      float *const coordData = new float[3 * totalVertices];
      int currentNormalIndex = 0;
      float *const normalData = new float[3 * totalVertices];
      int currentTexCoordIndex = 0;
      float *const texCoordData = new float[2 * totalVertices];
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

      // const float arrowVertices[12] = {0.0f, 0.9f, 0.0f, -0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f, 0.0f};
      // const float arrowNormals[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      // const unsigned int arrowIndices[12] = {2, 1, 0, 0, 3, 2, 0, 1, 2, 2, 3, 0};
      // WrStaticMesh *staticMesh =
      //  wr_static_mesh_new(12, 12, arrowVertices, arrowNormals, arrowNormals, arrowNormals, arrowIndices, false);

      // TODO: vertex_count and index_count always equal? (totalVertices)
      // TODO: handle outline
      WrStaticMesh *staticMesh =
        wr_static_mesh_new(totalVertices, totalVertices, coordData, normalData, texCoordData, texCoordData, indexData, false);

      mWrenMeshes.push_back(staticMesh);

      delete[] coordData;
      delete[] normalData;
      delete[] texCoordData;
      delete[] indexData;
    }

    // add all the children of this node to the queue
    for (size_t i = 0; i < node->mNumChildren; ++i)
      queue.push_back(node->mChildren[i]);
  }

  printf("create WREN objects, size %lld\n", mWrenMeshes.size());
  for (int i = 0; i < mWrenMeshes.size(); ++i) {
    WrRenderable *renderable = wr_renderable_new();
    WrMaterial *material = wr_phong_material_new();  // WbAppearance::fillWrenDefaultMaterial(NULL);
    wr_renderable_set_material(renderable, material, NULL);
    wr_material_set_default_program(material, WbWrenShaders::lineSetShader());
    wr_renderable_set_mesh(renderable, WR_MESH(mWrenMeshes[i]));
    wr_renderable_set_receive_shadows(renderable, true);
    wr_renderable_set_cast_shadows(renderable, false);  // TODO: handle shadows, mCastShadows?
    wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext::VM_REGULAR);

    WrTransform *transform = wr_transform_new();
    wr_transform_attach_child(wrenNode(), WR_NODE(transform));
    setWrenNode(transform);
    wr_transform_attach_child(transform, WR_NODE(renderable));
    // wr_transform_set_scale(boneTransform, scale); // TODO: handle scale?
    wr_node_set_visible(WR_NODE(transform), true);

    // TODO: segmentation + rangefinder
    // TODO: pickable?
    // TODO: should be moved elsewhere
    mWrenMaterials.push_back(material);
    mWrenRenderables.push_back(renderable);
    mWrenTransforms.push_back(transform);
  }
}

void WbColladaShape::deleteWrenObjects() {
  printf("> delete wren meshes\n");
  for (WrRenderable *renderable : mWrenRenderables) {
    wr_material_delete(wr_renderable_get_material(renderable, "picking"));
    wr_node_delete(WR_NODE(renderable));
  }

  for (WrStaticMesh *mesh : mWrenMeshes)
    wr_static_mesh_delete(mesh);

  for (WrMaterial *material : mWrenMaterials)
    wr_material_delete(material);

  for (WrTransform *transform : mWrenTransforms)
    wr_node_delete(WR_NODE(transform));

  mWrenRenderables.clear();
  mWrenMeshes.clear();
  mWrenMaterials.clear();
  mWrenTransforms.clear();
}

QString WbColladaShape::colladaPath() const {
  if (mUrl->value().isEmpty())
    return QString();
  return WbUrl::computePath(this, "url", mUrl->value());
}