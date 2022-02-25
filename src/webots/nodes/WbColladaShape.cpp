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

  const aiNode *node = scene->mRootNode;

  if (node->mNumChildren > 0) {
    for (unsigned int i = 0; i < node->mNumChildren; ++i) {
      printf("found: %d meshes\n", node->mChildren[i]->mNumMeshes);
      for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
        const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
        const aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];

        if (mesh->mNumVertices > 100000)
          warn(tr("mesh '%1' has more than 100'000 vertices, it is recommended to reduce the number of vertices.")
                 arg(mesh->mName.C_Str()));
      }
    }
  }
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