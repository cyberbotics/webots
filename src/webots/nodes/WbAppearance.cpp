// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbAppearance.hpp"

#include "WbImageTexture.hpp"
#include "WbLight.hpp"
#include "WbMFNode.hpp"
#include "WbMaterial.hpp"
#include "WbPreferences.hpp"
#include "WbRgb.hpp"
#include "WbSFNode.hpp"
#include "WbTextureTransform.hpp"
#include "WbVector2.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>
#include <wren/shader_program.h>

void WbAppearance::init() {
  mMaterial = findSFNode("material");
  mTexture = findSFNode("texture");
}

WbAppearance::WbAppearance(WbTokenizer *tokenizer) : WbAbstractAppearance("Appearance", tokenizer) {
  init();
}

WbAppearance::WbAppearance(const WbAppearance &other) : WbAbstractAppearance(other) {
  init();
}

WbAppearance::WbAppearance(const WbNode &other) : WbAbstractAppearance(other) {
  init();
}

WbAppearance::~WbAppearance() {
}

void WbAppearance::downloadAssets() {
  WbBaseNode::downloadAssets();
  if (texture())
    texture()->downloadAssets();
}

void WbAppearance::preFinalize() {
  WbAbstractAppearance::preFinalize();

  if (material())
    material()->preFinalize();

  if (texture())
    texture()->preFinalize();

  updateMaterial();
  updateTexture();
}

void WbAppearance::postFinalize() {
  WbAbstractAppearance::postFinalize();

  if (material())
    material()->postFinalize();
  if (texture())
    texture()->postFinalize();

  connect(mMaterial, &WbSFNode::changed, this, &WbAppearance::updateMaterial);
  connect(mTexture, &WbSFNode::changed, this, &WbAppearance::updateTexture);
  if (!WbWorld::instance()->isLoading())
    emit changed();
}

void WbAppearance::reset(const QString &id) {
  WbAbstractAppearance::reset(id);

  WbNode *const m = mMaterial->value();
  if (m)
    m->reset(id);
  WbNode *const t = mTexture->value();
  if (t)
    t->reset(id);
}

void WbAppearance::updateMaterial() {
  if (material())
    connect(material(), &WbMaterial::changed, this, &WbAppearance::updateMaterial, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void WbAppearance::updateTexture() {
  if (texture())
    connect(texture(), &WbImageTexture::changed, this, &WbAppearance::updateTexture, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

WbMaterial *WbAppearance::material() const {
  return dynamic_cast<WbMaterial *>(mMaterial->value());
}

WbImageTexture *WbAppearance::texture() const {
  return dynamic_cast<WbImageTexture *>(mTexture->value());
}

void WbAppearance::createWrenObjects() {
  WbAbstractAppearance::createWrenObjects();

  if (material())
    material()->createWrenObjects();
  if (texture())
    texture()->createWrenObjects();
}

WrMaterial *WbAppearance::fillWrenDefaultMaterial(WrMaterial *wrenMaterial) {
  if (!wrenMaterial || wrenMaterial->type != WR_MATERIAL_PHONG) {
    wr_material_delete(wrenMaterial);
    wrenMaterial = wr_phong_material_new();
  }
  wr_material_set_default_program(wrenMaterial, WbWrenShaders::defaultShader());
  return wrenMaterial;
}

WrMaterial *WbAppearance::modifyWrenMaterial(WrMaterial *wrenMaterial) {
  if (!wrenMaterial || wrenMaterial->type != WR_MATERIAL_PHONG) {
    wr_material_delete(wrenMaterial);
    wrenMaterial = wr_phong_material_new();
  }

  if (material()) {
    wr_material_set_default_program(wrenMaterial, WbWrenShaders::phongShader());
    wr_material_set_stencil_ambient_emissive_program(wrenMaterial, WbWrenShaders::phongStencilAmbientEmissiveShader());
    wr_material_set_stencil_diffuse_specular_program(wrenMaterial, WbWrenShaders::phongStencilDiffuseSpecularShader());

    material()->modifyWrenMaterial(wrenMaterial, texture() && texture()->wrenTexture());
  } else
    wrenMaterial = fillWrenDefaultMaterial(wrenMaterial);

  if (texture())
    texture()->modifyWrenMaterial(wrenMaterial, 0, 2);
  else
    wr_material_set_texture(wrenMaterial, NULL, 0);

  if (textureTransform())
    textureTransform()->modifyWrenMaterial(wrenMaterial);
  else
    wr_material_set_texture_transform(wrenMaterial, NULL);

  return wrenMaterial;
}

bool WbAppearance::isTextureLoaded() const {
  return (texture() && texture()->wrenTexture());
}

WbRgb WbAppearance::diffuseColor() const {
  if (material() && !isTextureLoaded())
    return material()->diffuseColor();
  else
    return WbRgb(1.0f, 1.0f, 1.0f);
}

void WbAppearance::pickColorInTexture(const WbVector2 &uv, WbRgb &pickedColor) const {
  WbImageTexture *tex = texture();
  if (tex) {
    WbVector2 uvTransformed = transformUVCoordinate(uv);
    tex->pickColor(uvTransformed, pickedColor);
  } else
    pickedColor.setValue(1.0, 1.0, 1.0);  // default value
}
