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

#include "WbTextureTransform.hpp"
#include "WbMatrix4.hpp"
#include "WbSFDouble.hpp"
#include "WbWorld.hpp"

#include <wren/material.h>
#include <wren/texture.h>
#include <wren/texture_transform.h>

void WbTextureTransform::init() {
  mCenter = findSFVector2("center");
  mRotation = findSFDouble("rotation");
  mScale = findSFVector2("scale");
  mTranslation = findSFVector2("translation");

  mWrenTextureTransform = NULL;
}

void WbTextureTransform::destroyWrenObjects() {
  if (mWrenTextureTransform)
    wr_texture_transform_delete(mWrenTextureTransform);
}

WbTextureTransform::WbTextureTransform(WbTokenizer *tokenizer) : WbBaseNode("TextureTransform", tokenizer) {
  init();
}

WbTextureTransform::WbTextureTransform(const WbTextureTransform &other) : WbBaseNode(other) {
  init();
}

WbTextureTransform::WbTextureTransform(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbTextureTransform::~WbTextureTransform() {
  destroyWrenObjects();
}

void WbTextureTransform::preFinalize() {
  WbBaseNode::preFinalize();

  updateCenter();
  updateRotation();
  updateScale();
  updateTranslation();
}

void WbTextureTransform::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mCenter, &WbSFVector2::changed, this, &WbTextureTransform::updateCenter);
  connect(mRotation, &WbSFDouble::changed, this, &WbTextureTransform::updateRotation);
  connect(mScale, &WbSFVector2::changed, this, &WbTextureTransform::updateScale);
  connect(mTranslation, &WbSFVector2::changed, this, &WbTextureTransform::updateTranslation);

  if (!WbWorld::instance()->isLoading())
    emit changed();
}

void WbTextureTransform::updateCenter() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbTextureTransform::updateRotation() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbTextureTransform::updateScale() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbTextureTransform::updateTranslation() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbTextureTransform::modifyWrenMaterial(WrMaterial *wrenMaterial) {
  destroyWrenObjects();

  // apply translation before rotation
  mWrenTextureTransform = wr_texture_transform_new();
  wr_texture_transform_set_scale(mWrenTextureTransform, mScale->x(), mScale->y());
  wr_texture_transform_set_position(mWrenTextureTransform, mTranslation->x(), mTranslation->y());
  wr_texture_transform_set_center(mWrenTextureTransform, mCenter->x(), mCenter->y());
  wr_texture_transform_set_rotation(mWrenTextureTransform, mRotation->value());

  wr_material_set_texture_transform(wrenMaterial, mWrenTextureTransform);
}

WbVector2 WbTextureTransform::transformUVCoordinate(const WbVector2 &uv) const {
  if (mWrenTextureTransform) {
    float coord[2] = {static_cast<float>(uv.x()), static_cast<float>(uv.y())};
    wr_texture_transform_apply_to_uv_coordinate(mWrenTextureTransform, coord);
    return WbVector2(coord[0], coord[1]);
  }

  return uv;
}

void WbTextureTransform::translate(const WbVector2 &offset) {
  const WbVector2 value = mTranslation->value() + offset;
  const WbVector2 intpart((int)value.x(), (int)value.y());
  mTranslation->setValueFromWebots(value - intpart);
}

QStringList WbTextureTransform::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "center"
         << "rotation"
         << "scale"
         << "translation";
  return fields;
}
