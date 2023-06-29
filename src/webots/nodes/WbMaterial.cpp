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

#include "WbMaterial.hpp"

#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/material.h>

void WbMaterial::init() {
  mInitialEmissiveColor = WbRgb();

  mAmbientIntensity = findSFDouble("ambientIntensity");
  mDiffuseColor = findSFColor("diffuseColor");
  mEmissiveColor = findSFColor("emissiveColor");
  mShininess = findSFDouble("shininess");
  mSpecularColor = findSFColor("specularColor");
  mTransparency = findSFDouble("transparency");
}

WbMaterial::WbMaterial(WbTokenizer *tokenizer) : WbBaseNode("Material", tokenizer) {
  init();
}

WbMaterial::WbMaterial(const WbMaterial &other) : WbBaseNode(other) {
  init();
}

WbMaterial::WbMaterial(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbMaterial::~WbMaterial() {
}

void WbMaterial::preFinalize() {
  WbBaseNode::preFinalize();

  updateAmbientIntensity();
  updateDiffuseColor();
  updateEmissiveColor();
  updateShininess();
  updateSpecularColor();
  updateTransparency();

  mInitialEmissiveColor = mEmissiveColor->value();
}

void WbMaterial::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mAmbientIntensity, &WbSFDouble::changed, this, &WbMaterial::updateAmbientIntensity);
  connect(mDiffuseColor, &WbSFColor::changed, this, &WbMaterial::updateDiffuseColor);
  connect(mEmissiveColor, &WbSFColor::changed, this, &WbMaterial::updateEmissiveColor);
  connect(mShininess, &WbSFDouble::changed, this, &WbMaterial::updateShininess);
  connect(mSpecularColor, &WbSFColor::changed, this, &WbMaterial::updateSpecularColor);
  connect(mTransparency, &WbSFDouble::changed, this, &WbMaterial::updateTransparency);

  if (!WbWorld::instance()->isLoading())
    emit changed();
}

float WbMaterial::transparency() const {
  return mTransparency->value();
}

const WbRgb &WbMaterial::emissiveColor() const {
  return mEmissiveColor->value();
}

void WbMaterial::setEmissiveColor(const WbRgb &color) {
  mEmissiveColor->setValue(color);
}

const WbRgb &WbMaterial::diffuseColor() const {
  return mDiffuseColor->value();
}

void WbMaterial::updateAmbientIntensity() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mAmbientIntensity, 0.0, 1.0, 0.5))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbMaterial::updateDiffuseColor() {
  if (WbFieldChecker::resetColorIfInvalid(this, mDiffuseColor))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbMaterial::updateEmissiveColor() {
  if (WbFieldChecker::resetColorIfInvalid(this, mEmissiveColor))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbMaterial::updateShininess() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mShininess, 0.0, 1.0, 0.5))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbMaterial::updateSpecularColor() {
  if (WbFieldChecker::resetColorIfInvalid(this, mSpecularColor))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbMaterial::updateTransparency() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransparency, 0.0, 1.0, 0.5))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbMaterial::modifyWrenMaterial(WrMaterial *wrenMaterial, bool textured) {
  WbRgb ambient, diffuse, specular, emissive;
  float ambientIntensity, shininess;

  ambientIntensity = mAmbientIntensity->value();
  ambient.setValue(ambientIntensity, ambientIntensity, ambientIntensity);
  emissive = mEmissiveColor->value();

  if (textured) {
    diffuse = WbRgb(1.0f, 1.0f, 1.0f);
    specular = WbRgb();
    shininess = 0.0f;
  } else {
    ambient.setValue(ambientIntensity * mDiffuseColor->red(), ambientIntensity * mDiffuseColor->green(),
                     ambientIntensity * mDiffuseColor->blue());
    diffuse.setValue(mDiffuseColor->red(), mDiffuseColor->green(), mDiffuseColor->blue());
    specular.setValue(mSpecularColor->red(), mSpecularColor->green(), mSpecularColor->blue());
    shininess = mShininess->value();
  }

  const float newAmbientColor[] = {static_cast<float>(ambient.red()), static_cast<float>(ambient.green()),
                                   static_cast<float>(ambient.blue())};

  const float newDiffuseColor[] = {static_cast<float>(diffuse.red()), static_cast<float>(diffuse.green()),
                                   static_cast<float>(diffuse.blue())};

  const float newSpecularColor[] = {static_cast<float>(specular.red()), static_cast<float>(specular.green()),
                                    static_cast<float>(specular.blue())};

  const float newEmissiveColor[] = {static_cast<float>(emissive.red()), static_cast<float>(emissive.green()),
                                    static_cast<float>(emissive.blue())};

  wr_phong_material_set_all_parameters(wrenMaterial, newAmbientColor, newDiffuseColor, newSpecularColor, newEmissiveColor,
                                       shininess, mTransparency->value());
}

QStringList WbMaterial::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "ambientIntensity"
         << "shininess"
         << "specularColor"
         << "transparency"
         << "emissiveColor"
         << "diffuseColor";
  return fields;
}
