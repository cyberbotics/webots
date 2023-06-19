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

#include "WbLed.hpp"

#include "WbAppearance.hpp"
#include "WbField.hpp"
#include "WbGroup.hpp"
#include "WbLight.hpp"
#include "WbMFNode.hpp"
#include "WbMaterial.hpp"
#include "WbPbrAppearance.hpp"
#include "WbShape.hpp"

#include "../../controller/c/messages.h"  // contains the definitions for the macros C_SET_LED

#include <QtCore/QDataStream>
#include <cassert>

void WbLed::init() {
  mColor = findMFColor("color");
  mGradual = findSFBool("gradual");
  mValue = 0;
}

WbLed::WbLed(WbTokenizer *tokenizer) : WbSolidDevice("LED", tokenizer) {
  init();
}

WbLed::WbLed(const WbLed &other) : WbSolidDevice(other) {
  init();
}

WbLed::WbLed(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbLed::~WbLed() {
  clearMaterialsAndLights();
}

void WbLed::preFinalize() {
  WbSolidDevice::preFinalize();
}

void WbLed::postFinalize() {
  WbSolidDevice::postFinalize();

  updateChildren();

  // disable WREN lights if not on
  bool on = mValue != 0;
  foreach (WbLight *light, mLights)
    light->toggleOn(on);
}

void WbLed::reset(const QString &id) {
  WbSolidDevice::reset(id);
  mValue = 0;
  foreach (WbLight *light, mLights)
    light->toggleOn(false);
  foreach (WbMaterial *material, mMaterials)
    material->setEmissiveColor(WbRgb(0.0, 0.0, 0.0));
  foreach (WbPbrAppearance *pbrAppearance, mPbrAppearances)
    pbrAppearance->setEmissiveColor(WbRgb(0.0, 0.0, 0.0));
}

void WbLed::updateIfNeeded(WbField *field) {
  if (field->name() == "material" || field->name() == "appearance")
    updateChildren();
}

void WbLed::updateChildren() {
  WbSolid::updateChildren();

  if (!isPostFinalizedCalled())
    return;

  findMaterialsAndLights(this);
  if (mGradual->isTrue() && colorsCount() > 1)
    parsingWarn(tr("Too many colors defined for a gradual LED."));

  // update color of lights and materials
  setMaterialsAndLightsColor();
}

void WbLed::findMaterialsAndLights(const WbGroup *group) {
  int size = group->children().size();
  if (size < 1) {
    clearMaterialsAndLights();
    return;
  }

  if (group == this) {
    clearMaterialsAndLights();
    size = 1;  // we look only into the first child of the WbLed node
  }

  for (int i = 0; i < size; ++i) {
    WbBaseNode *const n = group->child(i);
    WbLight *lightChild = dynamic_cast<WbLight *>(n);
    WbGroup *groupChild = dynamic_cast<WbGroup *>(n);

    if (n->nodeType() == WB_NODE_SHAPE) {
      WbAppearance *appearance = dynamic_cast<WbShape *>(n)->appearance();
      if (appearance) {
        WbMaterial *material = appearance->material();
        if (material)
          mMaterials.append(material);

        connect(appearance, &WbAppearance::fieldChanged, this, &WbLed::updateIfNeeded, Qt::UniqueConnection);
        connect(appearance->parentNode(), &WbShape::fieldChanged, this, &WbLed::updateIfNeeded, Qt::UniqueConnection);
      } else {
        WbPbrAppearance *pbrAppearance = dynamic_cast<WbShape *>(n)->pbrAppearance();
        if (pbrAppearance) {
          mPbrAppearances.append(pbrAppearance);

          connect(pbrAppearance, &WbPbrAppearance::fieldChanged, this, &WbLed::updateIfNeeded, Qt::UniqueConnection);
          connect(pbrAppearance->parentNode(), &WbShape::fieldChanged, this, &WbLed::updateIfNeeded, Qt::UniqueConnection);
        }
      }
    } else if (lightChild)
      mLights.append(lightChild);
    else if (groupChild) {
      findMaterialsAndLights(groupChild);
      connect(groupChild, &WbGroup::childrenChanged, this, &WbLed::updateChildren, Qt::UniqueConnection);
    }
  }

  if (group == this && !isAnyMaterialOrLightFound())
    parsingWarn(tr("No PBRAppearance, Material and no Light found. "
                   "The first child of a LED should be either a Shape, a Light "
                   "or a Group containing Shape and Light nodes."));
}

bool WbLed::isAnyMaterialOrLightFound() {
  return (mMaterials.size() > 0 || mLights.size() > 0 || mPbrAppearances.size() > 0);
}

void WbLed::handleMessage(QDataStream &stream) {
  unsigned char byte;
  int v;
  stream >> byte;

  switch (byte) {
    case C_LED_SET:
      stream >> v;
      if (isAnyMaterialOrLightFound())
        setValue(v);
      break;
    default:
      assert(0);
  }
}

bool WbLed::isGradual() const {
  return mGradual->value();
}

void WbLed::setValue(int value) {
  if (mValue == value)
    return;
  mValue = value;

  setMaterialsAndLightsColor();
}

void WbLed::setMaterialsAndLightsColor() {
  // compute the new color
  float r, g, b;
  if (mGradual->isTrue()) {
    if (mColor->isEmpty()) {  // RGB case
      r = (float)((mValue >> 16) & 0xff) / 255.0f;
      g = (float)((mValue >> 8) & 0xff) / 255.0f;
      b = (float)(mValue & 0xff) / 255.0f;
    } else {  // monochrome case
      float i = (float)mValue / 255.0f;
      const WbRgb &c = mColor->item(0);
      r = i * c.red();
      g = i * c.green();
      b = i * c.blue();
    }
  } else {
    if (mValue > 0) {
      if (mValue - 1 < mColor->size()) {
        const WbRgb &c = mColor->item(mValue - 1);
        r = c.red();
        g = c.green();
        b = c.blue();
      } else {
        r = 0.0f;
        g = 0.0f;
        b = 0.0f;
      }
    } else {
      r = 0.0f;
      g = 0.0f;
      b = 0.0f;
    }
  }

  assert(r >= 0 && r <= 1 && g >= 0 && g <= 1 && b >= 0 && b <= 1);
  WbRgb lightColor(r, g, b);

  // update every material
  foreach (WbMaterial *material, mMaterials)
    material->setEmissiveColor(lightColor);

  // same for PbrAppearances
  foreach (WbPbrAppearance *pbrAppearance, mPbrAppearances)
    pbrAppearance->setEmissiveColor(lightColor);

  // update every lights
  const bool on = mValue != 0;
  foreach (WbLight *light, mLights) {
    light->setColor(lightColor);
    // disable WREN lights if not on
    light->toggleOn(on);
  }
}

void WbLed::powerOn(bool e) {  // turn off
  WbDevice::powerOn(e);
  if (!e)
    setValue(0);
}
