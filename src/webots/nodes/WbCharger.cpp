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

#include "WbCharger.hpp"

#include "WbAppearance.hpp"
#include "WbField.hpp"
#include "WbLight.hpp"
#include "WbLog.hpp"
#include "WbMFDouble.hpp"
#include "WbMFNode.hpp"
#include "WbMaterial.hpp"
#include "WbPbrAppearance.hpp"
#include "WbRobot.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbShape.hpp"
#include "WbVrmlNodeUtilities.hpp"
#include "WbWorld.hpp"

struct VisualElement {
  VisualElement(WbBaseNode *n, double r, double g, double b) {
    node = n;
    initialRed = r;
    initialGreen = g;
    initialBlue = b;
  }
  WbBaseNode *node;
  double initialRed;
  double initialGreen;
  double initialBlue;
};

void WbCharger::init() {
  mBattery = findMFDouble("battery");
  mRadius = findSFDouble("radius");
  mEmissiveColor = findSFColor("emissiveColor");
  mGradual = findSFBool("gradual");
  mParentRobot = NULL;
  mRobot = NULL;
  mElementsUpdateRequired = true;
  if (mBattery->size() > CURRENT_ENERGY)
    mSavedEnergies[stateId()] = mBattery->item(CURRENT_ENERGY);
}

WbCharger::WbCharger(WbTokenizer *tokenizer) : WbSolid("Charger", tokenizer) {
  init();
}

WbCharger::WbCharger(const WbCharger &other) : WbSolid(other) {
  init();
}

WbCharger::WbCharger(const WbNode &other) : WbSolid(other) {
  init();
}

WbCharger::~WbCharger() {
}

void WbCharger::postFinalize() {
  WbSolid::postFinalize();

  const WbNode *topNode = WbVrmlNodeUtilities::findTopNode(this);
  mParentRobot = dynamic_cast<const WbRobot *>(topNode);
}

void WbCharger::clearMaterialsAndLights() {
  foreach (VisualElement *visualElement, mVisualElements)
    delete visualElement;
  mVisualElements.clear();
}

void WbCharger::updateMaterialsAndLights(double batteryRatio) {
  foreach (VisualElement *const visualElement, mVisualElements) {
    // compute the color of the indicators
    float cr, cg, cb;
    if (batteryRatio == 1.0) {
      cr = mEmissiveColor->red();
      cg = mEmissiveColor->green();
      cb = mEmissiveColor->blue();
    } else if (mGradual->value()) {
      cr = (mEmissiveColor->red() - visualElement->initialRed) * batteryRatio + visualElement->initialRed;
      cg = (mEmissiveColor->green() - visualElement->initialGreen) * batteryRatio + visualElement->initialGreen;
      cb = (mEmissiveColor->blue() - visualElement->initialBlue) * batteryRatio + visualElement->initialBlue;
    } else {
      cr = visualElement->initialRed;
      cg = visualElement->initialGreen;
      cb = visualElement->initialBlue;
    }
    WbMaterial *material = dynamic_cast<WbMaterial *>(visualElement->node);
    WbPbrAppearance *appearance = dynamic_cast<WbPbrAppearance *>(visualElement->node);
    WbLight *light = dynamic_cast<WbLight *>(visualElement->node);
    const WbRgb color(cr, cg, cb);
#ifndef NDEBUG
    const bool clampNeeded = WbRgb(cr, cg, cb).clampValuesIfNeeded();
    assert(!clampNeeded);
#endif
    if (material)
      material->setEmissiveColor(color);
    else if (appearance)
      appearance->setEmissiveColor(color);
    else if (light)
      light->setColor(color);
  }
}

bool WbCharger::isAnyMaterialOrLightFound() const {
  return mVisualElements.size() > 0;
}

void WbCharger::findMaterialsAndLights(const WbGroup *const g) {
  int size = g->children().size();
  if (size < 1) {
    clearMaterialsAndLights();
    return;
  }

  if (g == this) {
    clearMaterialsAndLights();
    size = 1;  // we look only into the first child of the WbCharger node
  }

  for (int i = 0; i < size; ++i) {
    WbBaseNode *const n = g->child(i);
    const WbShape *const shape = dynamic_cast<WbShape *>(n);
    WbLight *const light = dynamic_cast<WbLight *>(n);
    const WbGroup *const group = dynamic_cast<WbGroup *>(n);
    if (shape) {
      const WbAppearance *const appearance = shape->appearance();
      WbPbrAppearance *const pbrAppearance = shape->pbrAppearance();
      if (appearance) {
        connect(appearance, &WbAppearance::destroyed, this, &WbCharger::updateElementsWhenRequired, Qt::UniqueConnection);
        connect(appearance, &WbAppearance::changed, this, &WbCharger::updateElementsWhenRequired, Qt::UniqueConnection);
        WbMaterial *const material = appearance->material();
        if (material) {
          connect(material, &WbMaterial::destroyed, this, &WbCharger::updateElementsWhenRequired, Qt::UniqueConnection);
          mVisualElements.append(new VisualElement(material, material->initialEmissiveColor().red(),
                                                   material->initialEmissiveColor().green(),
                                                   material->initialEmissiveColor().blue()));
        }
      } else if (pbrAppearance) {
        connect(pbrAppearance, &WbMaterial::destroyed, this, &WbCharger::updateElementsWhenRequired, Qt::UniqueConnection);
        mVisualElements.append(new VisualElement(pbrAppearance, pbrAppearance->initialEmissiveColor().red(),
                                                 pbrAppearance->initialEmissiveColor().green(),
                                                 pbrAppearance->initialEmissiveColor().blue()));
      }
    } else if (light) {
      connect(light, &WbLight::destroyed, this, &WbCharger::updateElementsWhenRequired, Qt::UniqueConnection);
      mVisualElements.append(
        new VisualElement(light, light->initialColor().red(), light->initialColor().green(), light->initialColor().blue()));
    } else if (group) {
      connect(group, &WbGroup::childAdded, this, &WbCharger::updateElementsWhenRequired, Qt::UniqueConnection);
      connect(group, &WbGroup::destroyed, this, &WbCharger::updateElementsWhenRequired, Qt::UniqueConnection);
      findMaterialsAndLights(group);
    }
  }
  if (g == this && !isAnyMaterialOrLightFound()) {
    parsingWarn(tr("No Material and no Light found. "
                   "The first child of a Charger should be either a Shape, a Light "
                   "or a Group containing Shape and Light nodes."));
  }
}

void WbCharger::prePhysicsStep(double ms) {
  WbSolid::prePhysicsStep(ms);

  if (mBattery->size() < 3)
    return;

  if (mElementsUpdateRequired)
    findMaterialsAndLights(this);

  const QList<WbRobot *> &robots = WbWorld::instance()->robots();
  foreach (WbRobot *const robot, robots)
    checkContact(robot);

  const double currentEnergy = mBattery->item(CURRENT_ENERGY);
  const double maxEnergy = mBattery->item(MAX_ENERGY);
  const double energyUploadSpeed = mBattery->item(ENERGY_UPLOAD_SPEED);
  double newEnergy = currentEnergy;

  // The Charger collects energy from the Nature (Sun, Earth, Water, etc.)
  if (mRobot == NULL || mRobot->battery().size() < 3) {
    newEnergy += energyUploadSpeed * ms / 1000;
    if (newEnergy > maxEnergy)
      newEnergy = maxEnergy;
  } else {  // exchange of energy from the Charger to the Robot
    double e = (mRobot->energyUploadSpeed() * ms) / 1000.0;
    if (e > currentEnergy) {  // robot cannot take more than available
      e = currentEnergy;
      newEnergy = 0.0;
    } else
      newEnergy = currentEnergy - e;  // transfer

    double robotCurrentEnergy = mRobot->currentEnergy();
    // special case:
    //   if the current energy of the robot is already bigger that its max energy
    //   the robot battery cannot be filled
    if (robotCurrentEnergy >= mRobot->maxEnergy())
      newEnergy = currentEnergy;  // no energy is transferred
    else {
      robotCurrentEnergy += e;
      if (robotCurrentEnergy > mRobot->maxEnergy())
        robotCurrentEnergy = mRobot->maxEnergy();
    }
    mRobot->setCurrentEnergy(robotCurrentEnergy);
  }

  // store value in battery
  mBattery->setItem(CURRENT_ENERGY, newEnergy);

  // energy level
  const double r = currentEnergy / maxEnergy;
  updateMaterialsAndLights(r);
}

void WbCharger::checkContact(WbRobot *const r) {
  if (mParentRobot && mParentRobot == r)
    // do not charge itself
    return;

  if (mRobot && mRobot != r)
    return;  // Charger is already busy

  const WbVector3 &dist = matrix().translation() - r->translation();
  const double norm2 = dist.length2();
  double r2 = mRadius->value();
  if (mRobot)
    r2 *= 1.1;  // tolerance to maintain contact
  r2 *= r2;
  // printf("range^2: %g <= %g\n", norm2, r2);
  if (norm2 > r2)
    // current robot is leaving
    mRobot = NULL;
  else if (mRobot == NULL) {
    mRobot = r;
    // now Charger is busy with that robot...
    // printf("found one robot %p\n", (void *)robot);
  }
}

void WbCharger::reset(const QString &id) {
  WbSolid::reset(id);
  mRobot = NULL;
  if (mBattery->size() > CURRENT_ENERGY)
    mBattery->setItem(CURRENT_ENERGY, mSavedEnergies[id]);
  if (mBattery->size() > MAX_ENERGY)
    updateMaterialsAndLights(mBattery->item(CURRENT_ENERGY) / mBattery->item(MAX_ENERGY));
}

void WbCharger::save(const QString &id) {
  WbSolid::save(id);
  if (mBattery->size() > CURRENT_ENERGY)
    mSavedEnergies[id] = mBattery->item(CURRENT_ENERGY);
}
