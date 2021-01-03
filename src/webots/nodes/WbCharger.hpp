// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_CHARGER_HPP
#define WB_CHARGER_HPP

#include <QtCore/QList>
#include "WbSolid.hpp"

class WbGroup;
class WbMFDouble;
class WbRobot;
class WbSFBool;
class WbSFColor;
class WbSFDouble;

struct VisualElement;

class WbCharger : public WbSolid {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbCharger(WbTokenizer *tokenizer = NULL);
  WbCharger(const WbCharger &other);
  explicit WbCharger(const WbNode &other);
  virtual ~WbCharger();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CHARGER; }
  void postFinalize() override;
  void prePhysicsStep(double) override;
  void checkContact(WbRobot *const r);
  void reset() override;
  void save() override;
  enum { CURRENT_ENERGY = 0, MAX_ENERGY = 1, ENERGY_UPLOAD_SPEED = 2 };

private:
  // user accessible fields
  WbMFDouble *mBattery;
  WbSFDouble *mRadius;
  WbSFColor *mEmissiveColor;
  WbSFBool *mGradual;

  // private fields
  const WbRobot *mParentRobot;
  WbRobot *mRobot;  // robot currently connected to the Charger
  bool mDone;
  bool mElementsUpdateRequired;
  QList<VisualElement *> mVisualElements;
  double mInitialEnergy;

  WbCharger &operator=(const WbCharger &);  // non copyable
  WbNode *clone() const override { return new WbCharger(*this); }
  void init();
  bool isAnyMaterialOrLightFound() const;
  void findMaterialsAndLights(const WbGroup *const g);
  void clearMaterialsAndLights();
  void updateMaterialsAndLights(double batteryRatio);

private slots:
  void updateElementsWhenRequired() { mElementsUpdateRequired = true; }
};

#endif
