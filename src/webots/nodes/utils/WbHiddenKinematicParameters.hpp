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

#ifndef WB_HIDDEN_KINEMATIC_PARAMETERS_HPP
#define WB_HIDDEN_KINEMATIC_PARAMETERS_HPP

#include "WbRotation.hpp"
#include "WbVector3.hpp"

#include <QtCore/QMap>

class WbField;

namespace WbHiddenKinematicParameters {

  typedef QMap<int, WbVector3 *> PositionMap;

  class HiddenKinematicParameters {
  public:
    HiddenKinematicParameters() :
      mTranslation(NULL),
      mRotation(NULL),
      mPositions(NULL),
      mLinearVelocity(NULL),
      mAngularVelocity(NULL),
      mTranslationIsCreated(false),
      mRotationIsCreated(false) {}
    HiddenKinematicParameters(const WbVector3 *t, const WbRotation *r, PositionMap *p, const WbVector3 *l, const WbVector3 *a) :
      mTranslation(t),
      mRotation(r),
      mPositions(p),
      mLinearVelocity(l),
      mAngularVelocity(a),
      mTranslationIsCreated(false),
      mRotationIsCreated(false) {}
    ~HiddenKinematicParameters() {
      if (mPositions)
        qDeleteAll(*mPositions);
      delete mPositions;
      delete mAngularVelocity;
      delete mLinearVelocity;
      if (mTranslationIsCreated)
        delete mTranslation;
      if (mRotationIsCreated)
        delete mRotation;
    }
    const WbVector3 *translation() const { return mTranslation; }
    const WbRotation *rotation() const { return mRotation; }
    const PositionMap *positions() const { return mPositions; }
    const WbVector3 *linearVelocity() const { return mLinearVelocity; }
    const WbVector3 *angularVelocity() const { return mAngularVelocity; }
    void createTranslation(double x, double y, double z) {
      delete mTranslation;
      mTranslation = new WbVector3(x, y, z);
      mTranslationIsCreated = true;
    }
    void createRotation(double x, double y, double z, double angle) {
      delete mRotation;
      mRotation = new WbRotation(x, y, z, angle);
      mRotationIsCreated = true;
    }
    void createLinearVelocity(double x, double y, double z) {
      delete mLinearVelocity;
      mLinearVelocity = new WbVector3(x, y, z);
    }
    void createAngularVelocity(double x, double y, double z) {
      delete mAngularVelocity;
      mAngularVelocity = new WbVector3(x, y, z);
    }
    void insertPositions(int index, WbVector3 *positions) {
      if (mPositions == NULL)
        mPositions = new PositionMap;
      mPositions->insert(index, positions);
    }
    WbVector3 *positions(int index) { return mPositions ? mPositions->value(index) : NULL; }

  private:
    const WbVector3 *mTranslation;
    const WbRotation *mRotation;
    PositionMap *mPositions;
    const WbVector3 *mLinearVelocity;
    const WbVector3 *mAngularVelocity;
    bool mTranslationIsCreated;
    bool mRotationIsCreated;
  };

  typedef QMap<int, HiddenKinematicParameters *> HiddenKinematicParametersMap;
  void createHiddenKinematicParameter(WbField *field, HiddenKinematicParametersMap &map);
};  // namespace WbHiddenKinematicParameters

#endif  // WB_HIDDEN_KINEMATIC_PARAMETERS_HPP
