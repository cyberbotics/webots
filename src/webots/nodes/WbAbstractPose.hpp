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

#ifndef WB_ABSTRACT_POSE_HPP
#define WB_ABSTRACT_POSE_HPP

//
// Description: abstract node implementing 'translation', 'rotation' fields
//              functionalities and keeping the WREN::SceneNode up-to-date
//
// Inherited by: WbPose and WbSkin
//

#include <cassert>
#include "WbMatrix3.hpp"
#include "WbMatrix4.hpp"
#include "WbSFDouble.hpp"
#include "WbSFRotation.hpp"
#include "WbSFVector3.hpp"

class WbBaseNode;
class WbTranslateRotateManipulator;

class WbAbstractPose {
public:
  virtual ~WbAbstractPose();

  virtual WbBaseNode *baseNode() const { return mBaseNode; }

  // field accessors
  const WbVector3 &translation() const { return mTranslation->value(); }
  const WbRotation &rotation() const { return mRotation->value(); }
  WbSFVector3 *translationFieldValue() const { return mTranslation; }
  WbSFRotation *rotationFieldValue() const { return mRotation; }

  double translationStep() const { return mTranslationStep->value(); }
  double rotationStep() const { return mRotationStep->value(); }

  // setters
  void setTranslationAndRotation(double tx, double ty, double tz, double rx, double ry, double rz, double angle);

  void setTranslationAndRotation(const WbVector3 &v, const WbRotation &r);
  void translate(double tx, double ty, double tz) {
    setTranslation(mTranslation->x() + tx, mTranslation->y() + ty, mTranslation->z() + tz);
  }
  void translate(const WbVector3 &v) { translate(v.x(), v.y(), v.z()); }
  void setTranslation(double tx, double ty, double tz);
  void setTranslationFromOde(double tx, double ty, double tz) { mTranslation->setValueFromOde(tx, ty, tz); }
  void setTranslation(const WbVector3 &v) { setTranslation(v.x(), v.y(), v.z()); }
  void setTranslationFromOde(const WbVector3 &v) { mTranslation->setValueFromOde(v.x(), v.y(), v.z()); }
  void rotate(const WbVector3 &v);
  void setRotation(double x, double y, double z, double angle);
  void setRotation(const WbRotation &r);
  void setRotationFromOde(const WbRotation &r) { mRotation->setValueFromOde(r); }
  void setRotationAngle(double angle);

  // 4x4 transform matrices
  const WbMatrix4 &matrix() const;
  virtual const WbMatrix4 &vrmlMatrix() const;
  WbVector3 xAxis() const { return matrix().xAxis(); }
  WbVector3 yAxis() const { return matrix().yAxis(); }
  WbVector3 zAxis() const { return matrix().zAxis(); }

  // 3x3 absolute rotation matrix
  WbMatrix3 rotationMatrix() const;

  const WbQuaternion &relativeQuaternion() const { return mRelativeQuaternion; }

  // position in 'world' coordinates
  WbVector3 position() const { return matrix().translation(); }

  // translate-rotate manipulator
  WbTranslateRotateManipulator *translateRotateManipulator() const { return mTranslateRotateManipulator; }
  virtual void updateTranslateRotateHandlesSize();
  virtual void attachTranslateRotateManipulator();
  virtual void detachTranslateRotateManipulator();

  // check if translation and rotation field is visible and don't trigger parameter node regeneration
  bool canBeTranslated() const;
  bool canBeRotated() const;
  bool isTranslationFieldVisible() const;
  bool isRotationFieldVisible() const;

  virtual void emitTranslationOrRotationChangedByUser() { assert(false); }

protected:
  void init(WbBaseNode *node);

  // all constructors are reserved for derived classes only
  explicit WbAbstractPose(WbBaseNode *node) { init(node); }

  // in WbTrackWheel fields are created instead of loading them
  WbSFVector3 *mTranslation;
  WbSFRotation *mRotation;
  WbSFDouble *mTranslationStep;
  WbSFDouble *mRotationStep;

  void setMatrixNeedUpdateFlag() const;
  void updateRotation();
  void updateTranslation();
  void updateTranslationAndRotation();
  void applyTranslationToWren();
  void applyRotationToWren();
  void applyTranslationAndRotationToWren();

  WbTranslateRotateManipulator *mTranslateRotateManipulator;
  void createTranslateRotateManipulatorIfNeeded();
  bool mTranslateRotateManipulatorInitialized;

  void inline setTranslationAndRotationFromOde(double tx, double ty, double tz, double rx, double ry, double rz, double angle);

  // WREN objects and methods
  void deleteWrenObjects();
  WbBaseNode *mBaseNode;
  mutable WbMatrix4 *mMatrix;
  mutable bool mMatrixNeedUpdate;
  mutable WbMatrix4 mVrmlMatrix;
  mutable bool mVrmlMatrixNeedUpdate;
  mutable WbQuaternion mRelativeQuaternion;

private:
  mutable bool mIsTranslationFieldVisible;
  mutable bool mIsRotationFieldVisible;
  mutable bool mIsTranslationFieldVisibleReady;
  mutable bool mIsRotationFieldVisibleReady;
  mutable bool mCanBeTranslated;
  mutable bool mCanBeRotated;
  void updateTranslationFieldVisibility() const;
  void updateRotationFieldVisibility() const;

  virtual void updateMatrix() const;
};

void inline WbAbstractPose::setTranslationAndRotationFromOde(double tx, double ty, double tz, double rx, double ry, double rz,
                                                             double angle) {
  mTranslation->setValueFromOde(tx, ty, tz);
  mRotation->setValueFromOde(rx, ry, rz, angle);
}

#endif
