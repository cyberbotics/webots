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

#ifndef WB_ABSTRACT_TRANSFORM_HPP
#define WB_ABSTRACT_TRANSFORM_HPP

//
// Description: abstract node implementing 'translation', 'rotation', and
//              'scale' fields functionalities and keeping the WREN::SceneNode
//              up-to-date
//
// Inherited by: WbTransform and WbSkin
//

#include <cassert>
#include "WbMatrix3.hpp"
#include "WbMatrix4.hpp"
#include "WbSFDouble.hpp"
#include "WbSFRotation.hpp"
#include "WbSFVector3.hpp"

class WbBaseNode;
class WbScaleManipulator;
class WbTranslateRotateManipulator;

class WbAbstractTransform {
public:
  virtual ~WbAbstractTransform();

  virtual WbBaseNode *baseNode() const { return mBaseNode; }

  // field accessors
  const WbVector3 &translation() const { return mTranslation->value(); }
  const WbRotation &rotation() const { return mRotation->value(); }
  WbSFVector3 *translationFieldValue() const { return mTranslation; }
  WbSFRotation *rotationFieldValue() const { return mRotation; }
  const WbVector3 &scale() const { return mScale->value(); }
  WbSFVector3 *scaleFieldValue() const { return mScale; }
  bool absoluteScaleNeedUpdate() const { return mAbsoluteScaleNeedUpdate; }

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
  void setScale(double x, double y, double z) { mScale->setValue(x, y, z); };
  void setScale(const WbVector3 &s) { mScale->setValue(s); }
  void setScale(int coordinate, double s) { mScale->setComponent(coordinate, s); }

  // 4x4 transform matrices
  const WbMatrix4 &matrix() const;
  const WbMatrix4 &vrmlMatrix() const;
  WbVector3 xAxis() const { return matrix().xAxis(); }
  WbVector3 yAxis() const { return matrix().yAxis(); }
  WbVector3 zAxis() const { return matrix().zAxis(); }

  // scaling
  virtual const WbVector3 &absoluteScale() const;
  bool isTopTransform() const;
  virtual int constraintType() const = 0;

  // 3x3 absolute rotation matrix
  WbMatrix3 rotationMatrix() const {
    const WbVector3 &s = absoluteScale();
    WbMatrix3 m = matrix().extracted3x3Matrix();
    m.scale(1.0 / s.x(), 1.0 / s.y(), 1.0 / s.z());
    return m;
  }

  // position in 'world' coordinates
  WbVector3 position() const { return matrix().translation(); }

  // resize/scale manipulator
  WbScaleManipulator *scaleManipulator() { return mScaleManipulator; }
  bool isScaleManipulatorAttached() const;
  void updateResizeHandlesSize();
  void setResizeManipulatorDimensions();
  void setUniformConstraintForResizeHandles(bool enabled);

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

  void attachResizeManipulator();
  void detachResizeManipulator() const;
  bool hasResizeManipulator() const;

  virtual void emitTranslationOrRotationChangedByUser() { assert(false); }

protected:
  void init(WbBaseNode *node);

  // all constructors are reserved for derived classes only
  WbAbstractTransform(WbBaseNode *node) { init(node); }

  // in WbTrackWheel fields are created instead of loading them
  WbSFVector3 *mTranslation;
  WbSFRotation *mRotation;
  WbSFDouble *mTranslationStep;
  WbSFDouble *mRotationStep;

  void setScaleNeedUpdateFlag() const;
  void setMatrixNeedUpdateFlag() const;
  void updateRotation();
  void updateTranslation();
  void updateScale(bool warning = false);
  void updateTranslationAndRotation();
  void updateConstrainedHandleMaterials();
  void applyTranslationToWren();
  void applyRotationToWren();
  void applyScaleToWren();
  void applyTranslationAndRotationToWren();

  // A specific scale check is done in the WbSolid class
  WbSFVector3 *mScale;
  bool checkScale(int constraintType = 0, bool warning = false);
  bool checkScaleZeroValues(WbVector3 &correctedScale) const;
  bool checkScaleUniformity(WbVector3 &correctedScale, bool warning = false) const;
  bool checkScaleUniformity(bool warning = false);
  virtual bool checkScalingPhysicsConstraints(WbVector3 &correctedScale, int constraintType, bool warning = false) const;
  virtual void applyToScale();
  double mPreviousXscaleValue;
  mutable WbVector3 mAbsoluteScale;

  // WREN manipulators
  WbScaleManipulator *mScaleManipulator;
  virtual void createScaleManipulator();
  void createScaleManipulatorIfNeeded();
  bool mScaleManipulatorInitialized;

  WbTranslateRotateManipulator *mTranslateRotateManipulator;
  void createTranslateRotateManipulatorIfNeeded();
  bool mTranslateRotateManipulatorInitialized;

  void showResizeManipulator(bool enabled);

  void inline setTranslationAndRotationFromOde(double tx, double ty, double tz, double rx, double ry, double rz, double angle);

private:
  WbBaseNode *mBaseNode;
  mutable bool mIsTranslationFieldVisible;
  mutable bool mIsRotationFieldVisible;
  mutable bool mIsTranslationFieldVisibleReady;
  mutable bool mIsRotationFieldVisibleReady;
  mutable bool mCanBeTranslated;
  mutable bool mCanBeRotated;
  void updateTranslationFieldVisibility() const;
  void updateRotationFieldVisibility() const;

  void updateMatrix() const;
  void updateAbsoluteScale() const;
  mutable WbMatrix4 *mMatrix;
  mutable WbMatrix4 mVrmlMatrix;
  mutable bool mMatrixNeedUpdate;
  mutable bool mVrmlMatrixNeedUpdate;
  mutable bool mAbsoluteScaleNeedUpdate;
  mutable bool mHasSearchedTopTransform;
  mutable bool mIsTopTransform;

  // WREN objects and methods
  void deleteWrenObjects();
};

void inline WbAbstractTransform::setTranslationAndRotationFromOde(double tx, double ty, double tz, double rx, double ry,
                                                                  double rz, double angle) {
  mTranslation->setValueFromOde(tx, ty, tz);
  mRotation->setValueFromOde(rx, ry, rz, angle);
}

#endif
