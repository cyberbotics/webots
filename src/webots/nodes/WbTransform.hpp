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

#ifndef WB_TRANSFORM_HPP
#define WB_TRANSFORM_HPP

#include "WbMatrix3.hpp"
#include "WbPose.hpp"

class WbBaseNode;
class WbScaleManipulator;

class WbTransform : public WbPose {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbTransform(WbTokenizer *tokenizer = NULL);
  WbTransform(const WbTransform &other);
  explicit WbTransform(const WbNode &other);
  virtual ~WbTransform();

  // reimplemented functions
  int nodeType() const override { return WB_NODE_TRANSFORM; }
  void preFinalize() override;
  void postFinalize() override;

  void setScaleNeedUpdate();
  void createWrenObjects() override;

  QStringList fieldsToSynchronizeWithX3D() const override;

  const WbVector3 &scale() const { return mScale->value(); }
  WbSFVector3 *scaleFieldValue() const { return mScale; }
  bool absoluteScaleNeedUpdate() const { return mAbsoluteScaleNeedUpdate; }

  void setScale(double x, double y, double z) { mScale->setValue(x, y, z); };
  void setScale(const WbVector3 &s) { mScale->setValue(s); }
  void setScale(int coordinate, double s) { mScale->setComponent(coordinate, s); }

  // scaling
  virtual const WbVector3 &absoluteScale() const;
  virtual int constraintType() const;

  // 3x3 absolute rotation matrix
  WbMatrix3 rotationMatrix() const override;

  // resize/scale manipulator
  WbScaleManipulator *scaleManipulator() { return mScaleManipulator; }
  bool isScaleManipulatorAttached() const;
  void updateResizeHandlesSize();
  void setResizeManipulatorDimensions();
  void setUniformConstraintForResizeHandles(bool enabled);
  void attachResizeManipulator();
  void detachResizeManipulator() const;
  bool hasResizeManipulator() const;
  void showResizeManipulator(bool enabled);

protected:
  bool checkScale(int constraintType = 0, bool warning = false);
  void applyToScale();
  void applyScaleToWren();

  // A specific scale check is done in the WbSolid class
  WbSFVector3 *mScale;
  bool checkScaleZeroValues(WbVector3 &correctedScale) const;
  bool checkScaleUniformity(WbVector3 &correctedScale, bool warning = false) const;
  bool checkScaleUniformity(bool warning = false);
  virtual bool checkScalingPhysicsConstraints(WbVector3 &correctedScale, int constraintType, bool warning = false) const;
  double mPreviousXscaleValue;
  mutable WbVector3 mAbsoluteScale;

  void setScaleNeedUpdateFlag() const;

  // WREN manipulators
  WbScaleManipulator *mScaleManipulator;
  virtual void createScaleManipulator();
  void createScaleManipulatorIfNeeded();
  bool mScaleManipulatorInitialized;

protected slots:
  void updateConstrainedHandleMaterials();
  void updateScale(bool warning = false);

private:
  WbTransform &operator=(const WbTransform &);  // non copyable
  WbNode *clone() const override { return new WbTransform(*this); }
  void init();

  void applyToOdeScale();

  void updateAbsoluteScale() const;

  mutable bool mAbsoluteScaleNeedUpdate;

  // WREN objects and methods
  void deleteWrenObjects() override;
};

#endif
