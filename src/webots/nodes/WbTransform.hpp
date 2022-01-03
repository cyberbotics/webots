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

#ifndef WB_TRANSFORM_HPP
#define WB_TRANSFORM_HPP

//
// Description: a node that defines a 3D coordinate system transformation
//

#include "WbPose.hpp"

class WbTransform : public WbPose {
  Q_OBJECT

public:
  // constructors and destructor
  WbTransform(WbTokenizer *tokenizer = NULL);
  WbTransform(const WbTransform &other);
  WbTransform(const WbNode &other);
  // reimplemented functions
  int nodeType() const override { return WB_NODE_TRANSFORM; }

  const WbVector3 &scale() const { return mScale->value(); }
  WbSFVector3 *scaleFieldValue() const { return mScale; }
  WbScaleManipulator *scaleManipulator() { return mScaleManipulator; }

  void setScale(double x, double y, double z) { mScale->setValue(x, y, z); };
  void setScale(const WbVector3 &s) { mScale->setValue(s); }
  void setScale(int coordinate, double s) { mScale->setComponent(coordinate, s); }

  bool absoluteScaleNeedUpdate() const { return mAbsoluteScaleNeedUpdate; }
  const WbVector3 &absoluteScale() const;

  // resize/scale manipulator
  bool hasResizeManipulator() const;
  void attachResizeManipulator();
  void detachResizeManipulator() const;
  void updateResizeHandlesSize();
  void setResizeManipulatorDimensions();

protected:
  WbSFVector3 *mScale;
  mutable WbVector3 mAbsoluteScale;
  bool checkScale(int constraintType = 0, bool warning = false);
  bool checkScalePositivity(WbVector3 &correctedScale) const;
  bool checkScaleUniformity(WbVector3 &correctedScale, bool warning = false) const;
  bool checkScaleUniformity(bool warning = false);
  bool checkScalingPhysicsConstraints(WbVector3 &correctedScale, int constraintType, bool warning = false) const;
  void applyToScale();

  void updateScale(bool warning = false);
  void applyScaleToWren();

  // WREN manipulators
  WbScaleManipulator *mScaleManipulator;
  void createScaleManipulator();
  void createScaleManipulatorIfNeeded();
  bool mScaleManipulatorInitialized;

  void showResizeManipulator(bool enabled);

private:
  WbTransform &operator=(const WbTransform &);  // non copyable
  WbNode *clone() const override { return new WbTransform(*this); }
  void init();

  mutable bool mAbsoluteScaleNeedUpdate;
  void updateAbsoluteScale() const;
};

#endif
