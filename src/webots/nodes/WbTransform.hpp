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

//
// Description: extends the Pose node by implementing a scaling factor
//

#include "WbMatrix3.hpp"
#include "WbPose.hpp"

class WbBaseNode;

class WbTransform : public WbPose {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbTransform(WbTokenizer *tokenizer = NULL);
  WbTransform(const WbTransform &other);
  explicit WbTransform(const WbNode &other);

  // reimplemented functions
  int nodeType() const override { return WB_NODE_TRANSFORM; }
  void preFinalize() override;
  void postFinalize() override;

  void setScaleNeedUpdate() override;
  void createWrenObjects() override;

  QStringList fieldsToSynchronizeWithX3D() const override;

  const WbVector3 &scale() const { return mScale->value(); }
  WbSFVector3 *scaleFieldValue() const { return mScale; }

  void setScale(double x, double y, double z) { mScale->setValue(x, y, z); };
  void setScale(const WbVector3 &s) { mScale->setValue(s); }
  void setScale(int coordinate, double s) { mScale->setComponent(coordinate, s); }

  // scaling
  const WbVector3 &absoluteScale() const;

  mutable WbVector3 mAbsoluteScale;
  mutable bool mAbsoluteScaleNeedUpdate;

  // 3x3 absolute rotation matrix
  const WbMatrix4 &vrmlMatrix() const override;

protected:
  void applyToScale();
  void applyScaleToWren();

  // A specific scale check is done in the WbSolid class
  WbSFVector3 *mScale;
  double mPreviousXscaleValue;
  void sanitizeScale();

  void setScaleNeedUpdateFlag() const;

protected slots:
  void updateScale(bool warning = false);

private:
  WbTransform &operator=(const WbTransform &);  // non copyable
  WbNode *clone() const override { return new WbTransform(*this); }
  void init();

  void applyToOdeScale();

  void updateAbsoluteScale() const;
  void updateMatrix() const override;
};

#endif
