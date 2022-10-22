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

#ifndef WB_HINGE_JOINT_PARAMETERS_HPP
#define WB_HINGE_JOINT_PARAMETERS_HPP

#include "WbJointParameters.hpp"
#include "WbSFVector3.hpp"

class WbHingeJointParameters : public WbJointParameters {
  Q_OBJECT

public:
  explicit WbHingeJointParameters(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbHingeJointParameters(WbTokenizer *tokenizer = NULL, bool fromDeprecatedHinge2JointParameters = false);
  WbHingeJointParameters(const WbHingeJointParameters &other);
  explicit WbHingeJointParameters(const WbNode &other, bool fromDeprecatedHinge2JointParameters = false);
  virtual ~WbHingeJointParameters();

  int nodeType() const override { return WB_NODE_HINGE_JOINT_PARAMETERS; }
  void preFinalize() override;
  void postFinalize() override;

  double suspensionSpringConstant() const { return mSuspensionSpringConstant->value(); }
  double suspensionDampingConstant() const { return mSuspensionDampingConstant->value(); }
  const WbVector3 &suspensionAxis() const { return mSuspensionAxis->value(); }

  virtual const WbVector3 &anchor() const { return mAnchor->value(); }

  double stopErp() const { return mStopErp->value(); }
  double stopCfm() const { return mStopCfm->value(); }

signals:
  void anchorChanged();
  void suspensionChanged();
  void stopErpChanged();
  void stopCfmChanged();

private:
  WbHingeJointParameters &operator=(const WbHingeJointParameters &);  // non copyable
  WbNode *clone() const override { return new WbHingeJointParameters(*this); }
  void init(bool fromDeprecatedHinge2JointParameters = false);

  // fields
  WbSFVector3 *mAnchor;
  WbSFDouble *mSuspensionSpringConstant;
  WbSFDouble *mSuspensionDampingConstant;
  WbSFVector3 *mSuspensionAxis;
  WbSFDouble *mStopErp;
  WbSFDouble *mStopCfm;

private slots:
  void updateSuspension();
  void updateAxis() override;
  void updateStopErp();
  void updateStopCfm();
};

#endif
