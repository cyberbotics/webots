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

#ifndef WB_JOINT_PARAMETERS_HPP
#define WB_JOINT_PARAMETERS_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"
#include "WbSFVector3.hpp"

class WbJointParameters : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbJointParameters(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbJointParameters(WbTokenizer *tokenizer = NULL);
  WbJointParameters(const WbJointParameters &other);
  explicit WbJointParameters(const WbNode &other);
  virtual ~WbJointParameters();

  int nodeType() const override { return WB_NODE_JOINT_PARAMETERS; }
  void preFinalize() override;
  void postFinalize() override;

  double position() const { return mPosition->value(); }
  double maxStop() const { return mMaxStop->value(); }
  double minStop() const { return mMinStop->value(); }
  double springConstant() const { return mSpringConstant->value(); }
  double dampingConstant() const { return mDampingConstant->value(); }
  double staticFriction() const { return mStaticFriction->value(); }
  const WbVector3 axis() const { return mAxis ? mAxis->value() : WbVector3(); }

  void setPosition(double p) { mPosition->setValue(p); }
  void setPositionFromOde(double p) { mPosition->setValueFromOde(p); }

  bool clampPosition(double &p) const;

signals:
  void positionChanged();
  void minAndMaxStopChanged(double min, double max);
  void springAndDampingConstantsChanged();
  void axisChanged();

protected:
  WbSFVector3 *mAxis;  // axis default value redefined in a derived classes
  bool exportNodeHeader(WbWriter &writer) const override;

private:
  WbJointParameters &operator=(const WbJointParameters &);  // non copyable
  WbNode *clone() const override { return new WbJointParameters(*this); }
  void init();

  // fields
  WbSFDouble *mPosition;
  WbSFDouble *mMinStop;
  WbSFDouble *mMaxStop;
  WbSFDouble *mSpringConstant;
  WbSFDouble *mDampingConstant;
  WbSFDouble *mStaticFriction;

private slots:
  void updateMinAndMaxStop();
  void updateSpringConstant();
  void updateDampingConstant();
  void updateStaticFriction();
  virtual void updateAxis();
};

#endif
