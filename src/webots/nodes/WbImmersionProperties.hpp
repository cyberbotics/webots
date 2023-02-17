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

#ifndef WB_IMMERSION_PROPERTIES_HPP
#define WB_IMMERSION_PROPERTIES_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"
#include "WbSFString.hpp"
#include "WbSFVector3.hpp"

class WbImmersionProperties : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbImmersionProperties(WbTokenizer *tokenizer = NULL);
  WbImmersionProperties(const WbImmersionProperties &other);
  explicit WbImmersionProperties(const WbNode &other);
  virtual ~WbImmersionProperties();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_IMMERSION_PROPERTIES; }
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  const WbVector3 &dragForceCoefficients() const { return mDragForceCoefficients->value(); }
  const WbVector3 &dragTorqueCoefficients() const { return mDragTorqueCoefficients->value(); }
  double viscousResistanceForceCoefficient() const { return mViscousResistanceForceCoefficient->value(); }
  double viscousResistanceTorqueCoefficient() const { return mViscousResistanceTorqueCoefficient->value(); }
  const QString &fluidName() const { return mFluidName->value(); }
  const QString &referenceArea() const { return mReferenceArea->value(); }
  int immersionSurfaceMode() const { return mImmersionSurfaceMode; }

private:
  // user accessible fields
  WbSFVector3 *mDragForceCoefficients;
  WbSFVector3 *mDragTorqueCoefficients;
  WbSFDouble *mViscousResistanceForceCoefficient, *mViscousResistanceTorqueCoefficient;
  WbSFString *mFluidName;
  WbSFString *mReferenceArea;

  int mImmersionSurfaceMode;  // reference area code passed to ODE

  WbImmersionProperties &operator=(const WbImmersionProperties &);  // non copyable
  WbNode *clone() const override { return new WbImmersionProperties(*this); }
  void init();

private slots:
  void updateDragForceCoefficients();
  void updateDragTorqueCoefficients();
  void updateLinearViscousResistanceCoefficient();
  void updateAngularViscousResistanceCoefficient();
  void updateReferenceArea();
};

#endif
