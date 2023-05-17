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

#ifndef WB_SPOT_LIGHT_HPP
#define WB_SPOT_LIGHT_HPP

#include "WbLight.hpp"

class WbVector3;

class WbSpotLightRepresentation;

struct WrSpotLight;

class WbSpotLight : public WbLight {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbSpotLight(WbTokenizer *tokenizer = NULL);
  WbSpotLight(const WbSpotLight &other);
  explicit WbSpotLight(const WbNode &other);
  virtual ~WbSpotLight();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SPOT_LIGHT; }
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;

  // specific functions
  const WbVector3 &direction() const;
  double cutOffAngle() const;
  double beamWidth() const;
  double exponent() const;
  double computeAttenuation(double distance) const;
  WbVector3 computeAbsoluteLocation() const;

  QStringList fieldsToSynchronizeWithX3D() const override;

protected slots:
  void updateAmbientIntensity() override;
  void updateIntensity() override;
  void updateOn() override;
  void updateColor() override;

private:
  // user accessible fields
  WbSFVector3 *mAttenuation;
  WbSFVector3 *mLocation;
  WbSFDouble *mRadius;
  WbSFVector3 *mDirection;
  WbSFDouble *mCutOffAngle;
  WbSFDouble *mBeamWidth;

  WrSpotLight *mWrenLight;

  WbSpotLightRepresentation *mLightRepresentation;

  WbSpotLight &operator=(const WbSpotLight &);  // non copyable
  WbNode *clone() const override { return new WbSpotLight(*this); }
  void init();
  void applyLightIntensityToWren() override;
  void applyLightColorToWren() override;
  void applyLightVisibilityToWren() override;
  void applyLightShadowsToWren() override;
  void applyLightDirectionToWren();
  void applyLightBeamWidthAndCutOffAngleToWren();
  void applyLightAttenuationToWren();
  void applyNodeLocationToWren();
  void applyBillboardVisibilityToWren();
  void checkAmbientAndAttenuationExclusivity();

  void attachToUpperPose();
  void detachFromUpperPose();

private slots:
  void updateDirection();
  void updateCutOffAngle();
  void updateBeamWidth();
  void updateAttenuation();
  void updateLocation();
  void updateRadius();
  void updateOptionalRendering(int option);
};

#endif
