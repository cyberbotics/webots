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

#include "WbSpotLight.hpp"

#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbMFColor.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPose.hpp"
#include "WbSFBool.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbSFVector3.hpp"
#include "WbSpotLightRepresentation.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/node.h>
#include <wren/scene.h>
#include <wren/spot_light.h>
#include <wren/transform.h>

static const double gVrmlHalfLog = 0.5 * log(0.5);

void WbSpotLight::init() {
  mLightRepresentation = NULL;
  mWrenLight = NULL;

  mAttenuation = findSFVector3("attenuation");
  mLocation = findSFVector3("location");
  mRadius = findSFDouble("radius");
  mDirection = findSFVector3("direction");
  mCutOffAngle = findSFDouble("cutOffAngle");
  mBeamWidth = findSFDouble("beamWidth");
}

WbSpotLight::WbSpotLight(WbTokenizer *tokenizer) : WbLight("SpotLight", tokenizer) {
  init();
  if (tokenizer == NULL) {
    mDirection->setValueNoSignal(0, 1, -1);
    mAttenuation->setValueNoSignal(0.0, 0.0, 1.0);
    mBeamWidth->setValueNoSignal(0.7);
  }
}

WbSpotLight::WbSpotLight(const WbSpotLight &other) : WbLight(other) {
  init();
}

WbSpotLight::WbSpotLight(const WbNode &other) : WbLight(other) {
  init();
}

void WbSpotLight::preFinalize() {
  WbLight::preFinalize();

  updateAttenuation();
  updateLocation();
  updateRadius();
  updateDirection();
  updateCutOffAngle();
  updateBeamWidth();
}

void WbSpotLight::postFinalize() {
  WbLight::postFinalize();

  connect(mAttenuation, &WbSFVector3::changed, this, &WbSpotLight::updateAttenuation);
  connect(mLocation, &WbSFVector3::changed, this, &WbSpotLight::updateLocation);
  connect(mRadius, &WbSFDouble::changed, this, &WbSpotLight::updateRadius);
  connect(mDirection, &WbSFVector3::changed, this, &WbSpotLight::updateDirection);
  connect(mCutOffAngle, &WbSFDouble::changed, this, &WbSpotLight::updateCutOffAngle);
  connect(mBeamWidth, &WbSFDouble::changed, this, &WbSpotLight::updateBeamWidth);
}

WbSpotLight::~WbSpotLight() {
  if (areWrenObjectsInitialized()) {
    detachFromUpperPose();
    wr_node_delete(WR_NODE(mWrenLight));
    delete mLightRepresentation;
  }
}

WbVector3 WbSpotLight::computeAbsoluteLocation() const {
  WbVector3 location = mLocation->value();
  const WbPose *const up = upperPose();
  if (up)
    location = up->matrix() * location;
  return location;
}

void WbSpotLight::createWrenObjects() {
  mWrenLight = wr_spot_light_new();
  WbLight::createWrenObjects();
  attachToUpperPose();

  // Has to be done after WbLight::createWrenTransform (otherwise wrenNode() == NULL)
  mLightRepresentation =
    new WbSpotLightRepresentation(wrenNode(), mLocation->value(), mRadius->value(), mCutOffAngle->value(), mDirection->value());
  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::optionalRenderingChanged, this,
          &WbSpotLight::updateOptionalRendering);

  applyLightDirectionToWren();
  applyLightBeamWidthAndCutOffAngleToWren();
  applyLightAttenuationToWren();
  applyNodeLocationToWren();
  applyBillboardVisibilityToWren();
}

void WbSpotLight::updateOptionalRendering(int option) {
  if (option == WbWrenRenderingContext::VF_LIGHTS_POSITIONS)
    mLightRepresentation->setVisible(WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(option));
}

void WbSpotLight::updateAttenuation() {
  if (WbFieldChecker::resetVector3IfNegative(this, mAttenuation, WbVector3()))
    return;

  if (mAttenuation->value().x() > 0.0 || mAttenuation->value().y() > 0.0)
    parsingWarn(tr("A quadratic 'attenuation' should be preferred to have a realistic simulation of light. "
                   "Only the third component of the 'attenuation' field should be greater than 0."));

  checkAmbientAndAttenuationExclusivity();

  if (areWrenObjectsInitialized())
    applyLightAttenuationToWren();
}

void WbSpotLight::updateLocation() {
  if (areWrenObjectsInitialized())
    applyNodeLocationToWren();
  emit locationChanged();
}

void WbSpotLight::updateRadius() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mRadius, 0.0))
    return;

  if (areWrenObjectsInitialized())
    applyLightAttenuationToWren();
}

void WbSpotLight::updateAmbientIntensity() {
  checkAmbientAndAttenuationExclusivity();

  WbLight::updateAmbientIntensity();
}

void WbSpotLight::updateIntensity() {
  WbLight::updateIntensity();
}

void WbSpotLight::updateOn() {
  WbLight::updateOn();
  if (areWrenObjectsInitialized())
    applyBillboardVisibilityToWren();
}

void WbSpotLight::updateColor() {
  WbLight::updateColor();
}

void WbSpotLight::updateDirection() {
  if (areWrenObjectsInitialized())
    applyLightDirectionToWren();
}

void WbSpotLight::updateCutOffAngle() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mCutOffAngle, 0.0, M_PI_2, M_PI_2))
    return;

  if (mCutOffAngle->value() < mBeamWidth->value())
    mBeamWidth->setValueNoSignal(mCutOffAngle->value());

  if (areWrenObjectsInitialized())
    applyLightBeamWidthAndCutOffAngleToWren();
}

void WbSpotLight::updateBeamWidth() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mBeamWidth, 0.0))
    return;
  else if (mBeamWidth->value() > mCutOffAngle->value()) {
    parsingWarn(tr("Invalid 'beamWidth' changed to %1. The value should be less than or equal to 'cutOffAngle'.")
                  .arg(mCutOffAngle->value()));
    mBeamWidth->setValue(mCutOffAngle->value());
    return;
  }

  if (areWrenObjectsInitialized())
    applyLightBeamWidthAndCutOffAngleToWren();
}

void WbSpotLight::checkAmbientAndAttenuationExclusivity() {
  if (mAttenuation->value() != WbVector3(1.0, 0.0, 0.0) && ambientIntensity() != 0.0) {
    parsingWarn(
      tr("'ambientIntensity' and 'attenuation' cannot differ from their default values at the same time. 'ambientIntensity' "
         "was changed to 0."));
    setAmbientIntensity(0.0);
  }
}

void WbSpotLight::attachToUpperPose() {
  const WbPose *const upperPose = WbNodeUtilities::findUpperPose(this);
  if (upperPose)
    wr_transform_attach_child(upperPose->wrenNode(), WR_NODE(mWrenLight));
}

void WbSpotLight::detachFromUpperPose() {
  WrNode *node = WR_NODE(mWrenLight);
  WrTransform *parent = wr_node_get_parent(node);
  if (parent)
    wr_transform_detach_child(parent, node);
}

void WbSpotLight::applyLightIntensityToWren() {
  wr_spot_light_set_intensity(mWrenLight, mIntensity->value());
}

void WbSpotLight::applyLightColorToWren() {
  const float color[] = {static_cast<float>(mColor->red()), static_cast<float>(mColor->green()),
                         static_cast<float>(mColor->blue())};
  wr_spot_light_set_color(mWrenLight, color);
}

void WbSpotLight::applyLightVisibilityToWren() {
  wr_spot_light_set_on(mWrenLight, mOn->value());

  const int maxCount = wr_config_get_max_active_spot_light_count();
  const int activeCount = wr_scene_get_active_spot_light_count(wr_scene_get_instance());
  if (activeCount == maxCount)
    parsingWarn(
      tr("Maximum number of active spotlights (%1) has been reached, newly added lights won't be rendered.").arg(maxCount));
}

void WbSpotLight::applyLightShadowsToWren() {
  wr_spot_light_set_cast_shadows(mWrenLight, mCastShadows->value());
}

void WbSpotLight::applyLightAttenuationToWren() {
  wr_spot_light_set_radius(mWrenLight, mRadius->value());
  wr_spot_light_set_attenuation(mWrenLight, mAttenuation->x(), mAttenuation->y(), mAttenuation->z());
  mLightRepresentation->setRadius(mRadius->value());
}

void WbSpotLight::applyNodeLocationToWren() {
  const float position[] = {static_cast<float>(mLocation->x()), static_cast<float>(mLocation->y()),
                            static_cast<float>(mLocation->z())};
  wr_spot_light_set_position_relative(mWrenLight, position);
  mLightRepresentation->setPosition(mLocation->value());
}

void WbSpotLight::applyBillboardVisibilityToWren() {
  const bool enabled =
    WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_LIGHTS_POSITIONS);
  mLightRepresentation->setVisible(isOn() && enabled);
}

void WbSpotLight::applyLightDirectionToWren() {
  const float d[] = {static_cast<float>(mDirection->x()), static_cast<float>(mDirection->y()),
                     static_cast<float>(mDirection->z())};
  wr_spot_light_set_direction(mWrenLight, d);
  mLightRepresentation->setDirection(mDirection->value());
}

void WbSpotLight::applyLightBeamWidthAndCutOffAngleToWren() {
  wr_spot_light_set_beam_width(mWrenLight, mBeamWidth->value());
  wr_spot_light_set_cutoff_angle(mWrenLight, mCutOffAngle->value());
  mLightRepresentation->setCutOffAngle(mCutOffAngle->value());
}

double WbSpotLight::exponent() const {
  return gVrmlHalfLog / log(cos(mBeamWidth->value()));
}

const WbVector3 &WbSpotLight::direction() const {
  return mDirection->value();
}

double WbSpotLight::cutOffAngle() const {
  return mCutOffAngle->value();
}

double WbSpotLight::beamWidth() const {
  return mBeamWidth->value();
}

double WbSpotLight::computeAttenuation(double distance) const {
  return 1.0 / (mAttenuation->x() + mAttenuation->y() * distance + mAttenuation->z() * distance * distance);
}

QStringList WbSpotLight::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "attenuation"
         << "beamWidth"
         << "cutOffAngle"
         << "direction"
         << "location"
         << "radius" << WbLight::fieldsToSynchronizeWithX3D();
  return fields;
}
