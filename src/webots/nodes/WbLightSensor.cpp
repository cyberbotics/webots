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

#include "WbLightSensor.hpp"

#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbLight.hpp"
#include "WbLookupTable.hpp"
#include "WbMFVector3.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMatrix3.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbSFBool.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbSensor.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include "../../controller/c/messages.h"

#include <wren/config.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <limits>

#include <QtCore/QDataStream>

// TODO: remove these dependencies :
// implementing generic functionalities on WbLight would be more modular
#include "WbDirectionalLight.hpp"
#include "WbPointLight.hpp"
#include "WbSpotLight.hpp"

class LightRay {
public:
  LightRay(WbLightSensor *sensor, const WbVector3 &lightDirection, double distance, const WbLight *light, double direct,
           double attenuation, dSpaceID spaceId) :
    mLight(light),
    mSensor(sensor),
    mDirect(direct),
    mAttenuation(attenuation),
    mCollided(false) {
    assert(spaceId && sensor && mLight);

    // setup ray geom for ODE collision detection
    mGeom = dCreateRay(spaceId, distance);
    dGeomSetDynamicFlag(mGeom);
    const WbVector3 &t = sensor->matrix().translation();
    dGeomRaySet(mGeom, t.x(), t.y(), t.z(), lightDirection.x(), lightDirection.y(), lightDirection.z());

    // set receiver as callback data in case there is a collision
    dGeomSetData(mGeom, new WbOdeGeomData(sensor));
  };

  ~LightRay() {
    if (mGeom) {
      WbOdeGeomData *odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(mGeom));
      delete odeGeomData;
      dGeomDestroy(mGeom);
    }
  }

  bool hasCollided() const { return mCollided; }
  void setCollided() { mCollided = true; }
  const WbLight *light() const { return mLight; }
  double directIntensity() const { return mDirect; }
  double attenuation() const { return mAttenuation; }
  dGeomID geom() const { return mGeom; }

  bool recomputeRayDirection() {
    const WbVector3 &sensorPos = mSensor->matrix().translation();
    const WbVector3 &sensorDirection = mSensor->matrix().xAxis();

    WbVector3 L;
    double distance = 0.0;
    mSensor->computeLightMeasurement(mLight, sensorDirection, sensorPos, L, distance, mDirect, mAttenuation);
    if (mDirect <= 0.0)
      return false;

    // recompute ray length and direction
    dGeomRaySetLength(mGeom, distance);
    dGeomRaySet(mGeom, sensorPos[0], sensorPos[1], sensorPos[2], L[0], L[1], L[2]);
    return true;
  }

private:
  const WbLight *mLight;
  const WbLightSensor *mSensor;
  double mDirect;
  double mAttenuation;
  bool mCollided;  // the geom has collided yet
  dGeomID mGeom;   // geom that checks collision of this packet
};

void WbLightSensor::init() {
  mValue = 0.0;
  mLut = NULL;
  mSensor = NULL;
  mRgbIntensity = new WbRgb();
  mSensorUpdateRequested = false;

  mLookupTable = findMFVector3("lookupTable");
  mColorFilter = findSFColor("colorFilter");
  mOcclusion = findSFBool("occlusion");
  mResolution = findSFDouble("resolution");

  mTransform = NULL;
  mMaterial = NULL;
  mRenderable = NULL;
  mMesh = NULL;

  mNeedToReconfigure = false;
}

WbLightSensor::WbLightSensor(WbTokenizer *tokenizer) : WbSolidDevice("LightSensor", tokenizer) {
  init();
}

WbLightSensor::WbLightSensor(const WbLightSensor &other) : WbSolidDevice(other) {
  init();
}

WbLightSensor::WbLightSensor(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbLightSensor::~WbLightSensor() {
  delete mLut;
  delete mSensor;
  delete mRgbIntensity;

  qDeleteAll(mRayList);

  if (areWrenObjectsInitialized()) {
    wr_node_delete(WR_NODE(mRenderable));
    wr_node_delete(WR_NODE(mTransform));
    wr_material_delete(mMaterial);
    wr_static_mesh_delete(mMesh);
  }
}

void WbLightSensor::preFinalize() {
  WbSolidDevice::preFinalize();
  mSensor = new WbSensor();
  updateLookupTable();
}

void WbLightSensor::postFinalize() {
  WbSolidDevice::postFinalize();
  connect(mLookupTable, &WbMFVector3::changed, this, &WbLightSensor::updateLookupTable);
  connect(mResolution, &WbSFDouble::changed, this, &WbLightSensor::updateResolution);
  connect(mSensor, &WbSensor::stateChanged, this, &WbLightSensor::applySensorRayToWren);
}

void WbLightSensor::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      break;
    default:
      assert(0);
  }
}

void WbLightSensor::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_LIGHT_SENSOR_DATA;
    stream << mValue;

    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void WbLightSensor::addConfigure(WbDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mLookupTable->size();
  for (int i = 0; i < mLookupTable->size(); i++) {
    stream << (double)mLookupTable->item(i).x();
    stream << (double)mLookupTable->item(i).y();
    stream << (double)mLookupTable->item(i).z();
  }
  mNeedToReconfigure = false;
}

void WbLightSensor::writeConfigure(WbDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

void WbLightSensor::updateLookupTable() {
  // rebuild the lookup table
  delete mLut;
  mLut = new WbLookupTable(*mLookupTable);
  mValue = mLut->minValue();

  mNeedToReconfigure = true;
}

void WbLightSensor::updateResolution() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void WbLightSensor::prePhysicsStep(double ms) {
  WbSolidDevice::prePhysicsStep(ms);

  mSensorUpdateRequested = isPowerOn() && mSensor->needToRefreshInMs(ms);
  if (mSensorUpdateRequested) {
    // in case postProcessLightMeasurement() was not called since the
    // previous prePhysicsStep() we need to clear the list at this point
    qDeleteAll(mRayList);
    mRayList.clear();

    // reset before accumulation
    mRgbIntensity->setValue(0.0, 0.0, 0.0);

    if (mOcclusion->isTrue()) {
      // add ambient light contribution and create rays for collision detection
      setupRaysAndComputeDirectContributions(false);

      if (!mRayList.isEmpty())
        // sensor or target objects could move during physics step
        subscribeToRaysUpdate(mRayList[0]->geom());
    }
  }
}

void WbLightSensor::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();
  foreach (LightRay *ray, mRayList)
    ray->recomputeRayDirection();
}

void WbLightSensor::setupRaysAndComputeDirectContributions(bool finalSetup) {
  const WbVector3 &sensorAxis = matrix().xAxis();       // sensor normal vector (x-axis)
  const WbVector3 &sensorPos = matrix().translation();  // sensor position (world coordinate)

  QList<const WbLight *> lights = WbLight::lights();
  foreach (const WbLight *light, lights) {
    if (light->isOn()) {
      WbVector3 lightDirection;
      double distance, attenuation, direct;
      computeLightMeasurement(light, sensorAxis, sensorPos, lightDirection, distance, direct, attenuation);
      if (areOdeObjectsCreated() && mOcclusion->isTrue()) {
        if (finalSetup && direct <= 0.0)
          continue;

        // we need collision detection if the light source is in front of the sensor plate
        // and if occlusion check is required by the user
        mRayList.append(
          new LightRay(this, lightDirection, distance, light, direct, attenuation, WbOdeContext::instance()->space()));
      } else
        // otherwise, this light's contribution can be added immediately
        addSingleLightContribution(light, direct, attenuation);
    }
  }
}

void WbLightSensor::computeLightMeasurement(const WbLight *light,
                                            const WbVector3 &sensorAxis,  // sensor normal vector (x-axis)
                                            const WbVector3 &sensorPos,   // (world coordinate)
                                            WbVector3 &lightDirection, double &distance, double &direct,
                                            double &attenuation) const {
  double spotFactor = 1.0;

  const WbPointLight *pointLight = dynamic_cast<const WbPointLight *>(light);
  const WbDirectionalLight *directionalLight = dynamic_cast<const WbDirectionalLight *>(light);
  const WbSpotLight *spotLight = dynamic_cast<const WbSpotLight *>(light);

  if (pointLight) {
    const WbVector3 &lightPos = pointLight->computeAbsoluteLocation();
    lightDirection = lightPos - sensorPos;
    distance = lightDirection.length();  // sensor->light distance
    lightDirection /= distance;          // normalize lightDirection
    attenuation = pointLight->computeAttenuation(distance);
  } else if (directionalLight) {
    lightDirection = -directionalLight->direction();
    distance = 1000.0;  // assume DirectionalLight is 1 kilometer away
    lightDirection.normalize();
    attenuation = 1.0;  // no attenuation for DirectionalLight
  } else if (spotLight) {
    const WbVector3 &lightPos = spotLight->computeAbsoluteLocation();

    lightDirection = lightPos - sensorPos;
    distance = lightDirection.length();  // sensor->light distance
    lightDirection /= distance;          // normalize lightDirection
    attenuation = spotLight->computeAttenuation(distance);

    // compute spot's beam effect
    const WbVector3 &dir = spotLight->direction();
    const WbPose *up = spotLight->upperPose();
    WbVector3 R = up ? -(up->rotation().toMatrix3() * dir) : -dir;
    R.normalize();
    double alpha = WbMathsUtilities::clampedAcos(lightDirection.dot(R));  // both lightDirection and R are normalized
    assert(!std::isnan(alpha));
    if (alpha > spotLight->cutOffAngle())
      spotFactor = 0.0;
    else if (alpha <= spotLight->beamWidth())
      spotFactor = 1.0;
    else
      spotFactor = (alpha - spotLight->cutOffAngle()) / (spotLight->beamWidth() - spotLight->cutOffAngle());
  } else {
    assert(0);
    distance = 0.0;
    direct = 0.0;
    attenuation = 0.0;
    return;  // make compiler happy in release mode
  }

  // compute direct light intensity according to rays incidence angle
  // dot(sensorAxis, lightDirection) == cos(phi) because both sensorAxis and
  // lightDirection have been normalized above
  direct = 0.0;
  double cosine = sensorAxis.dot(lightDirection);
  if (cosine > 0.0)
    // light is in front of sensor
    direct = light->intensity() * cosine * spotFactor;
}

bool WbLightSensor::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void WbLightSensor::computeValue() {
  if (mOcclusion->isTrue())
    // add direct light contribution after collision detection
    postProcessLightMeasurement();
  else
    // add direct and ambient light contribution
    setupRaysAndComputeDirectContributions(true);

  // apply color filter
  const WbRgb &rgbFilter = mColorFilter->value();
  double finalIntensity = rgbFilter.red() * mRgbIntensity->red() + rgbFilter.green() * mRgbIntensity->green() +
                          rgbFilter.blue() * mRgbIntensity->blue();
  finalIntensity /= 3.0;

  // apply lookup table
  mValue = mLut->lookup(finalIntensity);

  // apply resolution if needed
  if (mResolution->value() != -1.0)
    mValue = WbMathsUtilities::discretize(mValue, mResolution->value());
}

void WbLightSensor::addSingleLightContribution(const WbLight *light, double direct, double attenuation) {
  const WbRgb &color = light->color();
  double attenuatedLight = attenuation * (light->ambientIntensity() + direct);

  mRgbIntensity->setRed(mRgbIntensity->red() + attenuatedLight * color.red());
  mRgbIntensity->setGreen(mRgbIntensity->green() + attenuatedLight * color.green());
  mRgbIntensity->setBlue(mRgbIntensity->blue() + attenuatedLight * color.blue());
}

void WbLightSensor::postProcessLightMeasurement() {
  assert(mOcclusion->isTrue());

  foreach (LightRay *ray, mRayList) {
    // in case of occlusion the direct intensity is zero
    double direct = ray->hasCollided() ? 0.0 : ray->directIntensity();

    // add this light contribution of measured intensity
    addSingleLightContribution(ray->light(), direct, ray->attenuation());
    delete ray;
  }

  // clear list
  mRayList.clear();
}

void WbLightSensor::rayCollisionCallback(dGeomID geom) {
  foreach (LightRay *ray, mRayList) {
    if (ray->geom() == geom) {
      ray->setCollided();
      return;
    }
  }

  assert(0);  // should never be reached
}

void WbLightSensor::createWrenObjects() {
  mTransform = wr_transform_new();
  mMaterial = wr_phong_material_new();
  wr_material_set_default_program(mMaterial, WbWrenShaders::lineSetShader());

  const float vertices[6] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  mMesh = wr_static_mesh_line_set_new(2, vertices, NULL);
  mRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mRenderable, false);
  wr_renderable_set_receive_shadows(mRenderable, false);
  wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VF_LIGHT_SENSORS_RAYS);
  wr_renderable_set_drawing_mode(mRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
  wr_renderable_set_material(mRenderable, mMaterial, NULL);

  WbSolidDevice::createWrenObjects();

  wr_transform_attach_child(mTransform, WR_NODE(mRenderable));
  wr_transform_attach_child(wrenNode(), WR_NODE(mTransform));

  if (!WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_LIGHT_SENSORS_RAYS))
    wr_node_set_visible(WR_NODE(mTransform), false);

  applySensorRayToWren();
  applyOptionalRenderingToWren();

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::optionalRenderingChanged, this,
          &WbLightSensor::updateOptionalRendering);
  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this, &WbLightSensor::updateLineScale);
}

void WbLightSensor::updateOptionalRendering(int option) {
  if (option == WbWrenRenderingContext::VF_LIGHT_SENSORS_RAYS) {
    if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(option))
      wr_node_set_visible(WR_NODE(mTransform), true);
    else
      wr_node_set_visible(WR_NODE(mTransform), false);
  }
}

void WbLightSensor::updateLineScale() {
  if (areWrenObjectsInitialized())
    applyOptionalRenderingToWren();
}

void WbLightSensor::applyOptionalRenderingToWren() {
  if (!areWrenObjectsInitialized())
    return;

  const float lineScale = wr_config_get_line_scale();
  const float scale[3] = {lineScale, lineScale, lineScale};
  wr_transform_set_scale(mTransform, scale);
}

void WbLightSensor::applySensorRayToWren() {
  const float greyColor[3] = {0.5f, 0.5f, 0.5f};
  const float yellowColor[3] = {1.0f, 1.0f, 0.0f};

  if (mSensor->isEnabled() && mSensor->isFirstValueReady())
    wr_phong_material_set_emissive(mMaterial, yellowColor);
  else
    wr_phong_material_set_emissive(mMaterial, greyColor);
}
