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

#ifndef WB_LIGHT_SENSOR_HPP
#define WB_LIGHT_SENSOR_HPP

#include <QtCore/QList>
#include "WbSolidDevice.hpp"

class WbLight;
class WbLookupTable;
class WbSensor;
class LightRay;

struct WrTransform;
struct WrMaterial;
struct WrStaticMesh;
struct WrRenderable;

class WbLightSensor : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbLightSensor(WbTokenizer *tokenizer = NULL);
  WbLightSensor(const WbLightSensor &other);
  explicit WbLightSensor(const WbNode &other);
  virtual ~WbLightSensor();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_LIGHT_SENSOR; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(WbDataStream &) override;
  void writeConfigure(WbDataStream &) override;
  void createWrenObjects() override;
  void prePhysicsStep(double ms) override;
  bool refreshSensorIfNeeded() override;

  // specific functions
  void rayCollisionCallback(dGeomID geom);

  void computeLightMeasurement(const WbLight *light, const WbVector3 &sensorAxis, const WbVector3 &sensorPos,
                               WbVector3 &lightDirection, double &distance, double &direct, double &attenuation) const;

private:
  // private functions
  WbLightSensor &operator=(const WbLightSensor &);  // non copyable
  WbNode *clone() const override { return new WbLightSensor(*this); }
  void init();
  void computeValue();
  void setupRaysAndComputeDirectContributions(bool finalSetup);
  void addSingleLightContribution(const WbLight *light, double direct, double attenuation);
  void postProcessLightMeasurement();
  void applyOptionalRenderingToWren();
  void updateRaysSetupIfNeeded() override;
  void addConfigure(WbDataStream &);

  // user accessible fields
  WbMFVector3 *mLookupTable;
  WbSFColor *mColorFilter;
  WbSFBool *mOcclusion;
  WbSFDouble *mResolution;

  // WREN
  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrStaticMesh *mMesh;
  WrMaterial *mMaterial;

  // other stuff
  WbLookupTable *mLut;
  WbSensor *mSensor;
  QList<LightRay *> mRayList;
  WbRgb *mRgbIntensity;
  double mValue;  // current sensor value according to lookup table
  bool mSensorUpdateRequested;
  bool mNeedToReconfigure;

private slots:
  void updateLineScale() override;
  void updateLookupTable();
  void updateResolution();
  void applySensorRayToWren();
  void updateOptionalRendering(int option);
};

#endif
