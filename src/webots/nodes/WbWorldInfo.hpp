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

#ifndef WB_WORLD_INFO_HPP
#define WB_WORLD_INFO_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSFString.hpp"
#include "WbSFVector3.hpp"
#include "WbVector3.hpp"

class WbReceiver;
class WbDamping;
class WbContactProperties;
class WbVersion;

class WbWorldInfo : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbWorldInfo(WbTokenizer *tokenizer = NULL);
  WbWorldInfo(const WbWorldInfo &other);
  explicit WbWorldInfo(const WbNode &other);
  virtual ~WbWorldInfo();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_WORLD_INFO; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;

  // field accessors
  const WbMFString &info() const { return *mInfo; }
  const QString &title() const { return mTitle->value(); }
  const QString &window() const { return mWindow->value(); }
  double gravity() const { return mGravity->value(); }
  double cfm() const { return mCfm->value(); }
  double erp() const { return mErp->value(); }
  const QString &physics() const { return mPhysics->value(); }
  double basicTimeStep() const { return mBasicTimeStep->value(); }
  double fps() const { return mFps->value(); }
  int optimalThreadCount() const { return mOptimalThreadCount->value(); }
  double physicsDisableTime() const { return mPhysicsDisableTime->value(); }
  double physicsDisableLinearThreshold() const { return mPhysicsDisableLinearThreshold->value(); }
  double physicsDisableAngularThreshold() const { return mPhysicsDisableAngularThreshold->value(); }
  WbDamping *defaultDamping() const;
  double lineScale() const;
  double dragForceScale() const { return mDragForceScale->value(); };
  double dragTorqueScale() const { return mDragTorqueScale->value(); };
  const QString &coordinateSystem() const { return mCoordinateSystem->value(); }
  const QString &gpsCoordinateSystem() const { return mGpsCoordinateSystem->value(); }
  const WbVector3 &gpsReference() const { return mGpsReference->value(); }
  int randomSeed() const { return mRandomSeed->value(); }
  int contactPropertiesCount() const;
  const WbMFNode &contactProperties() const { return *mContactProperties; }
  WbContactProperties *contactProperties(int index) const;
  double inkEvaporation() const { return mInkEvaporation->value(); }

  // Enums
  enum { X, Y, Z };

  // other accessors

  const WbVector3 &eastVector() const { return mEastVector; }
  const WbVector3 &northVector() const { return mNorthVector; }
  const WbVector3 &upVector() const { return mUpVector; }
  // returns the gravity vector (oriented along the down axis)
  const WbVector3 &gravityVector() const { return mGravityVector; }
  // returns a unit vector with the direction and orientation of the gravity
  const WbVector3 &gravityUnitVector() const { return mGravityUnitVector; }

  const WbReceiver *physicsReceiver() const { return mPhysicsReceiver; }

  void createOdeObjects() override;
  void createWrenObjects() override;

signals:
  void gpsCoordinateSystemChanged();
  void gpsReferenceChanged();
  void physicsDisableChanged();
  void titleChanged();
  void globalPhysicsPropertiesChanged();
  void optimalThreadCountChanged();
  void randomSeedChanged();

private:
  WbWorldInfo &operator=(const WbWorldInfo &);  // non copyable
  WbNode *clone() const override { return new WbWorldInfo(*this); }
  void exportNodeFields(WbWriter &writer) const override;
  void init(const WbVersion *version = NULL);

  // User accessible fields
  WbMFString *mInfo;
  WbSFString *mTitle;
  WbSFString *mWindow;
  WbSFDouble *mGravity;
  WbSFDouble *mCfm;
  WbSFDouble *mErp;
  WbSFString *mPhysics;
  WbSFDouble *mBasicTimeStep;
  WbSFDouble *mFps;
  WbSFInt *mOptimalThreadCount;
  WbSFDouble *mPhysicsDisableTime;
  WbSFDouble *mPhysicsDisableLinearThreshold;
  WbSFDouble *mPhysicsDisableAngularThreshold;
  WbSFNode *mDefaultDamping;
  WbSFDouble *mInkEvaporation;
  WbSFString *mCoordinateSystem;
  WbSFString *mGpsCoordinateSystem;
  WbSFVector3 *mGpsReference;
  WbSFDouble *mLineScale;
  WbSFDouble *mDragForceScale;
  WbSFDouble *mDragTorqueScale;
  WbSFInt *mRandomSeed;
  WbMFNode *mContactProperties;

  // physics receiver node
  WbReceiver *mPhysicsReceiver;

  // Gravity variables
  WbVector3 mEastVector;
  WbVector3 mNorthVector;
  WbVector3 mUpVector;
  WbVector3 mGravityVector;
  WbVector3 mGravityUnitVector;

  // Apply methods
  void applyLineScaleToWren();
  void applyToOdeGravity();
  void applyToOdeCfm();
  void applyToOdeErp();
  // Non-slot update methods
  void applyToOdeGlobalDamping();
  void applyToOdePhysicsDisableTime();
  void updateGravityBasis();

private slots:
  void updateBasicTimeStep();
  void updateFps();
  void updateOptimalThreadCount();
  void updateLineScale();
  void updateDragForceScale();
  void updateDragTorqueScale();
  void updateRandomSeed();
  void updateGravity();
  void updateCfm();
  void updateErp();
  void updateDefaultDamping();
  void updateCoordinateSystem();
  void updateGpsCoordinateSystem();
  void updateContactProperties();
  void displayOptimalThreadCountWarning();
};

#endif
