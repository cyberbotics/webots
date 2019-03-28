// Copyright 1996-2019 Cyberbotics Ltd.
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
  void preFinalize() override;
  void postFinalize() override;
  void reset() override;

  // field accessors
  const WbMFString &info() const { return *mInfo; }
  const QString &title() const { return mTitle->value(); }
  const QString &window() const { return mWindow->value(); }
  const WbVector3 &gravity() const { return mGravity->value(); }
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
  const WbVector3 &northDirection() const { return mNorthDirection->value(); }
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

  // returns a unit vector with same direction and orientation as gravity
  const WbVector3 &gravityUnitVector() const { return mGravityUnitVector; }
  // returns an orthonormal basis (b[X], b[Y] = -gravity().normalized(), b[Z])
  const WbVector3 *gravityBasis() const { return mGravityBasis; }

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
  void exportNodeFields(WbVrmlWriter &writer) const override;
  void init();

  // User accessible fields
  WbMFString *mInfo;
  WbSFString *mTitle;
  WbSFString *mWindow;
  WbSFVector3 *mGravity;
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
  WbSFVector3 *mNorthDirection;
  WbSFString *mGpsCoordinateSystem;
  WbSFVector3 *mGpsReference;
  WbSFDouble *mLineScale;
  WbSFInt *mRandomSeed;
  WbMFNode *mContactProperties;

  // physics receiver node
  WbReceiver *mPhysicsReceiver;

  // Gravity variables
  WbVector3 mGravityUnitVector;
  WbVector3 mGravityBasis[3];  // An orthonormal basis (b[X], b[Y] = -gravity().normalized(), b[Z])

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
  void updateRandomSeed();
  void updateGravity();
  void updateCfm();
  void updateErp();
  void updateDefaultDamping();
  void updateNorthDirection();
  void updateGpsCoordinateSystem();
  void updateContactProperties();
  void displayOptimalThreadCountWarning();
};

#endif
