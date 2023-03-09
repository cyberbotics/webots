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

#ifndef WB_PHYSICS_HPP
#define WB_PHYSICS_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"

// Note: inertia and mass properties must be specified in accordance with one of these two modes:
// Custom mode: both the center of mass and the inertia matrix, together with a positive mass, are specified.
// Default mode based on bounding object: the inertia matrix is not specified but the mass or the density has to be positive.
// Note: In the latter case, the inertia matrix is the inertia matrix of the bounding object around its intrinsic center of
// mass. Thus the solid behaves as if its mass distribution is translated to 'centerOfMass' while its physical boundary does not
// move. The mode is said to be invalid if none of the above apply. ODE updates are called only if the mode is valid, so that
// ODE always keeps the latest valid settings.

class WbDamping;
class WbMFVector3;
class WbSFNode;

class WbPhysics : public WbBaseNode {
  Q_OBJECT

public:
  // if mMode == INVALID, ODE uses the last valid specifications. If the loaded specifications were invalid, it uses bounding
  // object based computations.
  enum ModeType { CUSTOM_INERTIA_MATRIX, BOUNDING_OBJECT_BASED, INVALID };

  // constructors and destructor
  explicit WbPhysics(WbTokenizer *tokenizer = NULL);
  WbPhysics(const WbPhysics &other);
  explicit WbPhysics(const WbNode &other);
  virtual ~WbPhysics();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PHYSICS; }
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;

  // field accessors
  double density() const { return mDensity->value(); }
  void setDensity(double density, bool skipUpdate = false);
  double mass() const { return mMass->value(); }
  void setMass(double mass, bool skipUpdate = false);
  const WbMFVector3 &centerOfMass() const { return *mCenterOfMass; }
  void setCenterOfMass(double x, double y, double z, bool skipUpdate = false);
  const WbMFVector3 &inertiaMatrix() const { return *mInertiaMatrix; }
  void setInertiaMatrix(double v0x, double v0y, double v0z, double v1x, double v1y, double v1z, bool skipUpdate = false);
  WbDamping *damping() const;

  // other acessors
  bool hasApositiveMassOrDensity() const { return (mMass->value() > 0.0 || mDensity->value() > 0.0); }
  ModeType mode() const { return mMode; }
  void updateMode();

  // validity checks
  void checkMassAndDensity() const;
  void checkInertiaMatrix(bool showInfo);

signals:
  void massOrDensityChanged();
  void centerOfMassChanged();
  void inertialPropertiesChanged();
  void dampingChanged();
  void modeSwitched();

protected:
  bool exportNodeHeader(WbWriter &writer) const override;

private:
  // user accessible fields
  WbSFDouble *mDensity;
  WbSFDouble *mMass;
  WbMFVector3 *mCenterOfMass;
  WbMFVector3 *mInertiaMatrix;
  WbSFNode *mDamping;

  WbPhysics &operator=(const WbPhysics &);  // non copyable
  WbNode *clone() const override { return new WbPhysics(*this); }
  void init();
  bool mSkipUpdate;
  bool hasAvalidInertiaMatrix() const { return mHasAvalidInertiaMatrix; }
  void skipUpdate(bool b) { mSkipUpdate = b; }

  // validity check
  void checkDensity();
  void checkMass();
  void checkCenterOfMass() const;

  // Validity flags
  bool mHasAvalidInertiaMatrix;
  ModeType mMode;

private slots:
  void updateDensity();
  void updateMass();
  void updateCenterOfMass();
  void updateInertiaMatrix();
  void updateDamping();
};

#endif
