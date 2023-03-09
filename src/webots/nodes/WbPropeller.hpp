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

#ifndef WB_PROPELLER_HPP
#define WB_PROPELLER_HPP

#include "WbBaseNode.hpp"
#include "WbSFVector2.hpp"
#include "WbSFVector3.hpp"

struct WrTransform;
struct WrRenderable;
struct WrStaticMesh;
struct WrMaterial;

class WbLogicalDevice;
class WbSFNode;
class WbSolid;
class WbRotationalMotor;

class WbPropeller : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbPropeller(WbTokenizer *tokenizer = NULL);
  WbPropeller(const WbPropeller &other);
  explicit WbPropeller(const WbNode &other);
  virtual ~WbPropeller();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PROPELLER; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  void propagateSelection(bool selected) override;
  void setMatrixNeedUpdate() override;
  void write(WbWriter &writer) const override;
  void reset(const QString &id) override;
  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  void prePhysicsStep(double ms);

  // field accessors
  const WbVector3 &axis() const { return mShaftAxis->value(); }  // thrust direction
  // point where the thrust applies (expressed in relative coordinates)
  const WbVector3 &centerOfThrust() const { return mCenterOfThrust->value(); }
  const WbVector2 &thrustConstants() const { return mThrustConstants->value(); }  // constants used to compute thrust output
  const WbVector2 &torqueConstants() const { return mTorqueConstants->value(); }  // constants used to compute torque output
  WbRotationalMotor *motor() const;
  enum HelixType { FAST_HELIX, SLOW_HELIX };
  WbSolid *helix(HelixType type) const;
  WbSolid *helix() const { return mHelix; }  // current helix
  WbLogicalDevice *device() const;
  double position() const { return mPosition; }

  double currentThrust() const { return mCurrentThrust; }
  double currentTorque() const { return mCurrentTorque; }

private:
  // Scene Tree fields
  WbSFVector3 *mShaftAxis;
  WbSFVector3 *mCenterOfThrust;
  WbSFVector2 *mThrustConstants;
  WbSFVector2 *mTorqueConstants;
  WbSFDouble *mFastHelixThreshold;
  WbSFNode *mDevice;
  WbSFNode *mFastHelix;
  WbSFNode *mSlowHelix;
  double mPosition;
  WbSolid *mHelix;
  HelixType mHelixType;

  WbVector3 mNormalizedAxis;

  double mCurrentThrust;
  double mCurrentTorque;

  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrStaticMesh *mMesh;
  WrMaterial *mMaterial;

  WbPropeller &operator=(const WbPropeller &);  // non copyable
  WbNode *clone() const override { return new WbPropeller(*this); }
  void init();
  void updateHelix(double angularSpeed, bool ode = true);

private slots:
  // Update methods
  void updateShaftAxis();
  void updateShaftAxisRepresentation();
  void updateDevice();
  void updateHelixRepresentation();
  void updateOptionalRendering(int option);
};
#endif
