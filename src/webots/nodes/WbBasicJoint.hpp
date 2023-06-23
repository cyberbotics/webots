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

// Abstract node class representing the most general mechanical joint

#ifndef WB_BASIC_JOINT_HPP
#define WB_BASIC_JOINT_HPP

#include "WbBaseNode.hpp"
#include "WbOdeTypes.hpp"
#include "WbRotation.hpp"
#include "WbVector3.hpp"

class WbBoundingSphere;
class WbSlot;
class WbSolid;
class WbSolidReference;

struct WrTransform;
struct WrRenderable;
struct WrStaticMesh;
struct WrMaterial;

class WbBasicJoint : public WbBaseNode {
  Q_OBJECT

public:
  virtual ~WbBasicJoint();

  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  virtual void prePhysicsStep(double ms) {}
  virtual void postPhysicsStep() {}
  virtual bool setJoint();
  void write(WbWriter &writer) const override;
  virtual bool resetJointPositions();
  void setMatrixNeedUpdate() override;
  virtual void updateOdeWorldCoordinates() {}
  virtual void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const = 0;
  void reset(const QString &id) override;
  void save(const QString &id) override;
  void updateSegmentationColor(const WbRgb &color) override;

  void setSolidEndPoint(WbSolid *solid);
  void setSolidEndPoint(WbSolidReference *solid);
  void setSolidEndPoint(WbSlot *slot);

  WbSolid *solidEndPoint() const;
  WbSolidReference *solidReference() const;
  WbSolid *solidParent() const;
  virtual dJointID jointID() const { return mJoint; }
  // endPoint Solid translation and rotation if joint position is 0
  const WbVector3 &zeroEndPointTranslation() const { return mEndPointZeroTranslation; }
  const WbRotation &zeroEndPointRotation() const { return mEndPointZeroRotation; }
  bool isEnabled() const;

  // ray tracing
  WbBoundingSphere *boundingSphere() const override;

  void updateAfterParentPhysicsChanged();
  virtual void updateEndPointZeroTranslationAndRotation() = 0;

  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;
  QString endPointName() const override;

public slots:
  void updateEndPoint();

signals:
  void endPointChanged(WbBaseNode *node);

protected:
  WbBasicJoint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  WbBasicJoint(const WbBasicJoint &other);
  WbBasicJoint(const WbNode &other);

  virtual void setOdeJoint(dBodyID body, dBodyID parentBody);
  // anchor point on an hinge axis; also defined for a slider axis to set its graphical represention position
  virtual WbVector3 anchor() const;

  dJointID mJoint;
  // joint attached to the static environment (internally in ODE the first body cannot be NULL)
  bool mIsReverseJoint;
  // the second end of the joint, the first being its parent; this is is either a Solid or a SolidReference
  WbSFNode *mEndPoint;
  // axis, anchor, initial position, damping and spring constants, min and max stop
  WbSFNode *mParameters;

  // variables and methods about the endPoint Solid translation and rotation when joint position is 0
  WbVector3 mEndPointZeroTranslation;
  WbRotation mEndPointZeroRotation;
  void retrieveEndPointSolidTranslationAndRotation(WbVector3 &it, WbRotation &ir) const;
  dJointID mSpringAndDamperMotor;  // ODE linear motor used to simulate spring and damper effects by means of stops
  virtual void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) = 0;

  bool mIsEndPointPositionChangedByJoint;

  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrStaticMesh *mMesh;
  WrMaterial *mMaterial;

  const bool isJoint() const override { return true; }

protected slots:
  virtual void updateParameters() = 0;
  void updateSpringAndDampingConstants();
  virtual void updateOptionalRendering(int option);

private:
  WbBasicJoint &operator=(const WbBasicJoint &);  // non copyable
  void init();

private slots:
  virtual void updateJointAxisRepresentation() = 0;  // updates the WREN joint axis when the line scale changes
  void updateEndPointPosition();
  void updateBoundingSphere(WbBaseNode *subNode);
};
#endif
