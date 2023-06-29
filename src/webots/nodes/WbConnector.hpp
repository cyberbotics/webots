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

#ifndef WB_CONNECTOR_HPP
#define WB_CONNECTOR_HPP

#include "WbSolidDevice.hpp"

class WbSensor;
class WbVector3;
typedef double dQuaternion[4];

struct WrRenderable;
struct WrMaterial;
struct WrStaticMesh;
struct WrTransform;

class WbConnector : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbConnector(WbTokenizer *tokenizer = NULL);
  WbConnector(const WbConnector &other);
  explicit WbConnector(const WbNode &other);
  virtual ~WbConnector();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CONNECTOR; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(WbDataStream &stream) override;
  void writeConfigure(WbDataStream &) override;
  void createWrenObjects() override;
  void prePhysicsStep(double ms) override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  enum FaceType { UNKNOWN, SYMMETRIC, ACTIVE, PASSIVE };
  FaceType faceType() const { return mFaceType; }

  // interface with auto-assembly mechanism
  static bool isAllowingMouseMotion();
  static void solidHasMoved(WbSolid *solid);

private:
  // fields
  WbSFString *mType;               // connection type "symmetric", "active" or "passive"
  WbSFBool *mIsLocked;             // current locked state
  WbSFBool *mAutoLock;             // locks automatically in case of presence (but only when "isLocked")
  WbSFBool *mUnilateralLock;       // for "symmetric" type only: enable unilateral locking
  WbSFBool *mUnilateralUnlock;     // for "symmetric" type only: enable unilateral unlocking
  WbSFDouble *mDistanceTolerance;  // acceptable error in the distance between both Connectors
  WbSFDouble *mAxisTolerance;      // acceptable error angle between the two normals vectors
  WbSFDouble *mRotationTolerance;  // acceptable error in rotation angle
  WbSFInt *mNumberOfRotations;     // number of possible docking rotations
  WbSFBool *mSnap;                 // should automatically snap with peer connector when locked
  WbSFDouble *mTensileStrength;    // max pull force that the connector can withstand without breaking (Newtons)
  WbSFDouble *mShearStrength;      // max shear force that the connector can withstand without breaking (Newtons)

  // other stuff
  FaceType mFaceType;    // UNKNOWN, SYMMETRIC, ACTIVE or PASSIVE
  double mMinDist2;      // squared distanceTolerance
  dJointID mFixedJoint;  // ODE joint that does the connection
  WbConnector *mPeer;    // peer connector or NULL
  bool mStartup;         // do once flag
  WbSensor *mSensor;     // presence sensor
  int mValue;
  bool mIsJointInversed;
  QMap<QString, bool> mIsInitiallyLocked;
  bool mNeedToReconfigure;

  WrTransform *mTransform;
  WrTransform *mAxesTransform;
  WrTransform *mRotationsTransform;

  WrRenderable *mAxisRenderable[2];
  WrRenderable *mRotationsRenderable;

  WrMaterial *mMaterial[3];

  WrStaticMesh *mAxisMesh[2];
  WrStaticMesh *mRotationsMesh;

  WbConnector &operator=(const WbConnector &);  // non copyable
  WbNode *clone() const override { return new WbConnector(*this); }
  void addConfigure(WbDataStream &);

  bool isReadyToAttachTo(const WbConnector *other) const;
  void attachTo(WbConnector *other);
  void detachFromPeer();
  void createFixedJoint(WbConnector *other, const dBodyID b1, const dBodyID b2);
  void destroyFixedJoint();
  void lock();
  void unlock();
  void computeValue();
  WbConnector *detectPresence() const;
  bool isCompatibleWith(const WbConnector *other) const;
  double getDistance2(const WbConnector *other) const;
  bool isAlignedWith(const WbConnector *other) const;
  bool isXAlignedWith(const WbConnector *other) const;
  bool isZAlignedWith(const WbConnector *other) const;
  void detachIfForceExceedStrength();
  double findClosestRotationalAlignment(double alpha) const;
  void snapXAxes(WbConnector *other, dQuaternion q, const dBodyID b1, const dBodyID b2);
  void snapOrigins(WbConnector *other, const dBodyID b1, const dBodyID b2);
  void snapRotation(WbConnector *other, const WbVector3 &z1, const WbVector3 &z2, const dBodyID b1, const dBodyID b2);
  void rotateBodies(WbConnector *other, const dQuaternion q, const dBodyID b1, const dBodyID b2);
  void getOriginInWorldCoordinates(double out[3]) const;
  void snapNow(WbConnector *other, const dBodyID b1, const dBodyID b2);
  double getEffectiveTensileStrength() const;
  double getEffectiveShearStrength() const;
  void init();
  void hasMoved();
  void assembleWith(WbConnector *other);
  void assembleAxes(WbConnector *other);
  void applyOptionalRenderingToWren();
  void deleteWrenObjects();

private slots:
  void updateType();
  void updateIsLocked();
  void updateNumberOfRotations();
  void updateDistanceTolerance();
  void updateAxisTolerance();
  void updateRotationTolerance();
  void updateTensileStrength();
  void updateShearStrength();
  void updateLineScale() override;
  void updateOptionalRendering(int option);
};

#endif
