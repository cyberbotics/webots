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

//
//  WbTrack.hpp
//

// Implemented node class representing a generic robot tank track

#ifndef WB_TRACK_HPP
#define WB_TRACK_HPP

#include "WbSolid.hpp"

class WbBaseNode;
class WbBrake;
class WbLinearMotor;
class WbLogicalDevice;
class WbPositionSensor;
class WbSFInt;
class WbSFVector2;
class WbShape;
class WbTextureTransform;
class WbTrackWheel;

struct WrNode;

class WbTrack : public WbSolid {
  Q_OBJECT
public:
  explicit WbTrack(WbTokenizer *tokenizer = NULL);
  WbTrack(const WbTrack &other);
  explicit WbTrack(const WbNode &other);
  virtual ~WbTrack();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_TRACK; }
  void preFinalize() override;
  void createWrenObjects() override;
  void postFinalize() override;
  void prePhysicsStep(double ms) override;
  void exportNodeSubNodes(WbWriter &writer) const override;
  void exportNodeFields(WbWriter &writer) const override;
  void setMatrixNeedUpdate() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  double contactSurfaceVelocity() const { return mSurfaceVelocity; }
  double position() const { return mMotorPosition; }

  QVector<WbLogicalDevice *> devices() const;
  WbLinearMotor *motor() const;
  WbPositionSensor *positionSensor() const;
  WbBrake *brake() const;

  void animateMesh();

protected slots:
  void updateChildren() override;

private:
  WbTrack &operator=(const WbTrack &);  // non copyable
  WbNode *clone() const override { return new WbTrack(*this); }
  void init();

  WbMFNode *mDeviceField;
  WbSFVector2 *mTextureAnimationField;
  WbSFNode *mGeometryField;
  WbSFInt *mGeometriesCountField;

  // internal fields
  WbLinearMotor *mLinearMotor;
  WbBrake *mBrake;
  double mMotorPosition;
  double mSurfaceVelocity;
  dBodyID mBodyID;

  // wheels
  QVector<WbTrackWheel *> mWheelsList;
  void clearWheelsList();

  // texture animation
  WbShape *mShape;
  WbTextureTransform *mTextureTransform;
  QMap<QString, WbVector2> mSavedTextureTransformTranslations;

  // geometries animation
  struct PathSegment {
    PathSegment(const WbVector2 &pointA, const WbVector2 &pointB, double angle, double r, const WbVector2 &c,
                const WbVector2 &direction) :
      startPoint(pointA),
      endPoint(pointB),
      initialRotation(angle),
      radius(r),
      center(c),
      increment(direction) {}
    PathSegment(const WbVector2 &pointA, const WbVector2 &pointB, double angle, const WbVector2 &normalizedDirection) :
      startPoint(pointA),
      endPoint(pointB),
      initialRotation(angle),
      radius(-1),
      center(WbVector2()),
      increment(normalizedDirection) {}
    PathSegment() : initialRotation(M_PI), radius(0.0) {}

    WbVector2 startPoint;
    WbVector2 endPoint;
    double initialRotation;
    double radius;        // round path only
    WbVector2 center;     // round path only
    WbVector2 increment;  // straight path only
  };

  struct BeltPosition {
    BeltPosition(const WbVector2 &pos, double angle, int index) : position(pos), rotation(angle), segmentIndex(index) {}
    BeltPosition() : rotation(0.0), segmentIndex(0) {}

    WbVector2 position;
    double rotation;
    int segmentIndex;
  };

  double mPathLength;
  double mPathStepSize;
  QVector<PathSegment> mPathList;
  BeltPosition mFirstGeometryPosition;
  void computeBeltPath();
  BeltPosition computeNextGeometryPosition(BeltPosition current, double stepSize, bool segmentChanged = false) const;

  // animated mesh
  struct AnimatedObject {
    WbGeometry *geometry;
    WrMaterial *material;
    bool castShadows;
  };

  double mAnimationStepSize;
  QList<AnimatedObject *> mAnimatedObjectList;
  QList<WrTransform *> mBeltElements;
  QList<BeltPosition> mBeltPositions;
  QList<WrNode *> mWrenNodes;

  WbWrenMeshBuffers *createMeshBuffers(const WbGeometry *geom) const;
  void initAnimatedGeometriesBeltPosition();
  void clearAnimatedGeometries();
  bool findAndConnectAnimatedGeometries(bool connectSignals, QList<WbShape *> *shapeList);
  void exportAnimatedGeometriesMesh(WbWriter &writer) const;

private slots:
  void addDevice(int index);
  void updateDevices();
  void updateShapeNode();
  void updateTextureTransform();
  void updateWheelsList();
  void updateAnimatedGeometriesAfterFinalization(WbBaseNode *node);
  void updateAnimatedGeometries();
  void updateAnimatedGeometriesPath();
  void updateTextureAnimation();
};

#endif
