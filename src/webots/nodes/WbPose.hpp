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

#ifndef WB_POSE_HPP
#define WB_POSE_HPP

//
// Description: a node that defines a 3D coordinate system transformation
//
// Inherited by: WbSolid
//

#include "WbAbstractPose.hpp"
#include "WbGroup.hpp"
#include "WbShape.hpp"

class WbPose : public WbGroup, public WbAbstractPose {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbPose(WbTokenizer *tokenizer = NULL);
  WbPose(const WbPose &other);
  explicit WbPose(const WbNode &other);
  virtual ~WbPose();

  // reimplemented functions
  int nodeType() const override { return WB_NODE_POSE; }
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void updateCollisionMaterial(bool isColliding = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setMatrixNeedUpdate() override;
  void connectGeometryField(bool dynamic);
  void reset(const QString &id) override;
  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override {
    return QList<const WbBaseNode *>() << this;
  }

  // accessors to stored fields
  const WbVector3 translationFromFile(const QString &id) const { return mSavedTranslations[id]; }
  const WbRotation rotationFromFile(const QString &id) const { return mSavedRotations[id]; }
  void setTranslationFromFile(const WbVector3 &translation) { mSavedTranslations[stateId()] = translation; }
  void setRotationFromFile(const WbRotation &rotation) { mSavedRotations[stateId()] = rotation; }

  void save(const QString &id) override;

  // update of ODE data stored in geometry() for WbPose lying into a boundingObject
  void applyToOdeData(bool correctMass = true);

  // for a Pose lying into a boundingObject
  void listenToChildrenField();
  inline WbGeometry *geometry() const;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = false) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;

  void enablePoseChangedSignal() const { mPoseChangedSignalEnabled = true; }
  void emitTranslationOrRotationChangedByUser() override;
  WbVector3 translationFrom(const WbNode *fromNode) const;
  WbMatrix3 rotationMatrixFrom(const WbNode *fromNode) const;

  // export
  void exportBoundingObjectToX3D(WbWriter &writer) const override;
  QStringList fieldsToSynchronizeWithX3D() const override;

public slots:
  virtual void updateRotation();
  virtual void updateTranslation();
  virtual void updateTranslationAndRotation();

signals:
  void geometryInPoseInserted();
  void poseChanged();
  void translationOrRotationChangedByUser();

protected:
  // this constructor is reserved for derived classes only
  WbPose(const QString &modelName, WbTokenizer *tokenizer);

  mutable bool mPoseChangedSignalEnabled;

private:
  WbPose &operator=(const WbPose &);  // non copyable
  WbNode *clone() const override { return new WbPose(*this); }
  void init();

  // Positions and orientations storage
  QMap<QString, WbVector3> mSavedTranslations;
  QMap<QString, WbRotation> mSavedRotations;

  void applyToOdeGeomRotation();
  void applyToOdeGeomPosition(bool correctMass = true);
  void applyToOdeMass(WbGeometry *g, dGeomID geom);
  void destroyPreviousOdeGeoms();
  WbShape *shape() const;

private slots:
  void createOdeGeom(int index = -1);
  void createOdeGeomIfNeeded();
  void notifyJerk();
};

inline WbGeometry *WbPose::geometry() const {
  if (childCount() == 0)
    return NULL;

  WbBaseNode *const c = child(0);

  if (c->nodeType() == WB_NODE_SHAPE)
    return static_cast<WbShape *>(c)->geometry();

  return dynamic_cast<WbGeometry *>(c);
}

#endif
