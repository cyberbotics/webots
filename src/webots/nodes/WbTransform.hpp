// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_TRANSFORM_HPP
#define WB_TRANSFORM_HPP

//
// Description: a node that defines a 3D coordinate system transformation
//
// Inherited by: WbSolid
//

#include "WbAbstractTransform.hpp"
#include "WbGroup.hpp"
#include "WbShape.hpp"

class WbTransform : public WbGroup, public WbAbstractTransform {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbTransform(WbTokenizer *tokenizer = NULL);
  WbTransform(const WbTransform &other);
  explicit WbTransform(const WbNode &other);
  virtual ~WbTransform();

  // reimplemented functions
  int nodeType() const override { return WB_NODE_TRANSFORM; }
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void updateCollisionMaterial(bool isColliding = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setScaleNeedUpdate() override;
  void setMatrixNeedUpdate() override;
  void connectGeometryField(bool dynamic);

  // Scaling
  int constraintType() const override;

  // update of ODE data stored in geometry() for WbTransform lying into a boundingObject
  void applyToOdeData(bool correctMass = true);

  // for a Transform lying into a boundingObject
  void listenToChildrenField();
  inline WbGeometry *geometry() const;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = false) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;

  // resize/scale manipulator
  bool hasResizeManipulator() const override { return WbAbstractTransform::hasResizeManipulator(); }
  void attachResizeManipulator() override { WbAbstractTransform::attachResizeManipulator(); }
  void detachResizeManipulator() const override { WbAbstractTransform::detachResizeManipulator(); }
  void updateResizeHandlesSize() override { WbAbstractTransform::updateResizeHandlesSize(); }
  void setResizeManipulatorDimensions() { WbAbstractTransform::setResizeManipulatorDimensions(); }
  void setUniformConstraintForResizeHandles(bool enabled) override {
    WbAbstractTransform::setUniformConstraintForResizeHandles(enabled);
  }

  // translate-rotate manipulator
  void updateTranslateRotateHandlesSize() override { WbAbstractTransform::updateTranslateRotateHandlesSize(); }
  void attachTranslateRotateManipulator() override { WbAbstractTransform::attachTranslateRotateManipulator(); }
  void detachTranslateRotateManipulator() override { WbAbstractTransform::detachTranslateRotateManipulator(); }

  void enablePoseChangedSignal() const { mPoseChangedSignalEnabled = true; }
  void emitTranslationOrRotationChangedByUser() override;
  WbVector3 translationFrom(const WbNode *fromNode) const;
  WbMatrix3 rotationMatrixFrom(const WbNode *fromNode) const;

  // export
  void exportBoundingObjectToX3D(WbVrmlWriter &writer) const override;
  QStringList fieldsToSynchronizeWithX3D() const override;

public slots:
  virtual void updateRotation();
  virtual void updateTranslation();
  virtual void updateTranslationAndRotation();
  void showResizeManipulator(bool enabled) override;

signals:
  void geometryInTransformInserted();
  void poseChanged();
  void translationOrRotationChangedByUser();

protected:
  // this constructor is reserved for derived classes only
  WbTransform(const QString &modelName, WbTokenizer *tokenizer);
  void applyToScale() override;

  void createScaleManipulator() override;

protected slots:
  virtual void updateScale(bool warning = false);
  void updateConstrainedHandleMaterials();

private:
  WbTransform &operator=(const WbTransform &);  // non copyable
  WbNode *clone() const override { return new WbTransform(*this); }
  void init();

  mutable bool mPoseChangedSignalEnabled;

  void applyToOdeGeomRotation();
  void applyToOdeGeomPosition(bool correctMass = true);
  void applyToOdeMass(WbGeometry *g, dGeomID geom);
  void applyToOdeScale();
  void destroyPreviousOdeGeoms();
  WbShape *shape() const;

private slots:
  void createOdeGeom(int index = -1);
  void createOdeGeomIfNeeded();
  void notifyJerk();
};

inline WbGeometry *WbTransform::geometry() const {
  if (childCount() == 0)
    return NULL;

  WbBaseNode *const c = child(0);

  if (c->nodeType() == WB_NODE_SHAPE)
    return static_cast<WbShape *>(c)->geometry();

  return dynamic_cast<WbGeometry *>(c);
}

#endif
