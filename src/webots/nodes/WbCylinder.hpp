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

#ifndef WB_CYLINDER_HPP
#define WB_CYLINDER_HPP

#include "WbGeometry.hpp"
#include "WbSFDouble.hpp"

class WbSFInt;

class WbCylinder : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbCylinder(WbTokenizer *tokenizer = NULL);
  WbCylinder(const WbCylinder &other);
  explicit WbCylinder(const WbNode &other);
  virtual ~WbCylinder();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CYLINDER; }
  void postFinalize() override;
  void createWrenObjects() override;
  dGeomID createOdeGeom(dSpaceID space) override;
  void createResizeManipulator() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  bool shallExport() const override;
  void rescale(const WbVector3 &scale) override;

  // field accessors
  double radius() const { return mRadius->value(); }
  double scaledRadius() const;
  double height() const { return mHeight->value(); }
  double scaledHeight() const;

  // field setters
  void setRadius(double r) { mRadius->setValue(r); }
  void setHeight(double h) { mHeight->setValue(h); }

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const WbRay &ray) const override;

  // friction
  WbVector3 computeFrictionDirection(const WbVector3 &normal) const override;

  // Non-recursive texture mapping
  WbVector2 nonRecursiveTextureSizeFactor() const override { return WbVector2(2, 2); }

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  QStringList fieldsToSynchronizeWithX3D() const override;

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;
  void exportNodeFields(WbWriter &writer) const override;

private:
  WbCylinder &operator=(const WbCylinder &);  // non copyable
  WbNode *clone() const override { return new WbCylinder(*this); }
  void init();

  // user accessible fields
  WbSFBool *mBottom;
  WbSFDouble *mHeight;
  WbSFDouble *mRadius;
  WbSFBool *mSide;
  WbSFBool *mTop;
  WbSFInt *mSubdivision;

  bool sanitizeFields();

  // WREN
  void buildWrenMesh();
  void updateScale();

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;

  // ray tracing
  double computeLocalCollisionPoint(WbVector3 &point, int &faceIndex,
                                    const WbRay &ray) const;  // compute collision point and return the distance

private slots:
  void updateBottom();
  void updateRadius();
  void updateHeight();
  void updateSide();
  void updateTop();
  void updateSubdivision();
  void updateLineScale();
};

#endif
