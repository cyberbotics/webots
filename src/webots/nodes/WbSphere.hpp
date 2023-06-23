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

#ifndef WB_SPHERE_HPP
#define WB_SPHERE_HPP

#include "WbGeometry.hpp"
#include "WbSFDouble.hpp"

class WbVector3;

class WbSphere : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbSphere(WbTokenizer *tokenizer = NULL);
  WbSphere(const WbSphere &other);
  explicit WbSphere(const WbNode &other);
  virtual ~WbSphere();

  // field accessors
  double radius() const { return mRadius->value(); }
  double scaledRadius() const;

  // field setters
  void setRadius(double r) { mRadius->setValue(r); }

  // reimplemented functions
  int nodeType() const override { return WB_NODE_SPHERE; }
  void postFinalize() override;
  void createWrenObjects() override;
  dGeomID createOdeGeom(dSpaceID space) override;
  void createResizeManipulator() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void rescale(const WbVector3 &scale) override;

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const WbRay &ray) const override;

  // friction
  WbVector3 computeFrictionDirection(const WbVector3 &normal) const override;

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  QStringList fieldsToSynchronizeWithX3D() const override;

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;
  void exportNodeFields(WbWriter &writer) const override;

private:
  WbSphere &operator=(const WbSphere &);  // non copyable
  WbNode *clone() const override { return new WbSphere(*this); }
  void init();

  // user accessible fields
  WbSFDouble *mRadius;
  WbSFInt *mSubdivision;
  WbSFBool *mIco;

  bool sanitizeFields();

  // WREN
  void buildWrenMesh();
  void updateScale();

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;

  // ray tracing
  bool computeCollisionPoint(WbVector3 &point, const WbRay &ray) const;

private slots:
  void updateRadius();
  void updateMesh();
  void updateLineScale();
};

#endif
