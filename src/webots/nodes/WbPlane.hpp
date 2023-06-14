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

#ifndef WB_PLANE_HPP
#define WB_PLANE_HPP

#include "WbGeometry.hpp"

class WbPlane : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbPlane(WbTokenizer *tokenizer = NULL);
  WbPlane(const WbPlane &other);
  explicit WbPlane(const WbNode &other);
  virtual ~WbPlane();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PLANE; }
  void postFinalize() override;
  void write(WbWriter &writer) const override;
  void createWrenObjects() override;
  void createResizeManipulator() override;
  void rescale(const WbVector3 &scale) override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;

  dGeomID createOdeGeom(dSpaceID space) override;
  void setOdePosition(const WbVector3 &translation) override;
  void setOdeRotation(const WbMatrix3 &matrix) override;

  void updateOdePlanePosition();

  // field accessors
  const WbVector2 &size() const;
  const WbVector2 scaledSize() const;

  // Setters
  void setSize(const WbVector2 &size);
  void setSize(double x, double y);
  void setX(double x);
  void setY(double y);

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
  // user accessible fields
  WbSFVector2 *mSize;

  WbPlane &operator=(const WbPlane &);  // non copyable
  WbNode *clone() const override { return new WbPlane(*this); }
  void init();
  bool sanitizeFields();
  void updateScale();

  void computePlaneParams(WbVector3 &n, double &d);

  // ray tracing
  bool computeCollisionPoint(WbVector3 &point, const WbRay &ray) const;
  void computeMinMax(const WbVector3 &point, WbVector3 &min, WbVector3 &max) const;

private slots:
  void updateSize();
  void updateLineScale();
};

#endif
