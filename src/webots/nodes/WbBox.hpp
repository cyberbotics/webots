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

#ifndef WB_BOX_HPP
#define WB_BOX_HPP

#include "WbGeometry.hpp"

class WbSFVector3;

class WbBox : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbBox(WbTokenizer *tokenizer = NULL);
  WbBox(const WbBox &other);
  explicit WbBox(const WbNode &other);
  virtual ~WbBox();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_BOX; }
  void postFinalize() override;
  dGeomID createOdeGeom(dSpaceID space) override;
  void createWrenObjects() override;
  void createResizeManipulator() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void rescale(const WbVector3 &scale) override;

  // field accessors
  const WbVector3 &size() const;
  const WbVector3 scaledSize() const;

  // field setters
  void setSize(const WbVector3 &size);
  void setSize(double x, double y, double z);
  void setX(double x);
  void setY(double y);
  void setZ(double z);

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const WbRay &ray) const override;

  // friction
  WbVector3 computeFrictionDirection(const WbVector3 &normal) const override;

  // Non-recursive texture mapping
  WbVector2 nonRecursiveTextureSizeFactor() const override { return WbVector2(4, 2); }

  static WbVector2 computeTextureCoordinate(const WbVector3 &minBound, const WbVector3 &maxBound, const WbVector3 &point,
                                            bool nonRecursive, int intersectedFace = -1);
  static int findIntersectedFace(const WbVector3 &minBound, const WbVector3 &maxBound, const WbVector3 &intersectionPoint);

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  QStringList fieldsToSynchronizeWithX3D() const override;

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;

  // Fluid
  void checkFluidBoundingObjectOrientation() override;

private:
  WbBox &operator=(const WbBox &);  // non copyable
  WbNode *clone() const override { return new WbBox(*this); }
  void init();

  // user accessible fields
  WbSFVector3 *mSize;

  bool sanitizeFields();
  void updateScale();

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;

  // ray tracing
  // compute collision point and return distance
  double computeLocalCollisionPoint(WbVector3 &point, int &faceIndex, const WbRay &ray) const;

private slots:
  void updateSize();
  void updateLineScale();
};

#endif
