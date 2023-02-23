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

#ifndef WB_CONE_HPP
#define WB_CONE_HPP

#include "WbGeometry.hpp"

class WbSFDouble;

class WbCone : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbCone(WbTokenizer *tokenizer = NULL);
  WbCone(const WbCone &other);
  explicit WbCone(const WbNode &other);
  virtual ~WbCone();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CONE; }
  void postFinalize() override;
  void createWrenObjects() override;
  void createResizeManipulator() override;
  void rescale(const WbVector3 &scale) override;

  // Fields accessors
  double height() const;
  double scaledHeight() const;
  double bottomRadius() const;
  double scaledBottomRadius() const;

  // Fields setters
  void setHeight(double h);
  void setBottomRadius(double r);

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const WbRay &ray) const override;

  // friction
  WbVector3 computeFrictionDirection(const WbVector3 &normal) const override;

  // Non-recursive texture mapping
  WbVector2 nonRecursiveTextureSizeFactor() const override { return WbVector2(2, 1); }

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  QStringList fieldsToSynchronizeWithX3D() const override;

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;
  void exportNodeFields(WbWriter &writer) const override;

private:
  // user accessible fields
  WbSFDouble *mBottomRadius;
  WbSFDouble *mHeight;
  WbSFBool *mSide;
  WbSFBool *mBottom;
  WbSFInt *mSubdivision;

  bool sanitizeFields();

  // WREN
  void buildWrenMesh();
  virtual void updateScale();

  WbCone &operator=(const WbCone &);  // non copyable
  WbNode *clone() const override { return new WbCone(*this); }
  void init();

  // ray tracing
  // compute collision point and return the distance
  double computeLocalCollisionPoint(WbVector3 &point, const WbRay &ray) const;

private slots:
  void updateBottomRadius();
  void updateHeight();
  void updateSide();
  void updateBottom();
  void updateSubdivision();
};

#endif
