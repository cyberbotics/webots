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

#ifndef WB_ELEVATION_GRID_HPP
#define WB_ELEVATION_GRID_HPP

#include "WbGeometry.hpp"
#include "WbMFDouble.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSFNode.hpp"

class WbVector3;

typedef struct dxHeightfieldData *dHeightfieldDataID;

class WbElevationGrid : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbElevationGrid(WbTokenizer *tokenizer = NULL);
  WbElevationGrid(const WbElevationGrid &other);
  explicit WbElevationGrid(const WbNode &other);
  virtual ~WbElevationGrid();

  // field accessors
  // getters
  double xSpacing() const { return mXSpacing->value(); }
  double ySpacing() const { return mYSpacing->value(); }
  int xDimension() const { return mXDimension->value(); }
  int yDimension() const { return mYDimension->value(); }
  int height(int index) const { return mHeight->item(index); }
  double heightRange() const { return mMaxHeight - mMinHeight; }
  // setters
  void setXspacing(double x) { mXSpacing->setValue(x); }
  void setYspacing(double y) { mYSpacing->setValue(y); }
  void setHeightScaleFactor(double ratio) { mHeight->multiplyAllItems(ratio); }

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_ELEVATION_GRID; }
  void preFinalize() override;
  void postFinalize() override;
  dGeomID createOdeGeom(dSpaceID space) override;
  void createWrenObjects() override;
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

signals:
  void validElevationGridInserted();

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;
  void exportNodeFields(WbWriter &writer) const override;

private:
  WbElevationGrid &operator=(const WbElevationGrid &);  // non copyable
  WbNode *clone() const override { return new WbElevationGrid(*this); }
  void init();

  // user accessible fields
  WbMFDouble *mHeight;
  WbSFInt *mXDimension;
  WbSFDouble *mXSpacing;
  WbSFInt *mYDimension;
  WbSFDouble *mYSpacing;
  WbSFDouble *mThickness;

  // other variables
  double mMinHeight;  // min value in "height" field
  double mMaxHeight;  // max value in "height" field
  dHeightfieldDataID mHeightfieldData;
  double *mData;
  void checkHeight();
  double width() const { return mXSpacing->value() * (mXDimension->value() - 1); }
  double depth() const { return mYSpacing->value() * (mYDimension->value() - 1); }
  double height() const { return mMaxHeight - mMinHeight; }

  bool sanitizeFields();

  // WREN
  void buildWrenMesh();
  void updateScale();

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;
  bool setOdeHeightfieldData();
  double scaledWidth() const;
  double scaledDepth() const;
  double heightScaleFactor() const { return fabs(absoluteScale().y()); }

  // ray tracing
  double computeLocalCollisionPoint(const WbRay &ray, WbVector3 &localCollisionPoint) const;
  void computeMinMax(const WbVector3 &point, WbVector3 &min, WbVector3 &max) const;

private slots:
  void updateHeight();
  void updateXDimension();
  void updateXSpacing();
  void updateYDimension();
  void updateYSpacing();
  void updateThickness();
  void updateLineScale();
};

#endif
