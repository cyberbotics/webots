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

#ifndef WB_INDEXED_LINE_SET_HPP
#define WB_INDEXED_LINE_SET_HPP

#include "WbGeometry.hpp"

class WbCoordinate;

struct WrMaterial;

class WbIndexedLineSet : public WbGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbIndexedLineSet(WbTokenizer *tokenizer = NULL);
  WbIndexedLineSet(const WbIndexedLineSet &other);
  explicit WbIndexedLineSet(const WbNode &other);
  virtual ~WbIndexedLineSet();

  // field accessors
  WbCoordinate *coord() const;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_INDEXED_LINE_SET; }
  void postFinalize() override;
  void createWrenObjects() override;
  void setWrenMaterial(WrMaterial *material, bool castShadows) override;
  void rescale(const WbVector3 &scale) override {}
  void reset(const QString &id) override;

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(WbVector2 &uv, const WbRay &ray, int textureCoordSet = 0) const override { return false; }
  double computeDistance(const WbRay &ray) const override { return -1; }

  // resize manipulator
  bool hasResizeManipulator() const override { return false; }

  // friction
  WbVector3 computeFrictionDirection(const WbVector3 &normal) const override;

  QStringList fieldsToSynchronizeWithX3D() const override;

protected:
  // reimplemented protected functions
  bool isShadedGeometryPickable() override { return false; }

private:
  // user accessible fields
  WbSFNode *mCoord;
  WbMFInt *mCoordIndex;

  bool sanitizeFields();

  WbIndexedLineSet &operator=(const WbIndexedLineSet &);  // non copyable
  WbNode *clone() const override { return new WbIndexedLineSet(*this); }
  void init();

  // WREN
  void buildWrenMesh();
  int computeCoordsData(float *data);
  int estimateIndexCount(bool isOutlineMesh = false) const;

private slots:
  void updateCoord();
  void updateCoordIndex();
  void updateLineScale();
};

#endif
