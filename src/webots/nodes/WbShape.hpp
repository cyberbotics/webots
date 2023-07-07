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

#ifndef WB_SHAPE_HPP
#define WB_SHAPE_HPP

#include "WbBaseNode.hpp"
#include "WbGeometry.hpp"
#include "WbSFNode.hpp"

class WbAbstractAppearance;
class WbAppearance;
class WbPbrAppearance;
class WbRay;

class WbShape : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbShape(WbTokenizer *tokenizer = NULL);
  WbShape(const WbShape &other);
  explicit WbShape(const WbNode &other);
  virtual ~WbShape();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SHAPE; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setScaleNeedUpdate() override;
  bool isAValidBoundingObject(bool checkOde = true, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void propagateSelection(bool selected) override;
  void reset(const QString &id) override;
  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  // field accessors
  WbAppearance *appearance() const;
  WbPbrAppearance *pbrAppearance() const;
  WbAbstractAppearance *abstractAppearance() const;
  WbGeometry *geometry() const { return dynamic_cast<WbGeometry *>(mGeometry->value()); }
  WbSFNode *geometryField() const { return mGeometry; }
  bool isCastShadowsEnabled() const;

  // other accessors
  WrMaterial *wrenMaterial() const { return mWrenMaterial; }

  // infrared related functions
  void pickColor(const WbRay &ray, WbRgb &pickedColor, double *roughness = NULL, double *occlusion = NULL) const;

  // for a shape lying into a boundingObject
  void connectGeometryField() const;
  void disconnectGeometryField() const;

  // ray tracing
  WbBoundingSphere *boundingSphere() const override;

  void setAppearance(WbAppearance *appearance);
  void setPbrAppearance(WbPbrAppearance *appearance);
  void setGeometry(WbGeometry *geometry);
  void updateSegmentationColor(const WbRgb &color) override;

  // export
  bool exportNodeHeader(WbWriter &writer) const override;
  void exportBoundingObjectToX3D(WbWriter &writer) const override;
  QStringList fieldsToSynchronizeWithX3D() const override;

signals:
  void wrenMaterialChanged();
  void castShadowsChanged();
  void geometryInShapeInserted();

private:
  // user accessible fields
  WbSFNode *mAppearance;
  WbSFNode *mGeometry;
  WbSFBool *mCastShadows;
  WbSFBool *mIsPickable;

  WrMaterial *mWrenMaterial;

  WbShape &operator=(const WbShape &);  // non copyable
  WbNode *clone() const override { return new WbShape(*this); }
  void init();
  void applyMaterialToGeometry();
  void createWrenMaterial(int type);

private slots:
  void updateAppearance();
  void updateGeometry();
  void updateGeometryMaterial();
  void updateBoundingSphere(WbBaseNode *subNode);
  void updateCastShadows();
  void updateIsPickable();
  void createOdeGeom();
};

#endif
