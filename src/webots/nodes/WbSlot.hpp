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

#ifndef WB_SLOT_HPP
#define WB_SLOT_HPP

#include "WbBaseNode.hpp"
#include "WbSFNode.hpp"
#include "WbSFString.hpp"

class WbGroup;
class WbSolidReference;

class WbSlot : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbSlot(WbTokenizer *tokenizer = NULL);
  WbSlot(const WbSlot &other);
  explicit WbSlot(const WbNode &other);
  virtual ~WbSlot();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SLOT; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  void validateProtoNode() override;
  void write(WbWriter &writer) const override;
  void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setScaleNeedUpdate() override;
  void attachResizeManipulator() override;
  void detachResizeManipulator() const override;
  void reset(const QString &id) override;
  void save(const QString &id) override;
  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;
  void updateSegmentationColor(const WbRgb &color) override;

  // field accessors
  bool hasEndPoint() const { return mEndPoint->value() != NULL; }
  WbSFNode *endPointField() const { return mEndPoint; }
  WbNode *endPoint() const { return mEndPoint->value(); }
  WbSolid *solidEndPoint() const;
  WbSolidReference *solidReferenceEndPoint() const;
  WbSlot *slotEndPoint() const;
  WbGroup *groupEndPoint() const;
  const QString &slotType() const { return mSlotType->value(); }

  void setEndPoint(WbNode *node);

  // selection
  void propagateSelection(bool selected) override;

  // bounding sphere
  WbBoundingSphere *boundingSphere() const override;

  // lazy matrix multiplication system
  void setMatrixNeedUpdate() override;

  QString endPointName() const override;

signals:
  void endPointInserted(WbBaseNode *);  // called when a node is inserted in the endPoint

private:
  WbSlot &operator=(const WbSlot &);  // non copyable
  WbNode *clone() const override { return new WbSlot(*this); }
  void init();

  // user accessible fields
  WbSFNode *mEndPoint;
  WbSFString *mSlotType;

private slots:
  void endPointChanged();
  void updateType();
};

#endif
