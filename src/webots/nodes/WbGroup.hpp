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

#ifndef WB_GROUP_HPP
#define WB_GROUP_HPP

#include "WbBaseNode.hpp"
#include "WbHiddenKinematicParameters.hpp"
#include "WbMFNode.hpp"
#include "WbOdeTypes.hpp"

class WbBoundingSphere;

class WbGroup : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbGroup(WbTokenizer *tokenizer = NULL);
  WbGroup(const WbGroup &other);
  explicit WbGroup(const WbNode &other);
  virtual ~WbGroup();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_GROUP; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setScaleNeedUpdate() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  bool shallExport() const override;
  void reset(const QString &id) override;
  void save(const QString &id) override;
  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  // field accessors
  int childCount() const { return mChildren->size(); }
  WbBaseNode *child(int index) const;
  int nodeIndex(WbNode *child) const;
  const WbMFNode &children() const { return *mChildren; }
  const WbMFNode *childrenField() const { return mChildren; }

  // append at the end
  void addChild(WbNode *child);

  // insert a child at the specified index
  void insertChild(int index, WbNode *child);

  // set a child at the specified index
  void setChild(int index, WbNode *child);

  // remove all children without deleting them
  void clear();

  // remove and delete all children
  void deleteAllChildren();

  // remove and delete all solid children
  virtual void deleteAllSolids();

  // utility forward functions if the group/transform node has no solid ancestor
  // forward jerk notification to children
  virtual void forwardJerk();
  void writeParameters(WbWriter &writer) const override;
  virtual void collectHiddenKinematicParameters(WbHiddenKinematicParameters::HiddenKinematicParametersMap &map,
                                                int &counter) const;
  virtual bool resetHiddenKinematicParameters();
  virtual bool restoreHiddenKinematicParameters(const WbHiddenKinematicParameters::HiddenKinematicParametersMap &map,
                                                int &counter);
  void readHiddenKinematicParameter(WbField *field) override;

  // selection
  void propagateSelection(bool selected) override;

  // propagate change in segmentation color
  void updateSegmentationColor(const WbRgb &color) override;

  // bounding sphere
  WbBoundingSphere *boundingSphere() const override { return mBoundingSphere; }
  void recomputeBoundingSphere();
  // For a group in a boundingObject
  dSpaceID odeSpace() const { return mOdeSpace; }
  void setOdeData(dSpaceID s) { mOdeSpace = s; }

  // lazy matrix multiplication system
  void setMatrixNeedUpdate() override;

  // export
  void exportBoundingObjectToX3D(WbWriter &writer) const override;

signals:
  // called after the list of children has changed
  void childrenChanged();
  void topLevelListsUpdateRequested();
  void childAdded(WbBaseNode *child);
  void finalizedChildAdded(WbBaseNode *child);  // emit signal when inserting child after current node is finalized
  void notifyParentSlot(WbBaseNode *child);
  void notifyParentJoint(WbBaseNode *child);
  void childFinalizationHasProgressed(const int progress);  // 0: beginning, 100: end
  void worldLoadingStatusHasChanged(QString status);

protected:
  // this constructor is reserved for derived classes only
  WbGroup(const QString &modelName, WbTokenizer *tokenizer);

  // called when a node is inserted in the children of this group
  // or in the children of the children of this group, etc.
  virtual void descendantNodeInserted(WbBaseNode *decendant);

  // utility fields if the group/transform node has no solid ancestor
  bool mHasNoSolidAncestor;
  WbHiddenKinematicParameters::HiddenKinematicParametersMap mHiddenKinematicParametersMap;

  mutable WbBoundingSphere *mBoundingSphere;

private:
  WbGroup &operator=(const WbGroup &);  // non copyable
  WbNode *clone() const override { return new WbGroup(*this); }
  void init();

  // ODE (for Group lying into a boundingObject)
  dSpaceID mOdeSpace;

  // user accessible fields
  WbMFNode *mChildren;
  int mLoadProgress;

public slots:
  void cancelFinalization();
  void insertChildFromSlotOrJoint(WbBaseNode *decendant);

private slots:
  void insertChildPrivate(int index);
  void monitorChildFinalization(WbBaseNode *child);
};

#endif
