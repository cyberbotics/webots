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

#ifndef WB_BASE_NODE_HPP
#define WB_BASE_NODE_HPP

//
// Description: abstract base class for all types of nodes in Webots
//
// Inherited by:
//   Directly or indirectly by every node class in the 'nodes' subproject
//

#include "../../../include/controller/c/webots/nodes.h"
#include "WbNode.hpp"

#include "WbMatrix3.hpp"
#include "WbRgb.hpp"
#include "WbVector3.hpp"

class WbPose;
class WbTransform;
class WbSolid;
class WbBoundingSphere;

struct WrTransform;

class WbBaseNode : public WbNode {
  Q_OBJECT

public:
  // destructor
  virtual ~WbBaseNode();

  virtual void downloadAssets() {}
  // finalize() assumes that the whole world node/field structure is complete
  void finalize();
  virtual void preFinalize() {
    mPreFinalizeCalled = true;
    setCreationCompleted();
  }
  virtual void postFinalize();
  virtual void validateProtoNodes();
  virtual void validateProtoNode() {}
  bool isPreFinalizedCalled() const { return mPreFinalizeCalled; }
  bool isPostFinalizedCalled() const { return mPostFinalizeCalled; }
  void reset(const QString &id) override;

  // for libController
  virtual int nodeType() const = 0;

  // updates material of all WbGeometry descendants lying into a bounding object
  // reimplemented in WbGroup (recurse through all children), WbPose, WbShape and WbGeometry
  virtual void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) {}
  virtual void setSleepMaterial() {}

  // reimplemented in WbGroup (recurse through all children), WbPropeller, WbShape and WbGeometry
  virtual void propagateSelection(bool selected) {}

  // Method used to cache absolute scale values
  // reimplemented in WbGroup (recurse through all children), WbPose, WbSolid, WbShape and WbIndexedFaceSet
  virtual void setScaleNeedUpdate() {}

  // informs all children that their matrices need to be recomputed (inherited from WbGroup)
  // reimplemented in WbGroup (recurse through all children), WbPose, WbSolid, WbPropeller
  virtual void setMatrixNeedUpdate() {}

  // propagate segmentation color change reimplemented in WbGroup (recurse through all children), WbBasicJoint,
  // WbCadShape, WbShape, WbSkin, WbSlot and WbSolid
  virtual void updateSegmentationColor(const WbRgb &color) {}

  // Wren functions
  virtual void createWrenObjects();
  bool areWrenObjectsInitialized() const { return mWrenObjectsCreatedCalled; }
  WrTransform *wrenNode() const { return mWrenNode; }
  void setWrenNode(WrTransform *n) { mWrenNode = n; }

  // return the closest descendant node(s) with dedicated Wren node (may be the node itself)
  // only WbBillboard, WbGeometry, WbPose, and WbMuscle have a dedicated Wren node
  // used to properly apply Wren settings only to the current/descendant nodes and not to parent and sibling nodes
  virtual QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const {
    return QList<const WbBaseNode *>();
  }

  // Ode functions
  virtual void createOdeObjects() { mOdeObjectsCreatedCalled = true; }
  bool areOdeObjectsCreated() const { return mOdeObjectsCreatedCalled; }

  // node as bounding object
  virtual bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const { return false; }
  virtual bool isSuitableForInsertionInBoundingObject(bool warning = false) const { return false; }

  // Cached utility functions for permanent node properties
  bool isInBoundingObject() const;
  WbSolid *upperSolid() const;
  WbSolid *topSolid() const;
  WbPose *upperPose() const;
  WbTransform *upperTransform() const;

  // Cached function that can change if new USE nodes are added
  // return if this node or any of its instances is used in boundingObject
  WbNode::NodeUse nodeUse() const;

  // Ray tracing functions
  virtual WbBoundingSphere *boundingSphere() const { return NULL; }

  // resize/scale manipulator
  virtual bool hasResizeManipulator() const { return false; }
  virtual void attachResizeManipulator() {}
  virtual void detachResizeManipulator() const {}
  virtual void updateResizeHandlesSize() {}
  virtual void setUniformConstraintForResizeHandles(bool enabled) {}

  // only for PROTO instances
  // return the first finalized instance node of a PROTO (multiple finalized instances may exist)
  WbBaseNode *getFirstFinalizedProtoInstance() const;

  QString documentationUrl() const;

signals:
  void isBeingDestroyed(WbBaseNode *node);
  void visibleHandlesChanged(bool resizeHandlesEnabled);
  void finalizationCompleted(WbBaseNode *node);

public slots:
  virtual void showResizeManipulator(bool enabled) {}

protected:
  bool isUrdfRootLink() const override;
  virtual WbVector3 urdfRotation(const WbMatrix3 &rotationMatrix) const { return rotationMatrix.toEulerAnglesZYX(); }

  void exportUrdfJoint(WbWriter &writer) const override;

  // constructor:
  // if the tokenizer is NULL, then the node is constructed with the default field values
  // otherwise the field values are read from the tokenizer
  WbBaseNode(const QString &modelName, WbTokenizer *tokenizer);

  // copy constructor to be invoked from the copy constructors of derived classes
  // copies all the field values
  WbBaseNode(const WbBaseNode &other);
  WbBaseNode(const WbNode &other);

  // constructor for shallow nodes, should be used exclusively by the CadShape node
  WbBaseNode(const QString &modelName);

  void defHasChanged() override { finalize(); }
  void useNodesChanged() const override { mNodeUseDirty = true; };

  // semaphore used to cancel the finalization
  bool mFinalizationCanceled;

  bool isInvisibleNode() const;

  bool exportNodeHeader(WbWriter &writer) const override;

private:
  WbBaseNode &operator=(const WbBaseNode &);  // non copyable
  void init();

  // WREN
  WrTransform *mWrenNode;

  // finalize booleans
  bool mPreFinalizeCalled;
  bool mPostFinalizeCalled;
  bool mWrenObjectsCreatedCalled;
  bool mOdeObjectsCreatedCalled;

  // Bounding object info
  // TODO: remove mutable keywords: this breaks the const functions
  //       possible improvement solutions (to investigate):
  //         -> migrate the search/cache code into WbNodeUtilities
  //         -> migrate the search/cache code into not const functions called when setting the parent
  mutable bool mIsInBoundingObject;
  mutable bool mBoundingObjectFirstTimeSearch;
  mutable WbPose *mUpperPose;
  mutable bool mUpperPoseFirstTimeSearch;
  mutable WbTransform *mUpperTransform;
  mutable bool mUpperTransformFirstTimeSearch;
  mutable WbSolid *mUpperSolid;
  mutable bool mUpperSolidFirstTimeSearch;
  mutable WbSolid *mTopSolid;
  mutable bool mTopSolidFirstTimeSearch;
  mutable WbNode::NodeUse mNodeUse;
  mutable bool mNodeUseDirty;
};

#endif
