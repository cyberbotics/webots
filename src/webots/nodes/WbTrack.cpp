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

#include "WbTrack.hpp"

#include "WbAppearance.hpp"
#include "WbBrake.hpp"
#include "WbDictionary.hpp"
#include "WbField.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbLinearMotor.hpp"
#include "WbMFNode.hpp"
#include "WbMathsUtilities.hpp"
#include "WbNodeOperations.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPositionSensor.hpp"
#include "WbPrecision.hpp"
#include "WbRobot.hpp"
#include "WbSFInt.hpp"
#include "WbSFVector2.hpp"
#include "WbSlot.hpp"
#include "WbTextureTransform.hpp"
#include "WbTrackWheel.hpp"
#include "WbVrmlNodeUtilities.hpp"
#include "WbWrenVertexArrayFrameListener.hpp"

#include <ode/ode.h>
#include <cassert>

#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/transform.h>

void WbTrack::init() {
  mDeviceField = findMFNode("device");
  mTextureAnimationField = findSFVector2("textureAnimation");
  mGeometryField = findSFNode("animatedGeometry");
  mGeometriesCountField = findSFInt("geometriesCount");

  mSurfaceVelocity = 0.0;
  mMotorPosition = 0.0;
  mLinearMotor = NULL;
  mBrake = NULL;
  mBodyID = NULL;

  // texture animation
  mShape = NULL;
  mTextureTransform = NULL;
  mSavedTextureTransformTranslations[stateId()] = WbVector2();

  // geometries animation
  mPathLength = 0.0;
  mPathStepSize = 0.0;
  mAnimationStepSize = 0;
}

WbTrack::WbTrack(WbTokenizer *tokenizer) : WbSolid("Track", tokenizer) {
  init();
}

WbTrack::WbTrack(const WbTrack &other) : WbSolid(other) {
  init();
}

WbTrack::WbTrack(const WbNode &other) : WbSolid(other) {
  init();
}

WbTrack::~WbTrack() {
  WbWrenVertexArrayFrameListener::instance()->unsubscribeTrack(this);
  clearWheelsList();
  clearAnimatedGeometries();
}

void WbTrack::preFinalize() {
  WbSolid::preFinalize();

  updateDevices();

  WbMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    WbLogicalDevice *device = dynamic_cast<WbLogicalDevice *>(it.next());
    device->preFinalize();
  }

  WbBaseNode *node = dynamic_cast<WbBaseNode *>(mGeometryField->value());
  if (node)
    node->preFinalize();
}

void WbTrack::createWrenObjects() {
  WbSolid::createWrenObjects();

  QList<WbShape *> shapeNodes;
  findAndConnectAnimatedGeometries(false, &shapeNodes);

  WbNode *node = mGeometryField->value();
  if (node != NULL)
    static_cast<WbBaseNode *>(node)->createWrenObjects();
}

void WbTrack::postFinalize() {
  WbSolid::postFinalize();

  // get bodyID
  WbSolid *node = this;
  while (node) {
    mBodyID = node->body();
    if (mBodyID)
      break;
    node = WbNodeUtilities::findUpperSolid(node);
  }

  WbMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    WbLogicalDevice *device = dynamic_cast<WbLogicalDevice *>(it.next());
    device->postFinalize();
  }
  connect(mDeviceField, &WbMFNode::changed, this, &WbTrack::updateDevices);
  connect(mDeviceField, &WbMFNode::itemChanged, this, &WbTrack::addDevice);
  connect(mDeviceField, &WbMFNode::itemInserted, this, &WbTrack::addDevice);

  if (childCount() > 0) {
    WbGroup *group = dynamic_cast<WbGroup *>(child(0));
    if (group)
      connect(group, &WbGroup::childrenChanged, this, &WbTrack::updateChildren);
    if (mShape)
      connect(mShape, &WbShape::wrenMaterialChanged, this, &WbTrack::updateTextureTransform, Qt::UniqueConnection);
  }
  connect(this, &WbGroup::childrenChanged, this, &WbTrack::updateChildren);

  // animated geometries
  WbBaseNode *geometry = dynamic_cast<WbBaseNode *>(mGeometryField->value());
  if (geometry)
    geometry->postFinalize();
  updateAnimatedGeometriesPath();

  connect(mGeometryField, &WbSFNode::changed, this, &WbTrack::updateAnimatedGeometries);
  connect(mGeometriesCountField, &WbSFInt::changed, this, &WbTrack::updateAnimatedGeometries);

  for (int i = 0; i < mWheelsList.size(); ++i)
    connect(mWheelsList[i], &WbTrackWheel::changed, this, &WbTrack::updateAnimatedGeometriesPath, Qt::UniqueConnection);

  connect(mTextureAnimationField, &WbSFVector2::changed, this, &WbTrack::updateTextureAnimation);
}

void WbTrack::setMatrixNeedUpdate() {
  WbSolid::setMatrixNeedUpdate();

  WbNode *node = mGeometryField->value();
  if (node != NULL)
    static_cast<WbBaseNode *>(node)->setMatrixNeedUpdate();
}

void WbTrack::addDevice(int index) {
  WbRobot *const r = robot();
  assert(r);
  WbBaseNode *decendant = dynamic_cast<WbBaseNode *>(mDeviceField->item(index));
  r->descendantNodeInserted(decendant);
}

void WbTrack::reset(const QString &id) {
  WbSolid::reset(id);

  WbNode *const g = mGeometryField->value();
  if (g)
    g->reset(id);
  for (int i = 0; i < mDeviceField->size(); ++i)
    mDeviceField->item(i)->reset(id);

  mMotorPosition = 0.0;
  mSurfaceVelocity = 0.0;
  if (mTextureTransform)
    mTextureTransform->setTranslation(mSavedTextureTransformTranslations[id]);
}

void WbTrack::save(const QString &id) {
  WbSolid::save(id);

  WbNode *const g = mGeometryField->value();
  if (g)
    g->save(id);
  for (int i = 0; i < mDeviceField->size(); ++i)
    mDeviceField->item(i)->save(id);

  mSavedTextureTransformTranslations[id] = WbVector2();
  if (mShape && mShape->abstractAppearance()) {
    mTextureTransform = mShape->abstractAppearance()->textureTransform();
    if (mTextureTransform)
      mSavedTextureTransformTranslations[id] = mTextureTransform->translation();
  }
}

void WbTrack::updateDevices() {
  mLinearMotor = NULL;
  mBrake = NULL;
  WbMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    WbNode *node = it.next();
    if (!mLinearMotor) {
      mLinearMotor = dynamic_cast<WbLinearMotor *>(node);
      if (mLinearMotor)
        continue;
    }
    if (!mBrake)
      mBrake = dynamic_cast<WbBrake *>(node);
  }
}

bool WbTrack::findAndConnectAnimatedGeometries(bool connectSignals, QList<WbShape *> *shapeList) {
  WbBaseNode *geometry = dynamic_cast<WbBaseNode *>(mGeometryField->value());
  if (!geometry)
    return false;

  QList<WbBaseNode *> geometryNodes;
  geometryNodes.append(geometry);
  WbBaseNode *node = NULL;
  for (int i = 0; i < geometryNodes.size(); ++i) {
    node = geometryNodes[i];
    if (connectSignals && !node->isPostFinalizedCalled()) {
      connect(node, &WbBaseNode::finalizationCompleted, this, &WbTrack::updateAnimatedGeometriesAfterFinalization,
              Qt::UniqueConnection);
      return false;
    }

    WbShape *s = dynamic_cast<WbShape *>(node);
    if (s) {
      if (connectSignals) {
        // material automatically updated
        connect(s->geometryField(), &WbSFNode::changed, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);
        connect(s, &WbShape::castShadowsChanged, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);
        if (s->geometry())
          connect(s->geometry(), &WbGeometry::changed, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      }
      if (shapeList != NULL)
        shapeList->append(s);
      continue;
    }

    WbGroup *g = dynamic_cast<WbGroup *>(node);
    if (g) {
      // group, pose or transform nodes
      if (connectSignals) {
        connect(g, &WbGroup::finalizedChildAdded, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);
        connect(g->childrenField(), &WbMFNode::itemRemoved, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      }
      for (int j = 0; j < g->childCount(); ++j)
        geometryNodes.append(g->child(j));

      WbPose *t = dynamic_cast<WbPose *>(g);
      if (t) {
        t->enablePoseChangedSignal();
        connect(t, &WbPose::poseChanged, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      }

      continue;
    }

    WbSlot *slot = dynamic_cast<WbSlot *>(node);
    if (slot) {
      WbSlot *slot2 = slot->slotEndPoint();
      if (slot2) {
        WbBaseNode *endPoint = dynamic_cast<WbBaseNode *>(slot2->endPoint());
        if (endPoint)
          geometryNodes.append(endPoint);
        connect(slot2->endPointField(), &WbSFNode::changed, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      }
      connect(slot->endPointField(), &WbSFNode::changed, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      continue;
    }

    // ignore invalid node
    parsingWarn(tr("Invalid %1 used in 'animatedGeometries' field.").arg(node->nodeModelName()));
  }

  return true;
}

void WbTrack::updateChildren() {
  // update texture animation
  if (childCount() == 0) {
    mShape = NULL;
    mTextureTransform = NULL;
  } else
    updateShapeNode();

  // update geometry animation
  updateWheelsList();
}

void WbTrack::updateShapeNode() {
  mShape = NULL;

  WbBaseNode *firstChild = child(0);
  mShape = dynamic_cast<WbShape *>(firstChild);
  if (!mShape) {
    WbGroup *group = dynamic_cast<WbGroup *>(firstChild);
    if (group && group->children().size() > 0)
      mShape = dynamic_cast<WbShape *>(group->child(0));
  }
  updateTextureTransform();
}

void WbTrack::updateTextureTransform() {
  mTextureTransform = NULL;

  if (mShape && mShape->abstractAppearance()) {
    mTextureTransform = mShape->abstractAppearance()->textureTransform();
    if (mTextureTransform) {
      mSavedTextureTransformTranslations[stateId()] = mTextureTransform->translation();
      QList<WbNode *> useNodesList = WbVrmlNodeUtilities::findUseNodeAncestors(mTextureTransform);
      if (!useNodesList.isEmpty()) {
        mTextureTransform->parsingWarn(tr("Non-admissible TextureTransform USE node inside Track node."
                                          "This and ancestor USE nodes turned into DEF nodes: if texture animation enabled, "
                                          "the USE texture transform values will change independently from DEF node ones."));
        const int size = useNodesList.size();
        if (size > 0) {
          for (int i = 0; i < size; ++i)
            useNodesList[i]->makeDefNode();
          WbNodeOperations::instance()->updateDictionary(false, NULL);
        }
      }
    } else if (!mTextureAnimationField->value().isNull())
      mShape->abstractAppearance()->parsingWarn(
        tr("Texture animation is enabled only if the TextureTransform node is explicitly defined."));
    if (isPostFinalizedCalled())
      connect(mShape->abstractAppearance(), &WbAppearance::changed, this, &WbTrack::updateTextureTransform,
              Qt::UniqueConnection);
  }
}

void WbTrack::updateTextureAnimation() {
  if (!mTextureTransform && !mTextureAnimationField->value().isNull())
    parsingWarn(tr("Texture animation is enabled only if the TextureTransform node is explicitly defined."));
}

void WbTrack::updateWheelsList() {
  clearWheelsList();

  WbMFNode::Iterator it(*childrenField());
  WbTrackWheel *wheel = NULL;
  while (it.hasNext()) {
    wheel = dynamic_cast<WbTrackWheel *>(it.next());
    if (wheel) {
      mWheelsList.append(wheel);
      if (isPostFinalizedCalled())
        connect(wheel, &WbTrackWheel::changed, this, &WbTrack::updateAnimatedGeometriesPath, Qt::UniqueConnection);
    }
  }

  if (mWheelsList.isEmpty()) {
    mPathLength = 0.0;
    mPathList.clear();
    clearAnimatedGeometries();
  } else
    updateAnimatedGeometriesPath();
}

void WbTrack::clearWheelsList() {
  if (mWheelsList.isEmpty())
    return;

  for (int i = 0; i < mWheelsList.size(); ++i)
    disconnect(mWheelsList[i], &WbTrackWheel::changed, this, &WbTrack::updateAnimatedGeometriesPath);
  mWheelsList.clear();
}

QVector<WbLogicalDevice *> WbTrack::devices() const {
  QVector<WbLogicalDevice *> devices;
  WbLogicalDevice *device = NULL;
  WbMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    device = dynamic_cast<WbLogicalDevice *>(it.next());
    if (device)
      devices.append(device);
  }
  return devices;
}

WbPositionSensor *WbTrack::positionSensor() const {
  WbPositionSensor *s = NULL;
  WbMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    s = dynamic_cast<WbPositionSensor *>(it.next());
    if (s)
      return s;
  }
  return NULL;
}

WbLinearMotor *WbTrack::motor() const {
  if (isPreFinalizedCalled())
    return mLinearMotor;

  WbMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    WbLinearMotor *m = dynamic_cast<WbLinearMotor *>(it.next());
    if (m)
      return m;
  }
  return NULL;
}

WbBrake *WbTrack::brake() const {
  if (isPreFinalizedCalled())
    return mBrake;

  WbMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    WbBrake *b = dynamic_cast<WbBrake *>(it.next());
    if (b)
      return b;
  }
  return NULL;
}

void WbTrack::updateAnimatedGeometriesPath() {
  if (!isPostFinalizedCalled())
    return;

  mPathLength = 0.0;
  mPathList.clear();

  if (mWheelsList.isEmpty()) {
    clearAnimatedGeometries();
    return;
  }

  computeBeltPath();

  if (mGeometriesCountField->value() > 0 && mAnimatedObjectList.isEmpty())
    updateAnimatedGeometries();
  else {
    initAnimatedGeometriesBeltPosition();
    WbWrenVertexArrayFrameListener::instance()->subscribeTrack(this);
  }
}

void WbTrack::initAnimatedGeometriesBeltPosition() {
  const int numGeometries = mGeometriesCountField->value();
  mPathStepSize = mPathLength / numGeometries;
  BeltPosition beltPosition(mPathList[0].startPoint, mPathList[0].initialRotation, 0);
  mFirstGeometryPosition = beltPosition;
}

void WbTrack::updateAnimatedGeometriesAfterFinalization(WbBaseNode *node) {
  disconnect(node, &WbBaseNode::finalizationCompleted, this, &WbTrack::updateAnimatedGeometriesAfterFinalization);
  updateAnimatedGeometries();
}

void WbTrack::updateAnimatedGeometries() {
  clearAnimatedGeometries();

  if (mWheelsList.isEmpty())
    return;

  int numGeometries = mGeometriesCountField->value();
  WbBaseNode *geometry = dynamic_cast<WbBaseNode *>(mGeometryField->value());
  if (numGeometries <= 0 || !geometry)
    return;

  QList<WbShape *> shapeNodes;
  bool success = findAndConnectAnimatedGeometries(isPostFinalizedCalled(), &shapeNodes);
  if (!success)
    return;

  for (int i = 0; i < shapeNodes.size(); ++i) {
    WbGeometry *geom = shapeNodes[i]->geometry();
    if (geom == NULL)
      continue;

    WbIndexedFaceSet *ifs = dynamic_cast<WbIndexedFaceSet *>(geom);
    // cppcheck-suppress knownConditionTrueFalse
    if (ifs)
      ifs->updateTriangleMesh();

    if (!geom->isPostFinalizedCalled())
      connect(geom, &WbBaseNode::finalizationCompleted, this, &WbTrack::updateAnimatedGeometriesAfterFinalization,
              Qt::UniqueConnection);
    else {
      WrNode *wrenNode = WR_NODE(geom->wrenNode());
      assert(wrenNode);  // wren objects have to be already initialized during node finalization

      AnimatedObject *object = new AnimatedObject();
      object->geometry = geom;
      object->material = shapeNodes[i]->wrenMaterial();
      object->castShadows = shapeNodes[i]->isCastShadowsEnabled();
      connect(shapeNodes[i], &WbShape::wrenMaterialChanged, this, &WbTrack::updateAnimatedGeometries, Qt::UniqueConnection);

      // Hide WREN node (visible by default)
      wr_node_set_visible(wrenNode, false);
      mAnimatedObjectList.append(object);
    }
  }

  initAnimatedGeometriesBeltPosition();

  double stepSize = 0;
  mBeltPositions.reserve(numGeometries);
  BeltPosition beltPosition = mFirstGeometryPosition;

  for (int i = 0; i < numGeometries; ++i) {
    beltPosition = computeNextGeometryPosition(beltPosition, stepSize);
    mBeltPositions.append(beltPosition);
    if (beltPosition.segmentIndex < 0) {
      // abort
      clearAnimatedGeometries();
      return;
    }
    float p[3];
    float r[4];
    WbVector3(beltPosition.position.x(), 0.0, beltPosition.position.y()).toFloatArray(p);
    WbRotation(0.0, 1.0, 0.0, beltPosition.rotation).toFloatArray(r);

    WrTransform *transform = wr_transform_new();
    wr_transform_set_position(transform, p);
    wr_transform_set_orientation(transform, r);

    for (int j = 0; j < mAnimatedObjectList.size(); ++j) {
      WbGeometry *geom = mAnimatedObjectList[j]->geometry;

      WbMatrix4 geomMatrix = geom->matrix() * matrix().pseudoInversed();

      geomMatrix.translation().toFloatArray(p);
      WbRotation(geomMatrix.extracted3x3Matrix()).toFloatArray(r);

      WrTransform *meshTransform = wr_transform_new();
      wr_transform_set_position(meshTransform, p);
      wr_transform_set_orientation(meshTransform, r);

      WrRenderable *renderable = wr_renderable_new();
      wr_renderable_set_material(renderable, mAnimatedObjectList[j]->material, NULL);
      wr_renderable_set_mesh(renderable, WR_MESH(geom->wrenMesh()));
      wr_renderable_set_cast_shadows(renderable, mAnimatedObjectList[j]->castShadows);

      WrTransform *geomTransform = wr_transform_copy(geom->wrenNode());
      wr_transform_attach_child(geomTransform, WR_NODE(renderable));
      wr_transform_attach_child(meshTransform, WR_NODE(geomTransform));
      wr_transform_attach_child(transform, WR_NODE(meshTransform));
      wr_node_set_visible(WR_NODE(geomTransform), true);

      // Keep reference to nodes for deletion
      mWrenNodes.append(WR_NODE(renderable));
      mWrenNodes.append(WR_NODE(meshTransform));
    }

    if (i == 0)
      stepSize = mPathStepSize;

    wr_transform_attach_child(wrenNode(), WR_NODE(transform));
    mBeltElements.append(transform);
  }

  WbWrenVertexArrayFrameListener::instance()->subscribeTrack(this);
}

void WbTrack::clearAnimatedGeometries() {
  if (!mAnimatedObjectList.isEmpty())
    WbWrenVertexArrayFrameListener::instance()->unsubscribeTrack(this);

  for (WrNode *node : mWrenNodes)
    wr_node_delete(node);

  for (WrTransform *transform : mBeltElements)
    wr_node_delete(WR_NODE(transform));

  mWrenNodes.clear();
  mBeltElements.clear();

  for (int i = 0; i < mAnimatedObjectList.size(); ++i)
    delete mAnimatedObjectList[i];

  mAnimatedObjectList.clear();
}

void WbTrack::prePhysicsStep(double ms) {
  WbSolid::prePhysicsStep(ms);

  const double sec = ms * 0.001;
  if (mLinearMotor && mBodyID) {
    if (mLinearMotor->userControl()) {
      // force control
      dMass mass;
      dBodyGetMass(mBodyID, &mass);
      double force = mLinearMotor->rawInput();
      if (mBrake)
        force -= mBrake->getBrakingDampingConstant() * mSurfaceVelocity;
      mSurfaceVelocity = force * sec / mass.mass;
    } else
      // position or velocity control
      mSurfaceVelocity = -mLinearMotor->computeCurrentDynamicVelocity(ms, mMotorPosition);
  } else
    mSurfaceVelocity = 0.0;

  const double travelledDistance = mSurfaceVelocity * sec;
  mMotorPosition += travelledDistance;

  for (int i = 0; i < mWheelsList.size(); ++i)
    mWheelsList[i]->rotate(travelledDistance);

  // texture animation
  if (mTextureTransform) {
    mTextureTransform->translate(-0.001 * ms * mSurfaceVelocity * mTextureAnimationField->value());
    mTextureTransform->modifyWrenMaterial(mShape->wrenMaterial());
  }

  // geometries animation
  if (!mAnimatedObjectList.isEmpty()) {
    mAnimationStepSize += travelledDistance;
    while (mAnimationStepSize > mPathStepSize)
      mAnimationStepSize -= mPathStepSize;
    while (mAnimationStepSize < -mPathStepSize)
      mAnimationStepSize += mPathStepSize;
    WbWrenVertexArrayFrameListener::instance()->subscribeTrack(this);
  }
}

void WbTrack::animateMesh() {
  if (mAnimatedObjectList.isEmpty())
    return;

  double stepSize = mAnimationStepSize;
  mAnimationStepSize = 0;

  BeltPosition beltPosition = mFirstGeometryPosition;
  for (int i = 0; i < mGeometriesCountField->value(); ++i) {
    beltPosition = computeNextGeometryPosition(beltPosition, stepSize);
    mBeltPositions[i] = beltPosition;
    if (beltPosition.segmentIndex < 0) {
      // abort
      clearAnimatedGeometries();
      return;
    }

    float p[3];
    float r[4];
    WbVector3(beltPosition.position.x(), 0.0, beltPosition.position.y()).toFloatArray(p);
    WbRotation(0.0, 1.0, 0.0, beltPosition.rotation).toFloatArray(r);

    wr_transform_set_position(mBeltElements[i], p);
    wr_transform_set_orientation(mBeltElements[i], r);

    if (i == 0) {
      mFirstGeometryPosition = beltPosition;
      stepSize = mPathStepSize;
    }
  }
}

WbTrack::BeltPosition WbTrack::computeNextGeometryPosition(WbTrack::BeltPosition current, double stepSize,
                                                           bool segmentChanged) const {
  if (stepSize == 0)
    return current;

  const bool isPositiveStep = stepSize >= 0;
  const bool singleWheelCase = mWheelsList.size() == 1;
  const PathSegment segment = mPathList[current.segmentIndex];
  const WbVector2 endPoint = stepSize < 0 ? segment.startPoint : segment.endPoint;
  const WbVector2 maxDistanceVector = endPoint - current.position;
  double newStepSize = stepSize;

  if (singleWheelCase || (!maxDistanceVector.isNull() && maxDistanceVector.length() > 1e-10)) {
    double maxStepSize = 0.0;
    if (segment.radius < 0) {
      // straight
      maxStepSize = maxDistanceVector.length();
      if (singleWheelCase || fabs(stepSize) <= maxStepSize) {
        WbVector2 nextPosition = current.position + segment.increment * stepSize;
        return BeltPosition(nextPosition, segment.initialRotation, current.segmentIndex);
      }
    } else {
      // round
      WbVector2 relativePosition = current.position - segment.center;
      if (singleWheelCase)
        maxStepSize = fabs(stepSize);
      else {
        WbVector2 relativeEndPosition = endPoint - segment.center;
        WbVector2 c1, c2;
        if (isPositiveStep * segment.increment[0] > 0) {
          c1 = relativePosition;
          c2 = relativeEndPosition;
        } else {
          c1 = relativeEndPosition;
          c2 = relativePosition;
        }
        double angle = atan2(c2.x() * c1.y() - c2.y() * c1.x(), c2.x() * c1.x() + c2.y() * c1.y());
        if (angle < 0)
          angle += 2 * M_PI;

        maxStepSize = angle * segment.radius;
        if (maxStepSize < 0)
          maxStepSize += 2 * M_PI;
      }

      if (fabs(stepSize) <= maxStepSize) {
        const double angle = -(stepSize * segment.increment[0]) / segment.radius;
        const double newPositionX = relativePosition.x() * cos(angle) - relativePosition.y() * sin(angle);
        const double newPositionY = relativePosition.y() * cos(angle) + relativePosition.x() * sin(angle);
        const WbVector2 nextPosition = segment.center + WbVector2(newPositionX, newPositionY);
        double rotation = current.rotation - angle;
        if (segmentChanged) {
          if (!isPositiveStep) {
            int previousStep = current.segmentIndex + 1;
            if (previousStep == mPathList.size())
              previousStep = 0;
            rotation = mPathList[previousStep].initialRotation;
          } else
            rotation = segment.initialRotation;
          rotation -= angle;
        }
        return BeltPosition(nextPosition, rotation, current.segmentIndex);
      }
    }

    current.position = endPoint;
    current.rotation = 0.0;
    if (newStepSize < 0)
      newStepSize += maxStepSize;
    else
      newStepSize -= maxStepSize;
  }

  if (newStepSize != newStepSize) {  // NAN
    // abort generation
    parsingWarn(tr("Error during computation of Track animated geometries. "
                   "Please check the TrackWheel nodes in 'children' field."));
    return BeltPosition(WbVector2(), 0.0, -1);
  }

  int nextSegmentIndex = current.segmentIndex;
  if (isPositiveStep) {
    ++nextSegmentIndex;
    if (nextSegmentIndex == mPathList.size())
      nextSegmentIndex = 0;
  } else {
    --nextSegmentIndex;
    if (nextSegmentIndex == -1)
      nextSegmentIndex = mPathList.size() - 1;
  }
  current.segmentIndex = nextSegmentIndex;
  return computeNextGeometryPosition(current, newStepSize, true);
}

void WbTrack::computeBeltPath() {
  mPathLength = 0.0;

  int wheelsCount = mWheelsList.size();
  if (wheelsCount <= 0)
    return;

  WbVector2 center(mWheelsList[0]->position());
  double radius = mWheelsList[0]->radius();
  if (wheelsCount == 1) {
    // round path
    WbVector2 startPoint(center.x(), center.y() + radius);
    mPathList.append(PathSegment(startPoint, startPoint, 0.0, radius, center, WbVector2(1, 1)));
    mPathLength = 2 * M_PI * radius;
    return;
  }

  WbVector2 firstPoint, previousPoint, distanceVector;
  WbVector2 pointA, pointB;
  double previousRotation = 0.0;
  int nextIndex = 1;

  bool wheelsPositionError = false;
  for (int w = 0; w < wheelsCount; ++w) {
    if (w == wheelsCount - 1)
      nextIndex = 0;

    WbVector2 nextCenter = mWheelsList[nextIndex]->position();
    double nextRadius = mWheelsList[nextIndex]->radius();
    distanceVector = nextCenter - center;
    if (!wheelsPositionError && distanceVector.length() < 0.0000001) {
      wheelsPositionError = true;
      continue;
    }
    double wheelsAngle = atan2(distanceVector.y(), distanceVector.x());
    double absAngle = 0.0;
    bool isWheelInner = mWheelsList[w]->inner();
    bool isOuterTangent = isWheelInner == mWheelsList[nextIndex]->inner();
    if (isOuterTangent) {
      // outer tangent
      double relAngle = WbMathsUtilities::clampedAcos((radius - nextRadius) / distanceVector.length());
      assert(!std::isnan(relAngle));
      if (isWheelInner == 0)
        relAngle = -relAngle;
      absAngle = relAngle + wheelsAngle;
      pointA = WbVector2(cos(absAngle), sin(absAngle)) * radius + center;
      pointB = WbVector2(cos(absAngle), sin(absAngle)) * nextRadius + nextCenter;
    } else {
      // inner tangent
      double relAngle = WbMathsUtilities::clampedAcos((radius + nextRadius) / distanceVector.length());
      assert(!std::isnan(relAngle));
      if (isWheelInner == 0)
        relAngle = -relAngle;
      absAngle = relAngle + wheelsAngle;
      pointA.setXy(radius * cos(absAngle) + center.x(), radius * sin(absAngle) + center.y());
      pointB.setXy(nextRadius * cos(absAngle + M_PI) + nextCenter.x(), nextRadius * sin(absAngle + M_PI) + nextCenter.y());
    }

    if (w == 0)
      firstPoint = pointA;
    else {
      WbVector2 c1, c2, inc;
      if (isWheelInner == 1) {
        c1 = previousPoint - center;
        c2 = pointA - center;
        inc.setXy(1, 1);
      } else {
        c2 = previousPoint - center;
        c1 = pointA - center;
        inc.setXy(-1, -1);
      }
      double angle = atan2(c2.x() * c1.y() - c2.y() * c1.x(), c2.x() * c1.x() + c2.y() * c1.y());
      if (angle < 0)
        angle += 2 * M_PI;
      mPathLength += radius * angle;
      // round path
      mPathList.append(PathSegment(previousPoint, pointA, previousRotation, radius, center, inc));
    }

    // straight path
    distanceVector = pointB - pointA;
    mPathLength += distanceVector.length();
    if (fabs(distanceVector.y()) < 1e-10 && fabs(distanceVector.x()) < 1e-10) {
      WbVector2 centerDistanceVector = nextCenter - center;
      previousRotation = -atan2(centerDistanceVector.y(), centerDistanceVector.x()) - M_PI_2;
    } else
      previousRotation = -atan2(distanceVector.y(), distanceVector.x());
    mPathList.append(PathSegment(pointA, pointB, previousRotation, distanceVector.normalized()));

    previousPoint = pointB;
    center = nextCenter;
    radius = nextRadius;
    ++nextIndex;
  }

  // add last round path
  radius = mWheelsList[0]->radius();
  WbVector2 firstWheelCenter(mWheelsList[0]->position());
  WbVector2 c1(previousPoint - firstWheelCenter);
  WbVector2 c2(firstPoint - firstWheelCenter);
  double angle = atan2(c2.x() * c1.y() - c2.y() * c1.x(), c2.x() * c1.x() + c2.y() * c1.y());
  if (angle < 0)
    angle += 2 * M_PI;
  mPathLength += radius * angle;
  mPathList.append(PathSegment(previousPoint, firstPoint, previousRotation, radius, firstWheelCenter, WbVector2(1, 1)));

  if (wheelsPositionError)
    // multiple wheels at the same location
    parsingWarn(tr("Two or more consecutive TrackWheel nodes are located at the same position. "
                   "Only the first node is used."));
}

void WbTrack::exportAnimatedGeometriesMesh(WbWriter &writer) const {
  if (mAnimatedObjectList.size() == 0 || writer.isUrdf())
    return;

  WbNode *node = mGeometryField->value();

  QString positionString =
    QString("%1").arg(WbPrecision::doubleToString(mBeltPositions[0].position.x(), WbPrecision::DOUBLE_MAX)) + " 0 " +
    QString("%1").arg(WbPrecision::doubleToString(mBeltPositions[0].position.y(), WbPrecision::DOUBLE_MAX));
  QString rotationString =
    QString("0 1 0 %1").arg(WbPrecision::doubleToString(mBeltPositions[0].rotation, WbPrecision::DOUBLE_MAX));

  if (writer.isX3d())
    writer << "<Pose role='animatedGeometry'>";
  else {
    writer.indent();
    writer << "Transform {\n";
    writer.increaseIndent();
    writer.indent();
    writer << "translation " << positionString << "\n";
    writer.indent();
    writer << "rotation " << rotationString << "\n";
    writer.indent();
    writer << "children [\n";
    writer.increaseIndent();
  }

  writer.indent();
  node->write(writer);

  if (writer.isX3d())
    writer << "</Pose>";
  else {
    writer.indent();
    writer << "]\n";
    writer.decreaseIndent();
    writer.indent();
    writer << "}\n";
  }
}

void WbTrack::exportNodeSubNodes(WbWriter &writer) const {
  if (writer.isWebots()) {
    WbSolid::exportNodeSubNodes(writer);
    return;
  }

  foreach (WbField *field, fields()) {
    if (!field->isDeprecated() && (field->isVrml() && field->singleType() == WB_SF_NODE)) {
      const WbSFNode *const node = dynamic_cast<WbSFNode *>(field->value());
      if (node == NULL || node->value() == NULL || node->value()->shallExport()) {
        if (field->name() == "children")
          // export it manually in order to include animated geometries
          continue;

        if (writer.isX3d())
          field->value()->write(writer);
        else
          field->write(writer);
      }
    }
  }

  bool isEmpty = true;
  if (!writer.isX3d() && !writer.isUrdf()) {
    writer.indent();
    writer << "children [";
    writer.increaseIndent();
  }

  // write children nodes
  WbBaseNode *subNode = NULL;
  for (int i = 0; i < childCount(); ++i) {
    subNode = child(i);
    if (subNode->shallExport()) {
      writer.writeMFSeparator(!isEmpty, false);
      subNode->write(writer);
      isEmpty = false;
    }
  }

  // write animated geometries
  if (!writer.isX3d() && !writer.isUrdf() && !isEmpty)
    writer << "\n";
  isEmpty |= mAnimatedObjectList.isEmpty();

  exportAnimatedGeometriesMesh(writer);

  if (!writer.isX3d() && !writer.isUrdf()) {
    writer.decreaseIndent();
    if (!isEmpty)
      writer.indent();
    writer << "]\n";
  }
}

void WbTrack::exportNodeFields(WbWriter &writer) const {
  WbMatter::exportNodeFields(writer);
  if (writer.isX3d()) {
    if (!name().isEmpty())
      writer << " name='" << sanitizedName() << "'";
    writer << " type='track'";
    writer << " geometriesCount='" << mGeometriesCountField->value() << "'";
  }
}
