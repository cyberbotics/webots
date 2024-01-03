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

#include "WbObjectDetection.hpp"

#include "WbAffinePlane.hpp"
#include "WbBoundingSphere.hpp"
#include "WbBox.hpp"
#include "WbCapsule.hpp"
#include "WbCoordinate.hpp"
#include "WbCylinder.hpp"
#include "WbElevationGrid.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbSolid.hpp"
#include "WbSphere.hpp"

WbObjectDetection::WbObjectDetection(WbSolid *device, WbSolid *object, const int occlusion, const double maxRange,
                                     const double horizontalFieldOfView) :
  mDevice(device),
  mObject(object),
  mObjectRelativePosition(0.0, 0.0, 0.0),
  mObjectSize(0.0, 0.0, 0.0),
  mUseBoundingSphereOnly(true),
  mMaxRange(maxRange),
  mOdeGeomData(NULL),
  mHorizontalFieldOfView(horizontalFieldOfView),
  mIsOmniDirectional(mHorizontalFieldOfView > M_PI),
  mOcclusion(occlusion) {
  if (mOcclusion == ONE_RAY) {
    const WbVector3 devicePosition = mDevice->position();
    const WbVector3 direction = object->position() - devicePosition;
    createRays(devicePosition, QList<WbVector3>() << WbVector3(), direction);
  }
  assert(mOcclusion == NONE || mOcclusion == ONE_RAY || mOcclusion == MULTIPLE_RAYS);
};

WbObjectDetection::~WbObjectDetection() {
  deleteRays();
}

bool WbObjectDetection::hasCollided() const {
  assert(!mRaysCollisionDepth.isEmpty());
  for (int i = 0; i < mRaysCollisionDepth.size(); i++) {
    if (mRaysCollisionDepth[i] <= 0.5 * mObjectSize[int(0.5 * i)])
      return false;
  }
  return true;
}

void WbObjectDetection::createRays(const WbVector3 &origin, const QList<WbVector3> &directions, const WbVector3 &offset) {
  if (!mOdeGeomData)
    mOdeGeomData = new WbOdeGeomData(mDevice);

  QListIterator<WbVector3> it(directions);
  while (it.hasNext()) {
    const WbVector3 dir = it.next() + offset;
    const dGeomID geom = dCreateRay(WbOdeContext::instance()->space(), dir.length());
    dGeomSetDynamicFlag(geom);
    dGeomRaySet(geom, origin.x(), origin.y(), origin.z(), dir.x(), dir.y(), dir.z());
    dGeomRaySetLength(geom, dir.length());

    // set device as callback data in case there is a collision
    dGeomSetData(geom, mOdeGeomData);
    mRayGeoms << geom;
    mRaysCollisionDepth << 0.0;
  }
}

void WbObjectDetection::deleteRays() {
  delete mOdeGeomData;
  mOdeGeomData = NULL;
  for (int i = 0; i < mRayGeoms.size(); i++)
    dGeomDestroy(mRayGeoms[i]);
  mRayGeoms.clear();
  mRaysCollisionDepth.clear();
}

void WbObjectDetection::setCollided(const dGeomID rayGeom, const double depth) {
  const int index = mOcclusion == ONE_RAY ? 0 : mRayGeoms.indexOf(rayGeom);
  assert(index >= 0);
  const double d = dGeomRayGetLength(rayGeom) - depth;
  if (mRaysCollisionDepth[index] < d)
    mRaysCollisionDepth[index] = d;
}

void WbObjectDetection::updateRayDirection() {
  const WbVector3 &devicePosition = mDevice->position();
  const WbVector3 offset = mObject->position() - devicePosition;
  QList<WbVector3> directions;
  if (mOcclusion == ONE_RAY)
    directions << WbVector3();
  else
    directions << computeCorners();
  if (directions.size() != mRayGeoms.size()) {
    deleteRays();
    createRays(devicePosition, directions, offset);
  } else {
    for (int i = 0; i < mRayGeoms.size(); ++i) {
      const WbVector3 dir = directions[i] + offset;
      dGeomRaySet(mRayGeoms[i], devicePosition.x(), devicePosition.y(), devicePosition.z(), dir.x(), dir.y(), dir.z());
      dGeomRaySetLength(mRayGeoms[i], dir.length());
    }
  }
}

bool WbObjectDetection::recomputeRayDirection(const WbAffinePlane *frustumPlanes) {
  if (mOcclusion == NONE)
    return true;

  mObject->updateTransformForPhysicsStep();
  if (!isContainedInFrustum(frustumPlanes))
    return false;

  updateRayDirection();
  return true;
}

void WbObjectDetection::mergeBounds(WbVector3 &referenceObjectSize, WbVector3 &referenceObjectRelativePosition,
                                    const WbVector3 &addedObjectSize, const WbVector3 &addedObjectRelativePosition) {
  const double minX = qMin(referenceObjectRelativePosition.x() - 0.5 * referenceObjectSize.x(),
                           addedObjectRelativePosition.x() - 0.5 * addedObjectSize.x());
  const double minY = qMin(referenceObjectRelativePosition.y() - 0.5 * referenceObjectSize.y(),
                           addedObjectRelativePosition.y() - 0.5 * addedObjectSize.y());
  const double minZ = qMin(referenceObjectRelativePosition.z() - 0.5 * referenceObjectSize.z(),
                           addedObjectRelativePosition.z() - 0.5 * addedObjectSize.z());
  const double maxX = qMax(referenceObjectRelativePosition.x() + 0.5 * referenceObjectSize.x(),
                           addedObjectRelativePosition.x() + 0.5 * addedObjectSize.x());
  const double maxY = qMax(referenceObjectRelativePosition.y() + 0.5 * referenceObjectSize.y(),
                           addedObjectRelativePosition.y() + 0.5 * addedObjectSize.y());
  const double maxZ = qMax(referenceObjectRelativePosition.z() + 0.5 * referenceObjectSize.z(),
                           addedObjectRelativePosition.z() + 0.5 * addedObjectSize.z());
  referenceObjectSize.setX(maxX - minX);
  referenceObjectSize.setY(maxY - minY);
  referenceObjectSize.setZ(maxZ - minZ);
  referenceObjectRelativePosition.setX((maxX + minX) / 2.0);
  referenceObjectRelativePosition.setY((maxY + minY) / 2.0);
  referenceObjectRelativePosition.setZ((maxZ + minZ) / 2.0);
}

bool WbObjectDetection::doesChildrenHaveBoundingObject(const WbSolid *solid) {
  if (solid->boundingObject())
    return true;
  else {
    foreach (WbSolid *sc, solid->solidChildren()) {
      if (doesChildrenHaveBoundingObject(sc))
        return true;
    }
  }
  return false;
}

bool WbObjectDetection::isWithinBounds(const WbAffinePlane *frustumPlanes, const WbBaseNode *boundingObject,
                                       WbVector3 &objectSize, WbVector3 &objectRelativePosition, const WbBaseNode *rootObject) {
  int nodeType = WB_NODE_NO_NODE;
  bool useBoundingSphere = false;
  if (boundingObject)
    nodeType = boundingObject->nodeType();
  else if (!rootObject)
    return false;
  else if (doesChildrenHaveBoundingObject(dynamic_cast<const WbSolid *>(rootObject)))
    return false;
  else
    useBoundingSphere = true;

  const WbBaseNode *referenceObject = boundingObject;
  if (useBoundingSphere)
    referenceObject = rootObject;
  const WbPose *pose = dynamic_cast<const WbPose *>(referenceObject);
  if (!pose)
    pose = referenceObject->upperPose();
  assert(pose);
  const WbMatrix3 objectRotation = pose->rotationMatrix();
  WbVector3 objectPosition = pose->position();

  if (nodeType == WB_NODE_SHAPE) {
    const WbShape *shape = static_cast<const WbShape *>(boundingObject);
    boundingObject = shape->geometry();
    return isWithinBounds(frustumPlanes, boundingObject, objectSize, objectRelativePosition);
  } else if (nodeType == WB_NODE_GROUP || nodeType == WB_NODE_POSE) {
    bool visible = false;
    const WbGroup *group = static_cast<const WbGroup *>(boundingObject);
    for (int i = 0; i < group->childCount(); ++i) {
      boundingObject = group->child(i);
      if (!visible) {
        if (isWithinBounds(frustumPlanes, boundingObject, objectSize, objectRelativePosition))
          visible = true;
      } else {
        WbVector3 newObjectSize, newObjectRelativePosition;
        if (isWithinBounds(frustumPlanes, boundingObject, newObjectSize, newObjectRelativePosition))
          mergeBounds(objectSize, objectRelativePosition, newObjectSize, newObjectRelativePosition);
      }
    }
    return visible;
  }

  const WbVector3 devicePosition = mDevice->position();
  const WbMatrix3 deviceInverseRotation = mDevice->rotationMatrix().transposed();
  if (boundingObject &&
      (nodeType == WB_NODE_BOX || nodeType == WB_NODE_INDEXED_FACE_SET || nodeType == WB_NODE_ELEVATION_GRID)) {
    QVector<WbVector3> points;
    switch (nodeType) {
      case WB_NODE_BOX: {
        const WbBox *box = static_cast<const WbBox *>(boundingObject);
        const WbVector3 size = 0.5 * box->scaledSize();
        points.append(objectRotation * WbVector3(size.x(), size.y(), size.z()) + objectPosition);
        points.append(objectRotation * WbVector3(size.x(), size.y(), -size.z()) + objectPosition);
        points.append(objectRotation * WbVector3(size.x(), -size.y(), size.z()) + objectPosition);
        points.append(objectRotation * WbVector3(size.x(), -size.y(), -size.z()) + objectPosition);
        points.append(objectRotation * WbVector3(-size.x(), size.y(), size.z()) + objectPosition);
        points.append(objectRotation * WbVector3(-size.x(), size.y(), -size.z()) + objectPosition);
        points.append(objectRotation * WbVector3(-size.x(), -size.y(), size.z()) + objectPosition);
        points.append(objectRotation * WbVector3(-size.x(), -size.y(), -size.z()) + objectPosition);
        break;
      }
      case WB_NODE_INDEXED_FACE_SET: {
        const WbIndexedFaceSet *indexedFaceSet = static_cast<const WbIndexedFaceSet *>(boundingObject);
        const WbCoordinate *coordinates = indexedFaceSet->coord();
        for (int i = 0; i < coordinates->pointSize(); ++i) {
          points.append(objectRotation * coordinates->point(i) + objectPosition);
        }
        break;
      }
      case WB_NODE_ELEVATION_GRID: {
        const WbElevationGrid *elevationGrid = static_cast<const WbElevationGrid *>(boundingObject);
        const double xSpacing = elevationGrid->xSpacing();
        const double ySpacing = elevationGrid->ySpacing();
        const int xDimension = elevationGrid->xDimension();
        const int yDimension = elevationGrid->yDimension();
        for (int i = 0; i < xDimension; ++i) {
          for (int j = 0; j < yDimension; ++j)
            points.append(objectRotation * WbVector3(xSpacing * i, elevationGrid->height(i + j * xDimension), ySpacing * j) +
                          objectPosition);
        }
        break;
      }
      default:
        assert(false);
    }

    // Remove points not in the frustum
    QList<WbVector3> pointsInFrustum;
    QList<WbVector3> pointsAtBack;
    int pointsInside = 0;
    bool isOnePointOutsidePlane[4] = {false, false, false, false};
    bool isOnePointOnCorrectSide[4] = {false, false, false, false};
    for (int i = 0; i < points.size(); ++i) {
      bool inside;
      if (mIsOmniDirectional) {
        if (frustumPlanes[PARALLEL].distance(points[i]) > 0) {
          // object is in front of the omnidirectional device
          inside = true;
          for (int j = 0; j < 4; ++j)
            isOnePointOnCorrectSide[j] = true;
        } else {
          // object is at the back of the omnidirectional device
          inside = false;
          double minDistance = 0.0;
          int minIndex = -1;
          for (int j = 0; j < PARALLEL; ++j) {
            const double d = frustumPlanes[j].distance(points[i]);
            if (d < 0) {
              inside = true;
              break;
            } else if (minIndex < 0 || d < minDistance) {
              minDistance = d;
              minIndex = j;
            }
          }
          if (inside) {
            for (int j = 0; j < PARALLEL; ++j)
              isOnePointOnCorrectSide[j] = true;
          } else {
            for (int j = 0; j < PARALLEL; ++j)
              isOnePointOutsidePlane[j] = true;
            points[i] = devicePosition + frustumPlanes[minIndex].vectorProjection(points[i] - devicePosition);
          }
        }
      } else if (frustumPlanes[PARALLEL].distance(points[i]) > 0) {  // object is in front of the planar device
        inside = true;
        for (int j = 0; j < PARALLEL; ++j) {
          if (frustumPlanes[j].distance(points[i]) < 0) {
            points[i] = devicePosition + frustumPlanes[j].vectorProjection(points[i] - devicePosition);
            isOnePointOutsidePlane[j] = true;
            inside = false;
          } else
            isOnePointOnCorrectSide[j] = true;
        }
      } else {
        pointsAtBack.append(points[i]);
        continue;  // use points at the back of the planar device only partly inside the frustum
      }

      if (inside)
        pointsInside++;
      pointsInFrustum.append(points[i]);
    }

    // no points in front of the device
    if (pointsInFrustum.size() == 0)
      return false;
    // no point inside the frustum
    if (pointsInside == 0) {
      // no point is in the frustum, we need to make sure the object is 'crossing' the frustum
      // either horizontally or vertically
      if (!(isOnePointOutsidePlane[RIGHT] && isOnePointOutsidePlane[LEFT] && isOnePointOnCorrectSide[BOTTOM] &&
            isOnePointOnCorrectSide[TOP]) &&
          !(isOnePointOutsidePlane[BOTTOM] && isOnePointOutsidePlane[TOP] && isOnePointOnCorrectSide[RIGHT] &&
            isOnePointOnCorrectSide[LEFT])) {
        return false;
      }
    }
    // add points at the back of the device to ensure the whole object is detected
    pointsInFrustum << pointsAtBack;
    // move the points in the device referential
    for (int i = 0; i < pointsInFrustum.size(); ++i)
      pointsInFrustum[i] = deviceInverseRotation * (pointsInFrustum[i] - devicePosition);

    double minX = pointsInFrustum[0].x();
    double maxX = minX;
    double minY = pointsInFrustum[0].y();
    double maxY = minY;
    double minZ = pointsInFrustum[0].z();
    double maxZ = minZ;
    for (int i = 1; i < pointsInFrustum.size(); ++i) {
      minX = qMin(minX, pointsInFrustum[i].x());
      maxX = qMax(maxX, pointsInFrustum[i].x());
      minY = qMin(minY, pointsInFrustum[i].y());
      maxY = qMax(maxY, pointsInFrustum[i].y());
      minZ = qMin(minZ, pointsInFrustum[i].z());
      maxZ = qMax(maxZ, pointsInFrustum[i].z());
    }
    objectRelativePosition = WbVector3((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);
    objectSize.setX(maxX - minX);
    objectSize.setY(maxY - minY);
    objectSize.setZ(maxZ - minZ);
    mUseBoundingSphereOnly = false;
  } else if (useBoundingSphere ||
             (boundingObject && (nodeType == WB_NODE_SPHERE || nodeType == WB_NODE_CYLINDER || nodeType == WB_NODE_CAPSULE))) {
    double outsidePart[4] = {0.0, 0.0, 0.0, 0.0};
    if (useBoundingSphere) {
      WbBoundingSphere *boundingSphere = rootObject->boundingSphere();
      const double size = 2 * boundingSphere->scaledRadius();
      objectSize.setXyz(size, size, size);
      // correct the object center
      objectPosition = pose->matrix() * boundingSphere->center();
    } else {
      double height = 0;
      double radius = 0;
      switch (nodeType) {
        case WB_NODE_SPHERE: {
          const WbSphere *sphere = static_cast<const WbSphere *>(boundingObject);
          radius = sphere->scaledRadius();
          objectSize.setX(2 * radius);
          objectSize.setY(2 * radius);
          objectSize.setZ(2 * radius);
          break;
        }
        case WB_NODE_CYLINDER: {
          const WbCylinder *cylinder = static_cast<const WbCylinder *>(boundingObject);
          height = cylinder->scaledHeight();
          radius = cylinder->scaledRadius();
          break;
        }
        case WB_NODE_CAPSULE: {
          const WbCapsule *capsule = static_cast<const WbCapsule *>(boundingObject);
          radius = capsule->scaledRadius();
          height = capsule->scaledHeight() + 2 * radius;
          break;
        }
        default:
          assert(false);
      }
      if (nodeType == WB_NODE_CYLINDER || nodeType == WB_NODE_CAPSULE) {
        const WbMatrix3 rotation = deviceInverseRotation * objectRotation;
        const double xRange =
          fabs(rotation(0, 2) * height) + 2 * radius * sqrt(qMax(0.0, 1.0 - rotation(0, 2) * rotation(0, 2)));
        const double yRange =
          fabs(rotation(1, 2) * height) + 2 * radius * sqrt(qMax(0.0, 1.0 - rotation(1, 2) * rotation(1, 2)));
        const double zRange =
          fabs(rotation(2, 2) * height) + 2 * radius * sqrt(qMax(0.0, 1.0 - rotation(2, 2) * rotation(2, 2)));
        objectSize = WbVector3(xRange, yRange, zRange);
        mUseBoundingSphereOnly = false;
      }
    }
    // check distance between center and frustum planes
    if (!mIsOmniDirectional || frustumPlanes[PARALLEL].distance(objectPosition) < 0.0) {
      // if omnidirectional frustum, then check only if objects are in the back of the device
      bool inside = false;
      for (int j = 0; j < PARALLEL; ++j) {
        const double d = frustumPlanes[j].distance(objectPosition);
        const double halfObjectSize = objectSize[j % 2 + 1] / 2.0;
        if (mIsOmniDirectional) {
          if (d < -halfObjectSize) {  // object is completely inside
            inside = true;
            break;
          }
        } else {
          if (d < -halfObjectSize)  // object is completely outside
            return false;
          if (d < halfObjectSize)  // a part of the object is outside
            outsidePart[j] = halfObjectSize - d;
          inside = true;
        }
      }

      if (!inside)
        return false;  // object not visible in case of omnidirectional device
    }

    objectRelativePosition = deviceInverseRotation * (objectPosition - devicePosition);
    if (mHorizontalFieldOfView <= M_PI_2) {
      // do not recompute the object size and position if partly outside in case of fovX > PI
      // (a more complete computation will be needed and currently it seems to work quite well as-is)
      objectSize.setY(objectSize.y() - outsidePart[RIGHT] - outsidePart[LEFT]);
      objectSize.setZ(objectSize.z() - outsidePart[BOTTOM] - outsidePart[TOP]);
      objectRelativePosition +=
        0.5 * WbVector3(0, outsidePart[RIGHT] - outsidePart[LEFT], outsidePart[BOTTOM] - outsidePart[TOP]);
    }
  }
  return true;
}

bool WbObjectDetection::recursivelyCheckIfWithinBounds(WbSolid *solid, const bool boundsInitialized,
                                                       const WbAffinePlane *frustumPlanes) {
  bool initialized = boundsInitialized;
  if (initialized) {
    WbVector3 newObjectSize, newObjectRelativePosition;
    if (isWithinBounds(frustumPlanes, solid->boundingObject(), newObjectSize, newObjectRelativePosition))
      mergeBounds(mObjectSize, mObjectRelativePosition, newObjectSize, newObjectRelativePosition);
  } else
    initialized = isWithinBounds(frustumPlanes, solid->boundingObject(), mObjectSize, mObjectRelativePosition, solid);
  foreach (WbSolid *s, solid->solidChildren())
    initialized = recursivelyCheckIfWithinBounds(s, initialized, frustumPlanes);
  return initialized;
}

bool WbObjectDetection::isContainedInFrustum(const WbAffinePlane *frustumPlanes) {
  assert(mObject);
  const bool useBoundingSphereOnly = mUseBoundingSphereOnly;
  if (!recursivelyCheckIfWithinBounds(mObject, false, frustumPlanes))
    return false;
  // check distance
  if (distance() > (mMaxRange + mObjectSize.x() / 2.0))
    return false;

  if (mOcclusion != NONE && (mRayGeoms.isEmpty() || useBoundingSphereOnly != mUseBoundingSphereOnly))
    updateRayDirection();
  return true;
}

WbAffinePlane *WbObjectDetection::computeFrustumPlanes(const WbSolid *device, const double verticalFieldOfView,
                                                       const double horizontalFieldOfView, const double maxRange,
                                                       const bool isPlanarProjection) {
  const WbVector3 devicePosition = device->position();
  const WbMatrix3 deviceRotation = device->rotationMatrix();
  // construct the 4 planes defining the sides of the frustum
  const float halfFovX = horizontalFieldOfView / 2.0;
  const float halfFovY = verticalFieldOfView / 2.0;
  double x, y, z;
  if (isPlanarProjection || halfFovX < M_PI_2) {
    x = maxRange;
    y = maxRange * tan(halfFovX);
    z = maxRange * tan(halfFovY);
  } else {
    // omnidirectional sensor with horizontal FOV > PI
    const float angleY[4] = {-halfFovY, -halfFovY, halfFovY, halfFovY};
    const float angleX[4] = {halfFovX, -halfFovX, -halfFovX, halfFovX};
    for (int k = 0; k < 4; ++k) {
      const float helper = cosf(angleY[k]);
      // get x, y and z from the spherical coordinates
      if (angleY[k] > M_PI_4 || angleY[k] < -M_PI_4)
        y = maxRange * cosf(angleY[k] + M_PI_2) * sinf(angleX[k]);
      else
        y = maxRange * helper * sinf(angleX[k]);
      z = maxRange * sinf(angleY[k]);
      x = maxRange * helper * cosf(angleX[k]);
    }
  }
  const WbVector3 topRightCorner = devicePosition + deviceRotation * WbVector3(x, -y, z);
  const WbVector3 topLeftCorner = devicePosition + deviceRotation * WbVector3(x, y, z);
  const WbVector3 bottomRightCorner = devicePosition + deviceRotation * WbVector3(x, -y, -z);
  const WbVector3 bottomLeftCorner = devicePosition + deviceRotation * WbVector3(x, y, -z);
  WbAffinePlane *planes = new WbAffinePlane[PLANE_NUMBER];
  planes[RIGHT] = WbAffinePlane(devicePosition, topRightCorner, bottomRightCorner);              // right plane
  planes[BOTTOM] = WbAffinePlane(devicePosition, bottomRightCorner, bottomLeftCorner);           // bottom plane
  planes[LEFT] = WbAffinePlane(devicePosition, bottomLeftCorner, topLeftCorner);                 // left plane
  planes[TOP] = WbAffinePlane(devicePosition, topLeftCorner, topRightCorner);                    // top plane
  planes[PARALLEL] = WbAffinePlane(deviceRotation * WbVector3(maxRange, 0, 0), devicePosition);  // device plane
  return planes;
}

QList<WbVector3> WbObjectDetection::computeCorners() const {
  QList<WbVector3> points;
  WbVector3 size = 0.5 * mObjectSize;
  if (mUseBoundingSphereOnly) {  // 6 rays for bounding sphere
    points << WbVector3(size.x(), 0, 0);
    points << WbVector3(-size.x(), 0, 0);
    points << WbVector3(0, size.y(), 0);
    points << WbVector3(0, -size.y(), 0);
    points << WbVector3(0, 0, size.z());
    points << WbVector3(0, 0, -size.z());
  } else {  // 8 rays for bounding box
    points << WbVector3(size.x(), size.y(), size.z());
    points << WbVector3(-size.x(), size.y(), size.z());
    points << WbVector3(size.x(), -size.y(), size.z());
    points << WbVector3(-size.x(), -size.y(), size.z());
    points << WbVector3(size.x(), size.y(), -size.z());
    points << WbVector3(-size.x(), size.y(), -size.z());
    points << WbVector3(size.x(), -size.y(), -size.z());
    points << WbVector3(-size.x(), -size.y(), -size.z());
  }
  return points;
}
