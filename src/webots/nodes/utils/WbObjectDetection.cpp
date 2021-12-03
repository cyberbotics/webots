// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
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

WbObjectDetection::WbObjectDetection(WbSolid *device, WbSolid *object, bool needToCheckCollision, double maxRange) :
  mObjectRelativePosition(0.0, 0.0, 0.0),
  mObjectSize(0.0, 0.0, 0.0),
  mObject(object),
  mMaxRange(maxRange),
  mCollisionDepth(0.0),
  mGeom(NULL) {
  if (needToCheckCollision) {
    assert(device);

    // setup ray geom for ODE collision detection
    const WbVector3 devicePosition = device->matrix().translation();
    const WbVector3 objectPosition = object->matrix().translation();
    const WbVector3 direction = objectPosition - devicePosition;
    mGeom = dCreateRay(WbOdeContext::instance()->space(), direction.length());
    dGeomSetDynamicFlag(mGeom);
    dGeomRaySet(mGeom, devicePosition.x(), devicePosition.y(), devicePosition.z(), direction.x(), direction.y(), direction.z());
    dGeomRaySetLength(mGeom, direction.length());

    // set device as callback data in case there is a collision
    dGeomSetData(mGeom, new WbOdeGeomData(device));
  }
};

WbObjectDetection::~WbObjectDetection() {
  deleteRay();
}

bool WbObjectDetection::hasCollided() const {
  return mCollisionDepth > 0.5 * mObjectSize.x();
}

void WbObjectDetection::deleteRay() {
  if (mGeom) {
    WbOdeGeomData *odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(mGeom));
    delete odeGeomData;
    dGeomDestroy(mGeom);
    mGeom = NULL;
  }
}

void WbObjectDetection::setCollided(double depth) {
  double distance = dGeomRayGetLength(mGeom) - depth;
  if (mCollisionDepth < distance)
    mCollisionDepth = distance;
}

bool WbObjectDetection::recomputeRayDirection(WbSolid *device, const WbVector3 &devicePosition, const WbMatrix3 &deviceRotation,
                                              const WbMatrix3 &deviceInverseRotation, const WbAffinePlane *frustumPlanes) {
  assert(mGeom);
  mObject->updateTransformForPhysicsStep();
  // recompute ray properties
  if (!computeObject(devicePosition, deviceRotation, deviceInverseRotation, frustumPlanes))
    return false;

  const WbVector3 objectPosition = mObject->matrix().translation();
  const WbVector3 direction = objectPosition - devicePosition;
  dGeomRaySet(mGeom, devicePosition.x(), devicePosition.y(), devicePosition.z(), direction.x(), direction.y(), direction.z());
  dGeomRaySetLength(mGeom, direction.length());
  return true;
}

void WbObjectDetection::mergeBounds(WbVector3 &referenceObjectSize, WbVector3 &referenceObjectRelativePosition,
                                    const WbVector3 &addedObjectSize, const WbVector3 &addedObjectRelativePosition) {
  double minX = qMin(referenceObjectRelativePosition.x() - 0.5 * referenceObjectSize.x(),
                     addedObjectRelativePosition.x() - 0.5 * addedObjectSize.x());
  double minY = qMin(referenceObjectRelativePosition.y() - 0.5 * referenceObjectSize.y(),
                     addedObjectRelativePosition.y() - 0.5 * addedObjectSize.y());
  double minZ = qMin(referenceObjectRelativePosition.z() - 0.5 * referenceObjectSize.z(),
                     addedObjectRelativePosition.z() - 0.5 * addedObjectSize.z());
  double maxX = qMax(referenceObjectRelativePosition.x() + 0.5 * referenceObjectSize.x(),
                     addedObjectRelativePosition.x() + 0.5 * addedObjectSize.x());
  double maxY = qMax(referenceObjectRelativePosition.y() + 0.5 * referenceObjectSize.y(),
                     addedObjectRelativePosition.y() + 0.5 * addedObjectSize.y());
  double maxZ = qMax(referenceObjectRelativePosition.z() + 0.5 * referenceObjectSize.z(),
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
    foreach (WbSolid *solidChildren, solid->solidChildren()) {
      if (doesChildrenHaveBoundingObject(solidChildren))
        return true;
    }
  }
  return false;
}

bool WbObjectDetection::computeBounds(const WbVector3 &devicePosition, const WbMatrix3 &deviceRotation,
                                      const WbMatrix3 &deviceInverseRotation, const WbAffinePlane *frustumPlanes,
                                      const WbBaseNode *boundingObject, WbVector3 &objectSize,
                                      WbVector3 &objectRelativePosition, const WbBaseNode *rootObject) {
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
  const WbTransform *transform = dynamic_cast<const WbTransform *>(referenceObject);
  if (!transform)
    transform = referenceObject->upperTransform();
  assert(transform);
  WbMatrix3 objectRotation = transform->rotationMatrix();
  WbVector3 objectPosition = transform->matrix().translation();

  if (nodeType == WB_NODE_SHAPE) {
    const WbShape *shape = static_cast<const WbShape *>(boundingObject);
    boundingObject = shape->geometry();
    return computeBounds(devicePosition, deviceRotation, deviceInverseRotation, frustumPlanes, boundingObject, objectSize,
                         objectRelativePosition);
  } else if (nodeType == WB_NODE_GROUP || nodeType == WB_NODE_TRANSFORM) {
    bool visible = false;
    const WbGroup *group = static_cast<const WbGroup *>(boundingObject);
    for (int i = 0; i < group->childCount(); ++i) {
      boundingObject = group->child(i);
      if (!visible) {
        if (computeBounds(devicePosition, deviceRotation, deviceInverseRotation, frustumPlanes, boundingObject, objectSize,
                          objectRelativePosition))
          visible = true;
      } else {
        WbVector3 newObjectSize, newObjectRelativePosition;
        if (computeBounds(devicePosition, deviceRotation, deviceInverseRotation, frustumPlanes, boundingObject, newObjectSize,
                          newObjectRelativePosition))
          mergeBounds(objectSize, objectRelativePosition, newObjectSize, newObjectRelativePosition);
      }
    }
    return visible;
  }

  if (boundingObject &&
      (nodeType == WB_NODE_BOX || nodeType == WB_NODE_INDEXED_FACE_SET || nodeType == WB_NODE_ELEVATION_GRID)) {
    QVector<WbVector3> points;
    switch (nodeType) {
      case WB_NODE_BOX: {
        const WbBox *box = static_cast<const WbBox *>(boundingObject);
        WbVector3 size = 0.5 * box->scaledSize();
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
          points.append(objectRotation * (coordinates->point(i) * mObject->absoluteScale()) + objectPosition);
        }
        break;
      }
      case WB_NODE_ELEVATION_GRID: {
        const WbElevationGrid *elevationGrid = static_cast<const WbElevationGrid *>(boundingObject);
        double xSpacing = elevationGrid->xSpacing();
        double ySpacing = elevationGrid->ySpacing();
        int xDimension = elevationGrid->xDimension();
        int yDimension = elevationGrid->yDimension();
        for (int i = 0; i < xDimension; ++i) {
          for (int j = 0; j < yDimension; ++j)
            points.append(objectRotation * (WbVector3(xSpacing * i, elevationGrid->height(i + j * xDimension), ySpacing * j) *
                                            mObject->absoluteScale()) +
                          objectPosition);
        }
        break;
      }
      default:
        assert(false);
    }
    // Remove points not in the frustum
    QVector<WbVector3> pointsInFrustum;
    int pointsInside = 0;
    bool isOnePointOutsidePlane[4] = {false, false, false, false};
    bool isOnePointOnCorrectSide[4] = {false, false, false, false};
    for (int i = 0; i < points.size(); ++i) {
      if (frustumPlanes[PARALLEL].distance(points[i]) > 0) {  // object is in front of the device
        bool inside = true;
        for (int j = 0; j < 4; ++j) {
          if (frustumPlanes[j].distance(points[i]) < 0) {
            points[i] = devicePosition + frustumPlanes[j].vectorProjection(points[i] - devicePosition);
            inside = false;
            isOnePointOutsidePlane[j] = true;
          } else
            isOnePointOnCorrectSide[j] = true;
        }
        if (inside)
          pointsInside++;
        pointsInFrustum.append(points[i]);
      }
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
  } else if (useBoundingSphere ||
             (boundingObject && (nodeType == WB_NODE_SPHERE || nodeType == WB_NODE_CYLINDER || nodeType == WB_NODE_CAPSULE))) {
    double outsidePart[4] = {0.0, 0.0, 0.0, 0.0};
    if (useBoundingSphere) {
      WbBoundingSphere *boundingSphere = rootObject->boundingSphere();
      const WbVector3 &scale = transform->absoluteScale();
      const double size = 2 * boundingSphere->radius() * std::max(std::max(scale.x(), scale.y()), scale.z());
      objectSize.setXyz(size, size, size);
      // correct the object center
      objectPosition += boundingSphere->center();
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
      }
    }
    // check distance between center and frustum planes
    // Note: this sort of detection is only adapted for a field of view smaller than PI. If a larger FoV is desirable, the logic
    // should be changed by having two separate frustums, more details here: https://github.com/cyberbotics/webots/pull/3960
    for (int j = 0; j < 4; ++j) {
      const double distance = frustumPlanes[j].distance(objectPosition);
      const int objectAxis = j % 2 + 1;
      if (distance < -objectSize[objectAxis] / 2.0)  // the object is completely outside
        return false;
      else if (distance < objectSize[objectAxis] / 2.0)  // a part of the object is outside
        outsidePart[j] = objectSize[objectAxis] / 2.0 - distance;
    }
    objectSize.setY(objectSize.y() - outsidePart[RIGHT] - outsidePart[LEFT]);
    objectSize.setZ(objectSize.z() - outsidePart[BOTTOM] - outsidePart[TOP]);
    objectRelativePosition = deviceInverseRotation * (objectPosition - devicePosition);
    objectRelativePosition +=
      0.5 * WbVector3(0, outsidePart[RIGHT] - outsidePart[LEFT], outsidePart[BOTTOM] - outsidePart[TOP]);
  }
  return true;
}

bool WbObjectDetection::recursivelyComputeBounds(WbSolid *solid, bool boundsInitialized, const WbVector3 &devicePosition,
                                                 const WbMatrix3 &deviceRotation, const WbMatrix3 &deviceInverseRotation,
                                                 const WbAffinePlane *frustumPlanes) {
  bool initialized = boundsInitialized;
  if (initialized) {
    WbVector3 newObjectSize, newObjectRelativePosition;
    if (computeBounds(devicePosition, deviceRotation, deviceInverseRotation, frustumPlanes, solid->boundingObject(),
                      newObjectSize, newObjectRelativePosition))
      mergeBounds(mObjectSize, mObjectRelativePosition, newObjectSize, newObjectRelativePosition);
  } else
    initialized = computeBounds(devicePosition, deviceRotation, deviceInverseRotation, frustumPlanes, solid->boundingObject(),
                                mObjectSize, mObjectRelativePosition, solid);
  foreach (WbSolid *solid, solid->solidChildren())
    initialized =
      recursivelyComputeBounds(solid, initialized, devicePosition, deviceRotation, deviceInverseRotation, frustumPlanes);
  return initialized;
}

bool WbObjectDetection::computeObject(const WbVector3 &devicePosition, const WbMatrix3 &deviceRotation,
                                      const WbMatrix3 &deviceInverseRotation, const WbAffinePlane *frustumPlanes) {
  assert(mObject);
  if (!recursivelyComputeBounds(mObject, false, devicePosition, deviceRotation, deviceInverseRotation, frustumPlanes))
    return false;
  // check distance
  if (distance() > (mMaxRange + mObjectSize.x() / 2.0))
    return false;

  return true;
}

WbAffinePlane *WbObjectDetection::computeFrustumPlanes(const WbVector3 &devicePosition, const WbMatrix3 &deviceRotation,
                                                       const double verticalFieldOfView, const double horizontalFieldOfView,
                                                       const double maxRange) {
  // construct the 4 planes defining the sides of the frustum
  const double z = maxRange * tan(verticalFieldOfView / 2.0);
  const double y = maxRange * tan(horizontalFieldOfView / 2.0);
  const double x = maxRange;
  const WbVector3 topRightCorner = devicePosition + deviceRotation * WbVector3(x, -y, z);
  const WbVector3 topLeftCorner = devicePosition + deviceRotation * WbVector3(x, y, z);
  const WbVector3 bottomRightCorner = devicePosition + deviceRotation * WbVector3(x, -y, -z);
  const WbVector3 bottomLeftCorner = devicePosition + deviceRotation * WbVector3(x, y, -z);
  WbAffinePlane *planes = new WbAffinePlane[PLANE_NUMBER];
  planes[RIGHT] = WbAffinePlane(devicePosition, topRightCorner, bottomRightCorner);       // right plane
  planes[BOTTOM] = WbAffinePlane(devicePosition, bottomRightCorner, bottomLeftCorner);    // bottom plane
  planes[LEFT] = WbAffinePlane(devicePosition, bottomLeftCorner, topLeftCorner);          // left plane
  planes[TOP] = WbAffinePlane(devicePosition, topLeftCorner, topRightCorner);             // top plane
  planes[PARALLEL] = WbAffinePlane(deviceRotation * WbVector3(x, 0, 0), devicePosition);  // device plane
  for (int i = 0; i < PLANE_NUMBER; ++i)
    planes[i].normalize();
  return planes;
}
