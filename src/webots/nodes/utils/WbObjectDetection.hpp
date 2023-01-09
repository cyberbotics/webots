// Copyright 1996-2023 Cyberbotics Ltd.
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

#ifndef WB_OBJECT_DETECTION_HPP
#define WB_OBJECT_DETECTION_HPP

#include "WbMatrix3.hpp"
#include "WbVector3.hpp"

#include <ode/ode.h>

class WbAffinePlane;
class WbBaseNode;
class WbSolid;

class WbObjectDetection {
public:
  enum FrustumPlane { LEFT = 0, BOTTOM, RIGHT, TOP, PARALLEL, PLANE_NUMBER };

  WbObjectDetection(WbSolid *device, WbSolid *object, bool needToCheckCollision, double maxRange);
  virtual ~WbObjectDetection();

  bool hasCollided() const;
  WbSolid *device() const { return mDevice; }
  dGeomID geom() const { return mGeom; }
  WbSolid *object() const { return mObject; }
  const WbVector3 &objectSize() const { return mObjectSize; }
  const WbVector3 &objectRelativePosition() const { return mObjectRelativePosition; }

  void setCollided(double depth);

  void deleteRay();

  // Recomputes ray position and direction and returns if the current ray is valid or can be removed.
  bool recomputeRayDirection(const WbAffinePlane *frustumPlanes);

  // Computes whether the object is contained in the given frustum.
  bool isContainedInFrustum(const WbAffinePlane *frustumPlanes);

  // Computes the frustum plane for the given device ray.
  static WbAffinePlane *computeFrustumPlanes(const WbSolid *device, const double verticalFieldOfView,
                                             const double horizontalFieldOfView, const double maxRange);

private:
  static void mergeBounds(WbVector3 &referenceObjectSize, WbVector3 &referenceObjectRelativePosition,
                          const WbVector3 &addedObjectSize, const WbVector3 &addedObjectRelativePosition);
  static bool doesChildrenHaveBoundingObject(const WbSolid *solid);

  // Checks whether the object is in the bounds of the `frustumPlanes` frustum.
  //
  // @param[in] frustumPlanes Frustum of the device.
  // @param[in] boundingObject Bounding object of the target object.
  // @param[out] objectSize AABB of the target object.
  // @param[out] objectRelativePosition The object's position in respect to the device. The center of the object is calculated
  // from AABB points.
  // @param[in] rootObject If `rootObject` and `boundingObject` are not defined the method returns false.
  // @return Returns `true` if the object is inside the frustum, `false` otherwise.
  bool isWithinBounds(const WbAffinePlane *frustumPlanes, const WbBaseNode *boundingObject, WbVector3 &objectSize,
                      WbVector3 &objectRelativePosition, const WbBaseNode *rootObject = NULL);
  // Checks whether the object and its solid children are located visible in the `frustumPlanes` frustum.
  bool recursivelyCheckIfWithinBounds(WbSolid *solid, bool boundsInitialized, const WbAffinePlane *frustumPlanes);
  virtual double distance() = 0;

  WbSolid *mDevice;
  WbVector3 mObjectRelativePosition;
  WbVector3 mObjectSize;
  WbSolid *mObject;
  double mMaxRange;
  double mCollisionDepth;  // the geom collision depth
  dGeomID mGeom;           // geom that checks collision of this packet
};

#endif
