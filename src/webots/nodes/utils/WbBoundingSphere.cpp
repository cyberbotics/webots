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

#include "WbBoundingSphere.hpp"

#include "WbBaseNode.hpp"
#include "WbMatrix3.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPose.hpp"
#include "WbRay.hpp"
#include "WbRotation.hpp"
#include "WbShape.hpp"
#include "WbSkin.hpp"
#include "WbTransform.hpp"

#include <cassert>

bool gUpdatesEnabled = false;
bool gRayTracingEnabled = false;

void WbBoundingSphere::enableUpdates(bool enabled, WbBoundingSphere *root) {
  gUpdatesEnabled = enabled;
  if (enabled && root) {
    QList<WbBoundingSphere *> spheres;
    spheres << root;
    while (!spheres.isEmpty()) {
      WbBoundingSphere *bs = spheres.takeFirst();
      bs->mBoundSpaceDirty = true;
      bs->mParentCoordinatesDirty = true;
      spheres << bs->mSubBoundingSpheres;
    }
  }
}

WbBoundingSphere::WbBoundingSphere(const WbBaseNode *owner) :
  mRadius(0.0),
  mParentBoundingSphere(NULL),
  mOwner(NULL),
  mGeomOwner(NULL),
  mSkinOwner(NULL),
  mPoseOwner(NULL),
  mBoundSpaceDirty(false),
  mParentCoordinatesDirty(true),
  mRadiusInParentCoordinates(0.0),
  mGlobalCoordinatesUpdateTime(-1.0),
  mRadiusInGlobalCoordinates(0.0) {
  setOwner(owner);
}

WbBoundingSphere::WbBoundingSphere(const WbBaseNode *owner, const WbVector3 &center, double radius) :
  mCenter(center),
  mRadius(radius),
  mParentBoundingSphere(NULL),
  mOwner(NULL),
  mGeomOwner(NULL),
  mSkinOwner(NULL),
  mPoseOwner(NULL),
  mBoundSpaceDirty(true),
  mParentCoordinatesDirty(true),
  mRadiusInParentCoordinates(0.0),
  mGlobalCoordinatesUpdateTime(-1.0),
  mRadiusInGlobalCoordinates(0.0) {
  setOwner(owner);
}

WbBoundingSphere::~WbBoundingSphere() {
  foreach (WbBoundingSphere *sub, mSubBoundingSpheres)
    sub->setParentBoundingSphere(NULL);
  if (mParentBoundingSphere)
    mParentBoundingSphere->removeSubBoundingSphere(this);
}

void WbBoundingSphere::setOwner(const WbBaseNode *owner) {
  mOwner = owner;
  mPoseOwner = dynamic_cast<const WbPose *>(mOwner);
  mGeomOwner = dynamic_cast<const WbGeometry *>(mOwner);
  mSkinOwner = dynamic_cast<const WbSkin *>(mOwner);
}

double WbBoundingSphere::scaledRadius() {
  if (mBoundSpaceDirty)
    recomputeIfNeeded();
  double globalRadius;
  WbVector3 globalCenter;
  computeSphereInGlobalCoordinates(globalCenter, globalRadius);
  return globalRadius;
}

double WbBoundingSphere::radiusInParentCoordinates() {
  if (mBoundSpaceDirty)
    recomputeIfNeeded();
  if (mParentCoordinatesDirty)
    recomputeSphereInParentCoordinates();
  return mRadiusInParentCoordinates;
}

const WbVector3 &WbBoundingSphere::center() {
  if (mBoundSpaceDirty)
    recomputeIfNeeded();
  return mCenter;
}

const WbVector3 &WbBoundingSphere::centerInParentCoordinates() {
  if (mBoundSpaceDirty)
    recomputeIfNeeded();
  if (mParentCoordinatesDirty)
    recomputeSphereInParentCoordinates();
  return mCenterInParentCoordinates;
}

void WbBoundingSphere::empty() {
  set(WbVector3(), 0.0);
}

bool WbBoundingSphere::isEmpty() const {
  return mRadius == 0.0 && mCenter == WbVector3();
}

void WbBoundingSphere::set(const WbVector3 &center, const double radius) {
  if (mCenter == center && mRadius == radius)
    return;
  mCenter = center;
  mRadius = radius;
  if (!gRayTracingEnabled && !mBoundSpaceDirty) {
    mBoundSpaceDirty = true;
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

void WbBoundingSphere::addSubBoundingSphereToParentNode(const WbBaseNode *node) {
  const WbBaseNode *parent = dynamic_cast<const WbBaseNode *>(node->parentNode());
  while (parent) {
    if (parent->boundingSphere()) {
      parent->boundingSphere()->addSubBoundingSphere(node->boundingSphere());
      return;
    }
    parent = dynamic_cast<const WbBaseNode *>(parent->parentNode());
  }
}

void WbBoundingSphere::addSubBoundingSphere(WbBoundingSphere *subBoundingSphere) {
  if (!subBoundingSphere || mSubBoundingSpheres.contains(subBoundingSphere))
    return;
  mSubBoundingSpheres.append(subBoundingSphere);
  subBoundingSphere->setParentBoundingSphere(this);
  if (!mBoundSpaceDirty) {
    mBoundSpaceDirty = true;
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

void WbBoundingSphere::removeSubBoundingSphere(WbBoundingSphere *boundingSphere) {
  if (!mSubBoundingSpheres.contains(boundingSphere))
    return;
  mSubBoundingSpheres.removeOne(boundingSphere);
  if (mSubBoundingSpheres.isEmpty())
    empty();
  else if (!mBoundSpaceDirty) {
    mBoundSpaceDirty = true;
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

void WbBoundingSphere::enclose(const WbVector3 &point) {
  if (isEmpty()) {
    set(point, 0);
    return;
  }
  // Test if the sphere contains the point.
  if ((point - mCenter).length() <= mRadius)
    return;

  const WbVector3 delta = mCenter - point;
  const double newRadius = (delta.length() + mRadius) / 2.0;
  set(point + delta.normalized() * newRadius, newRadius);
}

bool WbBoundingSphere::enclose(const WbBoundingSphere *other) {
  if (other->isEmpty())
    return false;

  const WbVector3 &otherCenter = const_cast<WbBoundingSphere *>(other)->centerInParentCoordinates();
  const double otherRadius = const_cast<WbBoundingSphere *>(other)->radiusInParentCoordinates();
  if (isEmpty()) {
    set(otherCenter, otherRadius);
    return true;
  }

  // Test matching centers
  if (mCenter == otherCenter) {
    if (otherRadius > mRadius) {
      set(mCenter, otherRadius);
      return true;
    }
    return false;
  }

  const WbVector3 &distanceVector = otherCenter - mCenter;
  const double distance = distanceVector.length();
  const double sum = mRadius + distance + otherRadius;

  // Other is inside the instance
  if (sum <= mRadius * 2)
    return false;
  // Other contains the instance
  if (sum <= otherRadius * 2) {
    set(otherCenter, otherRadius);
    return true;
  }

  // General case
  // compute radius of the sphere which includes the two spheres.
  const double newRadius = sum / 2.0;
  set(mCenter + distanceVector.normalized() * (newRadius - mRadius), newRadius);
  return true;
}

void WbBoundingSphere::recomputeSphereInParentCoordinates() {
  if (!mParentCoordinatesDirty)
    return;

  if (mPoseOwner != NULL) {
    const WbTransform *const t = dynamic_cast<const WbTransform *const>(mPoseOwner);
    const WbVector3 &scale = t ? t->scale() : WbVector3(1.0, 1.0, 1.0);
    mRadiusInParentCoordinates = std::max(std::max(scale.x(), scale.y()), scale.z()) * mRadius;
    mCenterInParentCoordinates = mPoseOwner->vrmlMatrix() * mCenter;
  } else {
    mRadiusInParentCoordinates = mRadius;
    mCenterInParentCoordinates = mCenter;
  }
  mParentCoordinatesDirty = false;
}

void WbBoundingSphere::computeSphereInGlobalCoordinates(WbVector3 &center, double &radius) {
  const WbPose *upperPose = dynamic_cast<const WbPose *>(mPoseOwner);
  if (upperPose == NULL)
    upperPose = WbNodeUtilities::findUpperPose(mOwner);
  if (upperPose) {
    const WbTransform *t = dynamic_cast<const WbTransform *>(upperPose);
    if (t == NULL)
      t = upperPose->upperTransform();
    if (t) {
      const WbVector3 &scale = t->absoluteScale();
      radius = std::max(std::max(scale.x(), scale.y()), scale.z()) * mRadius;
    } else
      radius = mRadius;
    center = upperPose->matrix() * mCenter;
  } else {
    radius = mRadius;
    center = mCenter;
  }
}

void WbBoundingSphere::recomputeIfNeeded(bool dirtyOnly) {
  QSet<const WbBoundingSphere *> visited;
  recomputeIfNeededInternal(dirtyOnly, visited);
}

void WbBoundingSphere::recomputeIfNeededInternal(bool dirtyOnly, QSet<const WbBoundingSphere *> &visited) {
  // prevent infinite loop in case of SolidReference nodes
  if (visited.contains(this))
    return;
  visited << this;

  if ((dirtyOnly || gUpdatesEnabled) && !mBoundSpaceDirty)
    return;

  if (mSubBoundingSpheres.empty()) {
    // geometry or empty bounding sphere
    if (mGeomOwner || mSkinOwner) {
      if (mGeomOwner)
        mGeomOwner->recomputeBoundingSphere();
      else
        mSkinOwner->recomputeBoundingSphere();
    }
    mBoundSpaceDirty = false;
    return;
  }
  const WbVector3 prevCenter = mCenter;
  const double prevRadius = mRadius;
  mCenter = WbVector3();
  mRadius = 0.0;
  bool prevState = gRayTracingEnabled;
  gRayTracingEnabled = true;
  foreach (WbBoundingSphere *sub, mSubBoundingSpheres) {
    sub->recomputeIfNeededInternal(true, visited);
    if (sub->isEmpty())
      continue;
    enclose(sub);
  }
  gRayTracingEnabled = prevState;

  if (mParentBoundingSphere && (mCenter != prevCenter || mRadius != prevRadius))
    mParentCoordinatesDirty = true;
  mBoundSpaceDirty = false;
}

void WbBoundingSphere::parentUpdateNotification() const {
  if (mParentBoundingSphere) {
    WbBoundingSphere *parent = mParentBoundingSphere;
    while (parent != NULL) {
      parent->mBoundSpaceDirty = true;
      parent->mParentCoordinatesDirty = true;
      parent = parent->mParentBoundingSphere;
    }
  }
}

void WbBoundingSphere::setOwnerMoved() {
  assert(mPoseOwner);
  if (mParentBoundingSphere && !mParentCoordinatesDirty) {
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

void WbBoundingSphere::setOwnerSizeChanged() {
  assert(mGeomOwner || mSkinOwner || mPoseOwner);
  if (!mBoundSpaceDirty) {
    mBoundSpaceDirty = true;
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

WbBoundingSphere::IntersectingShape WbBoundingSphere::computeIntersection(const WbRay &ray, double timeStep) {
  recomputeIfNeeded();
  if (mGlobalCoordinatesUpdateTime < timeStep) {
    computeSphereInGlobalCoordinates(mCenterInGlobalCoordinates, mRadiusInGlobalCoordinates);
    mGlobalCoordinatesUpdateTime = timeStep;
  }

  IntersectingShape res(0.0, NULL);
  if (isEmpty())
    return res;

  std::pair<bool, double> intersectionPair = ray.intersects(mCenterInGlobalCoordinates, mRadiusInGlobalCoordinates);

  if (!intersectionPair.first)
    return res;

  // This sphere is intersected, therefore, test if one sub sphere is intersected
  if (mSubBoundingSpheres.isEmpty()) {
    if (mGeomOwner != NULL) {
      const double d = mGeomOwner->computeDistance(ray);
      if (d > 0.0) {
        res.shape = dynamic_cast<WbShape *>(mGeomOwner->parentNode());
        res.distance = d;
      }
    }
  } else {
    foreach (WbBoundingSphere *sub, mSubBoundingSpheres) {
      IntersectingShape intersection = sub->computeIntersection(ray, timeStep);
      if (intersection.shape != NULL && intersection.distance > 0 &&
          (res.shape == NULL || intersection.distance < res.distance)) {
        res.shape = intersection.shape;
        res.distance = intersection.distance;
      }
    }
  }
  return res;
}
