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

#ifndef WB_BOUNDING_SPHERE_HPP
#define WB_BOUNDING_SPHERE_HPP

//
// Description: The WbBoundingSphere class represents a virtual sphere which
//              bounds a 3D space. It can be used to easily represent
//              object's bounds.
//
//              The WbBoundingSphere is associated with finalized WbGroup and
//              WbGeometry nodes (referred as 'owner'). The bounding sphere is
//              not only used for Webots ray tracing, but also for graphical
//              functionalities (handles position, viewpoint move), so any WbGroup
//              or WbGeometry instance have a bounding sphere including bounding
//              objects.
//
//              The WbBoundingSphere is also included in a tree of WbBoundingSphere
//              instances using the 'parent' and 'subBoundingSphere' concepts.
//              But the sphere's radius and center are expressed in the owner node
//              coordinate system.
//
//              All the bound spaces information are cached and only computed
//              on request. But cached sphere's values are only set to dirty for
//              nodes involved in ray tracing, i.e. not for bounding objects nodes.
//              Thus for graphical functionalities it is important to recompute
//              the sphere before using it.
//

#include "WbVector3.hpp"

#include <QtCore/QObject>
#include <QtCore/QSet>

class WbBoundingSphere;
class WbShape;
class WbSkin;

class WbPose;
class WbBaseNode;
class WbGeometry;
class WbRay;

class WbBoundingSphere {
public:
  // Enable propagation of dirty state to parent bounding spheres
  static void enableUpdates(bool enabled, WbBoundingSphere *root = NULL);
  // utility function to search for parent bounding sphere in case of skipped levels
  // like Joint and Shape nodes
  static void addSubBoundingSphereToParentNode(const WbBaseNode *node);

  explicit WbBoundingSphere(const WbBaseNode *owner);
  WbBoundingSphere(const WbBaseNode *owner, const WbVector3 &center, double radius);
  virtual ~WbBoundingSphere();

  double scaledRadius();
  const WbVector3 &center();

  void computeSphereInGlobalCoordinates(WbVector3 &center, double &radius);

  // Set the bound space to be empty.
  void empty();
  bool isEmpty() const;

  void resetGlobalCoordinatesUpdateTime() { mGlobalCoordinatesUpdateTime = -1.0; }

  // Set relative center to owner coordinate system
  void set(const WbVector3 &center, const double radius);

  void setOwnerMoved();
  void setOwnerSizeChanged();

  // Manage the list of descending bounding spheres,
  // i.e. the bounding sphere of children nodes of the owner.
  // In case of Joint, Slot, and Shape nodes that doesn't have bounding spheres,
  // the bounding spheres of descendant nodes, like Geometry, is added directly
  // in the list of the parent node, i.e. these hierarchy levels are removed.
  void addSubBoundingSphere(WbBoundingSphere *subBoundingSphere);
  void removeSubBoundingSphere(WbBoundingSphere *boundingSphere);

  // Adjust bound space to include this point.
  // The point must be in local owner coordinates.
  void enclose(const WbVector3 &point);

  // Modify the bound space to also bound the other's one.
  // The resulting sphere is the sphere which bounds both bounding spheres.
  // If the instance is empty, it will be overriden.
  // If the other is empty, it will simply be ignored.
  // Return true if the merge has changed this instance.
  bool enclose(const WbBoundingSphere *other);

  // Compute intersection of a ray with this bound space and returns
  // the shape owner of the sub bounding sphere with minimal distance.
  // Ray must be in world coordinates.
  struct IntersectingShape {
    double distance;
    const WbShape *shape;
    IntersectingShape(double distance, const WbShape *shape) : distance(distance), shape(shape) {}
  };
  IntersectingShape computeIntersection(const WbRay &ray, double timeStep);

  void recomputeIfNeeded(bool dirtyOnly = true);

private:
  WbVector3 mCenter;
  double mRadius;
  WbBoundingSphere *mParentBoundingSphere;
  QList<WbBoundingSphere *> mSubBoundingSpheres;
  const WbBaseNode *mOwner;
  const WbGeometry *mGeomOwner;
  const WbSkin *mSkinOwner;
  const WbPose *mPoseOwner;

  // Cached values
  bool mBoundSpaceDirty;  // center and radius update required
  // to speed-up recomputation
  bool mParentCoordinatesDirty;
  WbVector3 mCenterInParentCoordinates;
  double mRadiusInParentCoordinates;
  // to speed-up ray tracing
  double mGlobalCoordinatesUpdateTime;
  WbVector3 mCenterInGlobalCoordinates;
  double mRadiusInGlobalCoordinates;

  void recomputeIfNeededInternal(bool dirtyOnly, QSet<const WbBoundingSphere *> &visited);
  void recomputeSphereInParentCoordinates();
  double radiusInParentCoordinates();
  const WbVector3 &centerInParentCoordinates();
  void parentUpdateNotification() const;
  void setParentBoundingSphere(WbBoundingSphere *parent) { mParentBoundingSphere = parent; }

  // setOwner doesn't work correctly if called from the 'node' constructors
  // because of dynamic_cast
  void setOwner(const WbBaseNode *owner);
};

#endif  // WB_BOUNDING_SPHERE_HPP
