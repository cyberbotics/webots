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

#include "Primitive.hpp"

#include "Debug.hpp"
#include "Frustum.hpp"

#include <algorithm>
#include <cmath>

namespace wren {
  namespace primitive {

    Plane::Plane(const glm::vec3 &normal, float negativeDistance) : mNormal(normal), mNegativeDistance(negativeDistance) {
    }

    Aabb::Aabb(const glm::vec3 &min, const glm::vec3 &max) : mBounds{min, max} {
    }

    Aabb::Aabb(const std::vector<glm::vec3> &vertices) {
      assert(vertices.size());

      mBounds[0] = mBounds[1] = vertices.front();
      for (size_t i = 1; i < vertices.size(); ++i) {
        if (vertices[i].x < mBounds[0].x)
          mBounds[0].x = vertices[i].x;
        if (vertices[i].y < mBounds[0].y)
          mBounds[0].y = vertices[i].y;
        if (vertices[i].z < mBounds[0].z)
          mBounds[0].z = vertices[i].z;

        if (vertices[i].x > mBounds[1].x)
          mBounds[1].x = vertices[i].x;
        if (vertices[i].y > mBounds[1].y)
          mBounds[1].y = vertices[i].y;
        if (vertices[i].z > mBounds[1].z)
          mBounds[1].z = vertices[i].z;
      }
    }

    Aabb::Aabb(const std::vector<Aabb> &aabbs) {
      this->extend(aabbs);
    }

    void Aabb::extend(const glm::vec3 &vertex) {
      if (vertex.x < mBounds[0].x)
        mBounds[0].x = vertex.x;
      if (vertex.y < mBounds[0].y)
        mBounds[0].y = vertex.y;
      if (vertex.z < mBounds[0].z)
        mBounds[0].z = vertex.z;

      if (vertex.x > mBounds[1].x)
        mBounds[1].x = vertex.x;
      if (vertex.y > mBounds[1].y)
        mBounds[1].y = vertex.y;
      if (vertex.z > mBounds[1].z)
        mBounds[1].z = vertex.z;
    }

    void Aabb::extend(const Aabb &aabb) {
      if (aabb.mBounds[0].x < mBounds[0].x)
        mBounds[0].x = aabb.mBounds[0].x;
      if (aabb.mBounds[0].y < mBounds[0].y)
        mBounds[0].y = aabb.mBounds[0].y;
      if (aabb.mBounds[0].z < mBounds[0].z)
        mBounds[0].z = aabb.mBounds[0].z;

      if (aabb.mBounds[1].x > mBounds[1].x)
        mBounds[1].x = aabb.mBounds[1].x;
      if (aabb.mBounds[1].y > mBounds[1].y)
        mBounds[1].y = aabb.mBounds[1].y;
      if (aabb.mBounds[1].z > mBounds[1].z)
        mBounds[1].z = aabb.mBounds[1].z;
    }

    void Aabb::extend(const std::vector<Aabb> &aabbs) {
      assert(aabbs.size());

      mBounds[0] = aabbs.front().mBounds[0];
      mBounds[1] = aabbs.front().mBounds[1];
      for (size_t i = 1; i < aabbs.size(); ++i) {
        if (aabbs[i].mBounds[0].x < mBounds[0].x)
          mBounds[0].x = aabbs[i].mBounds[0].x;
        if (aabbs[i].mBounds[0].y < mBounds[0].y)
          mBounds[0].y = aabbs[i].mBounds[0].y;
        if (aabbs[i].mBounds[0].z < mBounds[0].z)
          mBounds[0].z = aabbs[i].mBounds[0].z;

        if (aabbs[i].mBounds[1].x > mBounds[1].x)
          mBounds[1].x = aabbs[i].mBounds[1].x;
        if (aabbs[i].mBounds[1].y > mBounds[1].y)
          mBounds[1].y = aabbs[i].mBounds[1].y;
        if (aabbs[i].mBounds[1].z > mBounds[1].z)
          mBounds[1].z = aabbs[i].mBounds[1].z;
      }
    }

    std::vector<glm::vec3> Aabb::vertices() const {
      std::vector<glm::vec3> vertices{
        glm::vec3(mBounds[0].x, mBounds[0].y, mBounds[0].z), glm::vec3(mBounds[0].x, mBounds[0].y, mBounds[1].z),
        glm::vec3(mBounds[0].x, mBounds[1].y, mBounds[0].z), glm::vec3(mBounds[1].x, mBounds[0].y, mBounds[0].z),
        glm::vec3(mBounds[0].x, mBounds[1].y, mBounds[1].z), glm::vec3(mBounds[1].x, mBounds[0].y, mBounds[1].z),
        glm::vec3(mBounds[1].x, mBounds[1].y, mBounds[0].z), glm::vec3(mBounds[1].x, mBounds[1].y, mBounds[1].z)};

      return vertices;
    }

    Sphere::Sphere(const glm::vec3 &center, float radius) : mCenter(center), mRadius(radius) {
    }

    Point::Point(const glm::vec3 &position) : mPosition(position) {
    }

    Ray::Ray(const glm::vec3 &origin, const glm::vec3 &direction) : mOrigin(origin), mDirection(direction) {
    }

    Box::Box(const glm::vec3 &center, const glm::vec3 &extents) : mCenter(center), mExtents(extents) {
    }

    Rectangle::Rectangle(const glm::vec3 &center, const glm::vec2 &extents) : mCenter(center), mExtents(extents) {
    }

    Capsule::Capsule(const glm::vec3 &center, float radius, float height, bool hasSide, bool hasTop, bool hasBottom) :
      mCenter(center),
      mRadius(radius),
      mHalfHeight(0.5f * height),
      mHasSide(hasSide),
      mHasTop(hasTop),
      mHasBottom(hasBottom) {
    }

    Cone::Cone(const glm::vec3 &center, float radius, float height, bool hasSide, bool hasBottom) :
      mCenter(center),
      mRadius(radius),
      mHalfHeight(0.5f * height),
      mHasSide(hasSide),
      mHasBottom(hasBottom) {
    }

    Cylinder::Cylinder(const glm::vec3 &center, float radius, float height, bool hasSide, bool hasTop, bool hasBottom) :
      mCenter(center),
      mRadius(radius),
      mHalfHeight(0.5f * height),
      mHasSide(hasSide),
      mHasTop(hasTop),
      mHasBottom(hasBottom) {
    }

    TriangleMesh::TriangleMesh(const glm::vec3 &position) : mPosition(position) {
    }

    Sphere Capsule::computeBoundingSphere() const {
      Sphere sphere(mCenter, 0.0f);

      if (!mHasTop && !mHasBottom && !mHasSide)
        return sphere;

      if (mHasTop + mHasBottom + mHasSide == 1) {
        if (mHasTop || mHasBottom) {
          const float offsetY = mHasTop ? mHalfHeight : -mHalfHeight;
          sphere.mCenter += glm::vec3(0.0f, offsetY, 0.0f);
          sphere.mRadius = mRadius;
          return sphere;
        } else {
          sphere.mRadius = glm::length(glm::vec2(mRadius, mHalfHeight));
          return sphere;
        }
      } else if (mHasTop != mHasBottom) {  // top && side || side && bottom
        const float maxY = mHasTop ? mHalfHeight + mRadius : mHalfHeight;
        const float minY = mHasBottom ? -mHalfHeight - mRadius : -mHalfHeight;
        const float totalHeight = maxY - minY;
        sphere.mRadius = (0.5f * totalHeight) + (0.5f * mRadius * mRadius) / totalHeight;
        const float offsetY = mHasTop ? (maxY - sphere.mRadius) : (minY + sphere.mRadius);
        sphere.mCenter += glm::vec3(0.0f, offsetY, 0.0f);
        return sphere;
      } else {  // complete capsule
        sphere.mRadius = mHalfHeight + mRadius;
        return sphere;
      }
    }

    Sphere Cone::computeBoundingSphere() const {
      Sphere sphere(mCenter, 0.0f);

      if (!mHasBottom && !mHasSide)
        return sphere;

      if (!mHasSide || 2.0f * mHalfHeight <= mRadius) {  // consider it as disk
        sphere.mCenter += glm::vec3(0.0f, -mHalfHeight, 0.0f);
        sphere.mRadius = mRadius;
        return sphere;
      } else {
        sphere.mRadius = mHalfHeight + (mRadius * mRadius) / (4.0f * mHalfHeight);
        sphere.mCenter += glm::vec3(0.0f, mHalfHeight - sphere.mRadius, 0.0f);
        return sphere;
      }
    }

    Sphere Cylinder::computeBoundingSphere() const {
      Sphere sphere(mCenter, 0.0f);

      if (!mHasTop && !mHasBottom && !mHasSide)
        return sphere;

      if (mHasTop + mHasBottom + mHasSide == 1 && !mHasSide) {  // just one disk
        const float offsetY = mHasTop ? mHalfHeight : -mHalfHeight;
        sphere.mCenter += glm::vec3(0.0f, offsetY, 0.0f);
        sphere.mRadius = mRadius;
        return sphere;
      } else {
        sphere.mRadius = glm::length(glm::vec2(mRadius, mHalfHeight));
        return sphere;
      }
    }

    Sphere TriangleMesh::computeBoundingSphere() const {
      return computeBoundingSphereFromVertices(mVertices);
    }

    bool isPointAbovePlane(const Plane &plane, const glm::vec3 &point) {
      const float distance =
        plane.mNormal.x * point.x + plane.mNormal.y * point.y + plane.mNormal.z * point.z + plane.mNegativeDistance;
      return distance >= 0.0f;
    }

    bool isAabbAbovePlane(const Plane &plane, const Aabb &aabb) {
      // Fast plane intersection test using p-vertex selection, http://www.txutxi.com/?p=584
      const int px = static_cast<int>(plane.mNormal.x > 0.0f);
      const int py = static_cast<int>(plane.mNormal.y > 0.0f);
      const int pz = static_cast<int>(plane.mNormal.z > 0.0f);

      const float distance = plane.mNormal.x * aabb.mBounds[px].x + plane.mNormal.y * aabb.mBounds[py].y +
                             plane.mNormal.z * aabb.mBounds[pz].z + plane.mNegativeDistance;
      return distance >= 0.0f;
    }

    bool isSphereAbovePlane(const Plane &plane, const Sphere &sphere) {
      const float distance = glm::dot(plane.mNormal, sphere.mCenter) + plane.mNegativeDistance;
      return distance >= -sphere.mRadius;
    }

    bool isPointInsideAabb(const glm::vec3 &point, const Aabb &aabb) {
      return (point.x < aabb.mBounds[0].x - glm::epsilon<float>() || point.y < aabb.mBounds[0].y - glm::epsilon<float>() ||
              point.z < aabb.mBounds[0].z - glm::epsilon<float>() || point.x > aabb.mBounds[1].x + glm::epsilon<float>() ||
              point.y > aabb.mBounds[1].y + glm::epsilon<float>() || point.z > aabb.mBounds[1].z + glm::epsilon<float>());
    }

    bool rayIntersectAabb(const glm::vec3 &origin, const glm::vec3 &direction, const Aabb &aabb, bool inverse) {
      glm::vec3 inv;
      if (inverse)
        inv = 1.0f / direction;
      else
        inv = direction;

      const float t1 = (aabb.mBounds[0].x - origin.x) * inv.x;
      const float t2 = (aabb.mBounds[1].x - origin.x) * inv.x;
      const float t3 = (aabb.mBounds[0].y - origin.y) * inv.y;
      const float t4 = (aabb.mBounds[1].y - origin.y) * inv.y;
      const float t5 = (aabb.mBounds[0].z - origin.z) * inv.z;
      const float t6 = (aabb.mBounds[1].z - origin.z) * inv.z;

      const float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));
      if (tmax < 0)
        return false;

      const float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
      return tmin <= tmax;
    }

    bool aabbCollision(const Aabb &aabb1, const Aabb &aabb2) {
      return !(glm::any(glm::lessThan(aabb1.mBounds[1], aabb2.mBounds[0])) ||
               glm::any(glm::greaterThan(aabb1.mBounds[0], aabb2.mBounds[1])));
    }

    glm::vec3 projectPointOnAabb(const glm::vec3 &point, const Aabb &aabb) {
      glm::vec3 result = point;
      for (int i = 0; i < 3; ++i) {
        if (result[i] < aabb.mBounds[0][i])
          result[i] = aabb.mBounds[0][i];
        else if (result[i] > aabb.mBounds[1][i])
          result[i] = aabb.mBounds[1][i];
      }
      return result;
    }

    float closestDistanceToPlane(const Plane &plane, const Aabb &aabb) {
      // n-vertex selection
      const int nx = static_cast<int>(plane.mNormal.x < 0.0f);
      const int ny = static_cast<int>(plane.mNormal.y < 0.0f);
      const int nz = static_cast<int>(plane.mNormal.z < 0.0f);

      // projection
      return plane.mNormal.x * aabb.mBounds[nx].x + plane.mNormal.y * aabb.mBounds[ny].y +
             plane.mNormal.z * aabb.mBounds[nz].z + plane.mNegativeDistance;
    }

    float computeRayPlaneIntersection(const Ray &ray, const Plane &plane) {
      const float denominator = glm::dot(plane.mNormal, ray.mDirection);
      const float nominator = glm::dot(plane.mNormal, ray.mOrigin);
      return -(nominator + plane.mNegativeDistance) / denominator;
    }

    Sphere computeBoundingSphereFromVertices(const std::vector<glm::vec3> &vertices) {
      // Ritter's bounding sphere approximation:
      // 1. Pick a point x from P, search a point y in P, which has the largest distance from x;
      // 2. Search a point z in P, which has the largest distance from y. set up an
      //    initial sphere B, with its centre as the midpoint of y and z, the radius as
      //    half of the distance between y and z;
      // 3. If all points in P are within sphere B, then we get a bounding sphere.
      //    Otherwise, let p be a point outside the sphere, construct a new sphere covering
      //    both point p and previous sphere. Repeat this step until all points are covered.
      // Note that steps 1. and 2. help in computing a better fitting (smaller) sphere by
      // estimating the center of the final sphere and thus reducing the bias due to the enclosed
      // vertices order.
      const glm::vec3 &p1 = vertices.front();
      glm::vec3 p2 = p1;
      float maxDistanceSquared = 0.0f;

      for (const glm::vec3 &vertex : vertices) {
        const float distanceSquared = glm::distance2(p1, vertex);
        if (distanceSquared > maxDistanceSquared) {
          maxDistanceSquared = distanceSquared;
          p2 = vertex;
        }
      }

      primitive::Sphere sphere(0.5f * (p1 + p2), 0.5f * sqrt(maxDistanceSquared));

      for (const glm::vec3 &vertex : vertices) {
        const glm::vec3 centerToVertex = vertex - sphere.mCenter;
        const float length = glm::length(centerToVertex);
        const float delta = 0.5f * (length - sphere.mRadius);
        if (delta > 0.0f) {
          const glm::vec3 direction = centerToVertex / length;
          sphere.mCenter += delta * direction;
          sphere.mRadius += delta;
        }
      }

      return sphere;
    }

    Sphere mergeBoundingSpheres(const std::vector<Sphere> &spheres) {
      assert(spheres.size());

      glm::vec3 center(0.0f);
      for (const Sphere &sphere : spheres)
        center += sphere.mCenter;

      center /= static_cast<float>(spheres.size());

      float maxDistance = 0.0f;
      for (const Sphere &sphere : spheres) {
        const float distance = glm::length(sphere.mCenter - center) + sphere.mRadius;
        if (distance > maxDistance)
          maxDistance = distance;
      }

      return Sphere(center, maxDistance);
    }

  }  // namespace primitive
}  // namespace wren
