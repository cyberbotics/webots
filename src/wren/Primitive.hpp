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

#ifndef PRIMITIVE_HPP
#define PRIMITIVE_HPP

#include "Constants.hpp"

#include <array>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace wren {

  class Frustum;

  namespace primitive {

    struct Sphere;

    struct Primitive {
      virtual ~Primitive() {}
      virtual Sphere computeBoundingSphere() const = 0;
    };

    struct Sphere : public Primitive {
      explicit Sphere(const glm::vec3 &center = gVec3Zeros, float radius = 0.0f);

      Sphere computeBoundingSphere() const override { return *this; }

      glm::vec3 mCenter;
      float mRadius;
    };

    struct Plane : public Primitive {
      explicit Plane(const glm::vec3 &normal = gVec3UnitY, float negativeDistance = 0.0f);

      void normalize() {
        const float divisor = 1.0f / glm::length(mNormal);
        mNegativeDistance *= divisor;
        mNormal *= divisor;
      }

      Sphere computeBoundingSphere() const override { return gSphereInf; }

      glm::vec3 mNormal;        // correspond to a, b and c coefficients of the plane equation
      float mNegativeDistance;  // corresponds to d coefficient of the plane equation
    };

    struct Aabb : public Primitive {
      explicit Aabb(const glm::vec3 &min = gVec3Zeros, const glm::vec3 &max = gVec3Zeros);
      explicit Aabb(const std::vector<glm::vec3> &vertices);
      explicit Aabb(const std::vector<Aabb> &aabbs);

      void extend(const glm::vec3 &vertex);
      void extend(const Aabb &aabb);
      void extend(const std::vector<Aabb> &aabbs);

      std::vector<glm::vec3> vertices() const;
      Sphere computeBoundingSphere() const override {
        return Sphere(0.5f * (mBounds[1] + mBounds[0]), 0.5f * glm::length(mBounds[1] - mBounds[0]));
      }

      glm::vec3 mBounds[2];  // min. and max. vertices
    };

    struct Point : public Primitive {
      explicit Point(const glm::vec3 &position = gVec3Zeros);

      Sphere computeBoundingSphere() const override { return Sphere(mPosition, 0.0f); }

      glm::vec3 mPosition;
    };

    struct Ray : public Primitive {
      explicit Ray(const glm::vec3 &origin = gVec3Zeros, const glm::vec3 &direction = -gVec3UnitZ);

      Sphere computeBoundingSphere() const override { return gSphereInf; }

      glm::vec3 mOrigin;
      glm::vec3 mDirection;
    };

    struct Box : public Primitive {
      explicit Box(const glm::vec3 &center = gVec3Zeros, const glm::vec3 &extents = glm::vec3(0.5f));

      Sphere computeBoundingSphere() const override { return Sphere(mCenter, glm::length(mExtents)); }

      glm::vec3 mCenter;
      glm::vec3 mExtents;  // distances between center and faces
    };

    struct Rectangle : public Primitive {
      explicit Rectangle(const glm::vec3 &center = gVec3Zeros, const glm::vec2 &extents = glm::vec2(0.5f, 0.5f));

      Sphere computeBoundingSphere() const override {
        return Sphere(mCenter, glm::length(glm::vec3(mExtents.x, 0.0f, mExtents.y)));
      }

      glm::vec3 mCenter;
      glm::vec2 mExtents;  // distances between center and edges
    };

    struct Capsule : public Primitive {
      explicit Capsule(const glm::vec3 &center = gVec3Zeros, float radius = 0.5f, float height = 1.0f, bool hasSide = true,
                       bool hasTop = true, bool hasBottom = true);

      Sphere computeBoundingSphere() const override;

      glm::vec3 mCenter;
      float mRadius;
      float mHalfHeight;
      bool mHasSide;
      bool mHasTop;
      bool mHasBottom;
    };

    struct Cone : public Primitive {
      explicit Cone(const glm::vec3 &center = gVec3Zeros, float radius = 1.0f, float height = 1.0f, bool hasSide = true,
                    bool hasBottom = true);

      Sphere computeBoundingSphere() const override;

      glm::vec3 mCenter;
      float mRadius;
      float mHalfHeight;
      bool mHasSide;
      bool mHasBottom;
    };

    struct Cylinder : public Primitive {
      explicit Cylinder(const glm::vec3 &center = gVec3Zeros, float radius = 1.0f, float height = 1.0f, bool hasSide = true,
                        bool hasTop = true, bool hasBottom = true);

      Sphere computeBoundingSphere() const override;

      glm::vec3 mCenter;
      float mRadius;
      float mHalfHeight;
      bool mHasSide;
      bool mHasTop;
      bool mHasBottom;
    };

    struct TriangleMesh : public Primitive {
      explicit TriangleMesh(const glm::vec3 &position = gVec3Zeros);

      Sphere computeBoundingSphere() const override;

      glm::vec3 mPosition;

      std::vector<glm::vec3> mVertices;
      std::vector<unsigned int> mIndices;
    };

    bool isPointAbovePlane(const Plane &plane, const glm::vec3 &point);
    bool isAabbAbovePlane(const Plane &plane, const Aabb &aabb);
    bool isSphereAbovePlane(const Plane &plane, const Sphere &sphere);
    bool isPointInsideAabb(const glm::vec3 &point, const Aabb &aabb);
    bool rayIntersectAabb(const glm::vec3 &origin, const glm::vec3 &direction, const Aabb &aabb, bool inverse = true);
    bool aabbCollision(const Aabb &aabb1, const Aabb &aabb2);
    glm::vec3 projectPointOnAabb(const glm::vec3 &point, const Aabb &aabb);
    float closestDistanceToPlane(const Plane &plane, const Aabb &aabb);
    float computeRayPlaneIntersection(const Ray &ray, const Plane &plane);
    Sphere computeBoundingSphereFromVertices(const std::vector<glm::vec3> &vertices);
    Sphere mergeBoundingSpheres(const std::vector<Sphere> &spheres);

  }  // namespace primitive
}  // namespace wren

#endif  // PRIMITIVES_HPP
