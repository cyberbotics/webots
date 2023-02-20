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

#ifndef FRUSTUM_HPP
#define FRUSTUM_HPP

#include "Constants.hpp"
#include "Primitive.hpp"

#include <vector>

namespace wren {

  class Camera;

  class Frustum {
  public:
    enum FrustumPlane {
      FRUSTUM_PLANE_LEFT,
      FRUSTUM_PLANE_RIGHT,
      FRUSTUM_PLANE_BOTTOM,
      FRUSTUM_PLANE_TOP,
      FRUSTUM_PLANE_NEAR,
      FRUSTUM_PLANE_FAR
    };

    enum FrustumCorner {
      FRUSTUM_CORNER_NEAR_TOP_LEFT,
      FRUSTUM_CORNER_NEAR_TOP_RIGHT,
      FRUSTUM_CORNER_NEAR_BOTTOM_LEFT,
      FRUSTUM_CORNER_NEAR_BOTTOM_RIGHT,
      FRUSTUM_CORNER_FAR_TOP_LEFT,
      FRUSTUM_CORNER_FAR_TOP_RIGHT,
      FRUSTUM_CORNER_FAR_BOTTOM_LEFT,
      FRUSTUM_CORNER_FAR_BOTTOM_RIGHT,
      FRUSTUM_CORNER_COUNT
    };

    Frustum() = default;
    explicit Frustum(const glm::mat4 &matrix);

    const primitive::Plane &plane(FrustumPlane plane) const { return mPlanes[plane]; }
    const std::vector<glm::vec3> &corners() const { return mCorners; }
    const primitive::Aabb &aabb() const { return mAabb; }

    void recomputeFromMatrix(const glm::dmat4 &matrix);
    bool isInside(const primitive::Aabb &aabb) const;
    bool isInside(const primitive::Sphere &sphere) const;

  private:
    void computeCornersFromInverseMatrix(const glm::mat4 &matrix);
    void computeAabb() { mAabb = primitive::Aabb(mCorners); }

    std::array<primitive::Plane, 6> mPlanes;  // left, right, bottom, top, near, far
    std::vector<glm::vec3> mCorners;          // near to far, top to bottom, left to right
    primitive::Aabb mAabb;
  };

}  // namespace wren

#endif  // FRUSTUM_HPP
