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

#include "Frustum.hpp"

#include "Camera.hpp"
#include "Debug.hpp"

namespace wren {

  Frustum::Frustum(const glm::mat4 &matrix) {
    recomputeFromMatrix(matrix);
  }

  void Frustum::recomputeFromMatrix(const glm::dmat4 &matrix) {
    // compute frustum in view space using the Gribb-Hartmann method (normals pointing inside)
    mPlanes[FRUSTUM_PLANE_LEFT] =
      primitive::Plane(glm::vec3(matrix[0][3] + matrix[0][0], matrix[1][3] + matrix[1][0], matrix[2][3] + matrix[2][0]),
                       matrix[3][3] + matrix[3][0]);

    mPlanes[FRUSTUM_PLANE_RIGHT] =
      primitive::Plane(glm::vec3(matrix[0][3] - matrix[0][0], matrix[1][3] - matrix[1][0], matrix[2][3] - matrix[2][0]),
                       matrix[3][3] - matrix[3][0]);

    mPlanes[FRUSTUM_PLANE_BOTTOM] =
      primitive::Plane(glm::vec3(matrix[0][3] + matrix[0][1], matrix[1][3] + matrix[1][1], matrix[2][3] + matrix[2][1]),
                       matrix[3][3] + matrix[3][1]);

    mPlanes[FRUSTUM_PLANE_TOP] =
      primitive::Plane(glm::vec3(matrix[0][3] - matrix[0][1], matrix[1][3] - matrix[1][1], matrix[2][3] - matrix[2][1]),
                       matrix[3][3] - matrix[3][1]);

    mPlanes[FRUSTUM_PLANE_NEAR] =
      primitive::Plane(glm::vec3(matrix[0][3] + matrix[0][2], matrix[1][3] + matrix[1][2], matrix[2][3] + matrix[2][2]),
                       matrix[3][3] + matrix[3][2]);

    mPlanes[FRUSTUM_PLANE_FAR] =
      primitive::Plane(glm::vec3(matrix[0][3] - matrix[0][2], matrix[1][3] - matrix[1][2], matrix[2][3] - matrix[2][2]),
                       matrix[3][3] - matrix[3][2]);

    for (primitive::Plane &planePrimitive : mPlanes)
      planePrimitive.normalize();

    computeCornersFromInverseMatrix(glm::inverse(matrix));
    computeAabb();
  }

  void Frustum::computeCornersFromInverseMatrix(const glm::mat4 &matrix) {
    mCorners = {glm::vec3(-1.0f, 1.0f, -1.0f), glm::vec3(1.0f, 1.0f, -1.0f), glm::vec3(-1.0f, -1.0f, -1.0f),
                glm::vec3(1.0f, -1.0f, -1.0f), glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f),
                glm::vec3(-1.0f, -1.0f, 1.0f), glm::vec3(1.0f, -1.0f, 1.0f)};

    glm::vec4 tmp;
    for (glm::vec3 &point : mCorners) {
      tmp = matrix * glm::vec4(point, 1.0f);
      point = glm::vec3(tmp.x / tmp.w, tmp.y / tmp.w, tmp.z / tmp.w);
    }
  }

  bool Frustum::isInside(const primitive::Aabb &aabb) const {
    for (const primitive::Plane &planePrimitive : mPlanes) {
      if (!primitive::isAabbAbovePlane(planePrimitive, aabb))
        return false;
    }

    return true;
  }

  bool Frustum::isInside(const primitive::Sphere &sphere) const {
    for (const primitive::Plane &planePrimitive : mPlanes) {
      if (!primitive::isSphereAbovePlane(planePrimitive, sphere))
        return false;
    }

    return true;
  }

}  // namespace wren
