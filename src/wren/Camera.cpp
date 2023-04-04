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

#include "Camera.hpp"

#include "Debug.hpp"
#include "GlState.hpp"
#include "Transform.hpp"
#include "UniformBuffer.hpp"

#include <wren/camera.h>

#include <cmath>

namespace wren {
  static primitive::Aabb computeAabb(const glm::vec3 &position, float nearDistance) {
    const glm::vec3 nearVector(nearDistance);
    return primitive::Aabb(position - nearVector, position + nearVector);
  }

  void Camera::setDirection(const glm::vec3 &direction) {
    assert(glm::length(direction) > glm::epsilon<float>());

    const glm::vec3 orig = orientation() * -gVec3UnitZ;
    const glm::vec3 d = glm::normalize(direction);

    glm::quat rotation = glm::rotation(orig, d);
    applyRotation(rotation);
  }

  const primitive::Aabb &Camera::aabb() {
    if (mAabbDirty) {
      mNearAabb = computeAabb(TransformNode::position(), mNear);
      mAabbDirty = false;
    }
    return mNearAabb;
  }

  void Camera::updateUniforms() const {
    update();
    glstate::uniformBuffer(WR_GLSL_LAYOUT_UNIFORM_BUFFER_CAMERA_TRANSFORMS)->writeValue(&mMatrices);
  }

  bool Camera::isAabbVisible(const primitive::Aabb &aabb) const {
    return mFrustum.isInside(aabb);
  }

  bool Camera::isBoundingSphereVisible(const primitive::Sphere &boundingSphere) const {
    return mFrustum.isInside(boundingSphere);
  }

  void Camera::updateFromParent() {
    if (mParent->isMatrixDirty()) {
      TransformNode::updateFromParent();
      mIsViewDirty = true;
    }
  }

  void Camera::update() const {
    const bool updateFrustum = mIsViewDirty || mIsProjectionDirty;

    updateView();
    updateProjection();

    if (updateFrustum) {
      const glm::dmat4 dpView = mMatrices.mView;
      mFrustum.recomputeFromMatrix(mDpProjectionMatrix * dpView);
    }
  }

  Camera::Camera() :
    mIsViewDirty(true),
    mIsProjectionDirty(true),
    mProjectionMode(WR_CAMERA_PROJECTION_MODE_PERSPECTIVE),
    mAspectRatio(1.0f),
    mNear(0.05f),
    mFar(100.0f),
    mFovy(glm::quarter_pi<float>()),
    mHalfHeight(0.5f),
    mFlipY(false),
    mAabbDirty(false) {
    mNearAabb = computeAabb(TransformNode::position(), mNear);
  }

  void Camera::updateView() const {
    if (!mIsViewDirty)
      return;

    mUp = orientation() * gVec3UnitY;
    mForward = orientation() * -gVec3UnitZ;
    mRight = glm::cross(mForward, mUp);

    mMatrices.mView = glm::lookAt(position(), position() + mForward, mUp);

    mIsViewDirty = false;
  }

  static glm::mat4 infinitePerspective(float fovy, float aspectRatio, float near) {
    const float tanHalfFovy = tanf(fovy / 2.0f);

    glm::mat4 projection = glm::mat4(0.0f);
    projection[0][0] = 1.0f / (aspectRatio * tanHalfFovy);
    projection[1][1] = 1.0f / tanHalfFovy;
    projection[2][2] = -1.0f;
    projection[2][3] = -1.0f;
    projection[3][2] = -near;

    return projection;
  }

  void Camera::updateProjection() const {
    if (!mIsProjectionDirty)
      return;

    if (mProjectionMode == WR_CAMERA_PROJECTION_MODE_PERSPECTIVE) {
      mDpProjectionMatrix =
        glm::perspective(static_cast<double>(mFovy), static_cast<double>(mAspectRatio), static_cast<double>(mNear),
                         static_cast<double>(mFar));  // used for frustum culling & depth rendering
      mMatrices.mProjection = mDpProjectionMatrix;
      mMatrices.mInfiniteProjection = infinitePerspective(mFovy, mAspectRatio, mNear);  // used for rendering
    } else {
      mMatrices.mProjection =
        glm::ortho(-mHalfHeight * mAspectRatio, mHalfHeight * mAspectRatio, -mHalfHeight, mHalfHeight, mNear, mFar);
      mMatrices.mInfiniteProjection = mMatrices.mProjection;
    }
    if (mFlipY) {
      mMatrices.mProjection = glm::scale(mMatrices.mProjection, glm::vec3(1.0f, -1.0f, 1.0f));
      mMatrices.mInfiniteProjection = glm::scale(mMatrices.mInfiniteProjection, glm::vec3(1.0f, -1.0f, 1.0f));
    }

    mIsProjectionDirty = false;
  }

}  // namespace wren

// C interface
WrCamera *wr_camera_new() {
  return reinterpret_cast<WrCamera *>(wren::Camera::createCamera());
}

void wr_camera_set_projection_mode(WrCamera *camera, WrCameraProjectionMode mode) {
  reinterpret_cast<wren::Camera *>(camera)->setProjectionMode(mode);
}

void wr_camera_set_aspect_ratio(WrCamera *camera, float ratio) {
  reinterpret_cast<wren::Camera *>(camera)->setAspectRatio(ratio);
}

void wr_camera_set_near(WrCamera *camera, float near_distance) {
  reinterpret_cast<wren::Camera *>(camera)->setNear(near_distance);
}

void wr_camera_set_far(WrCamera *camera, float far_distance) {
  reinterpret_cast<wren::Camera *>(camera)->setFar(far_distance);
}

void wr_camera_set_fovy(WrCamera *camera, float fovy) {
  reinterpret_cast<wren::Camera *>(camera)->setFovy(fovy);
}

void wr_camera_set_height(WrCamera *camera, float height) {
  reinterpret_cast<wren::Camera *>(camera)->setHeight(height);
}

void wr_camera_set_position(WrCamera *camera, float *position) {
  reinterpret_cast<wren::Camera *>(camera)->setPosition(glm::make_vec3(position));
}

void wr_camera_set_orientation(WrCamera *camera, float *angle_axis) {
  reinterpret_cast<wren::Camera *>(camera)->setOrientation(glm::angleAxis(angle_axis[0], glm::make_vec3(&angle_axis[1])));
}

void wr_camera_set_flip_y(WrCamera *camera, bool flip_y) {
  return reinterpret_cast<wren::Camera *>(camera)->setFlipY(flip_y);
}

void wr_camera_apply_yaw(WrCamera *camera, float angle) {
  reinterpret_cast<wren::Camera *>(camera)->applyYaw(angle);
}

void wr_camera_apply_pitch(WrCamera *camera, float angle) {
  reinterpret_cast<wren::Camera *>(camera)->applyPitch(angle);
}

void wr_camera_apply_roll(WrCamera *camera, float angle) {
  reinterpret_cast<wren::Camera *>(camera)->applyRoll(angle);
}

float wr_camera_get_near(WrCamera *camera) {
  return reinterpret_cast<wren::Camera *>(camera)->nearDistance();
}

float wr_camera_get_far(WrCamera *camera) {
  return reinterpret_cast<wren::Camera *>(camera)->farDistance();
}

float wr_camera_get_fovy(WrCamera *camera) {
  return reinterpret_cast<wren::Camera *>(camera)->fovy();
}

float wr_camera_get_height(WrCamera *camera) {
  return reinterpret_cast<wren::Camera *>(camera)->height();
}
