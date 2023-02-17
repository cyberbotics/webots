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

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "Constants.hpp"
#include "Frustum.hpp"
#include "GlslLayout.hpp"
#include "Primitive.hpp"
#include "TransformNode.hpp"

#include <wren/camera.h>

namespace wren {

  // Represents a virtual camera, and is used by the Viewport class for rendering.
  // Inherits from Node and can be attached to a Transform for positioning.
  class Camera : public TransformNode {
  public:
    // Encapsulate memory management
    static Camera *createCamera() { return new Camera(); }

    void setProjectionMode(WrCameraProjectionMode mode) {
      mProjectionMode = mode;
      mIsProjectionDirty = true;
    }

    void setAspectRatio(float ratio) {
      mAspectRatio = ratio;
      mIsProjectionDirty = true;
    }

    void setNear(float nearDistance) {
      mNear = nearDistance;
      mIsProjectionDirty = true;
      mAabbDirty = true;
    }

    void setFar(float farDistance) {
      mFar = farDistance > 0.0f ? farDistance : 10000.0f;
      mIsProjectionDirty = true;
    }

    void setFovy(float fovy) {
      mFovy = fovy;
      mIsProjectionDirty = true;
    }

    void setHeight(float height) {
      mHalfHeight = 0.5f * height;
      mIsProjectionDirty = true;
    }

    void setWindow(float width, float height) {
      assert(mProjectionMode == WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC);
      mHalfHeight = 0.5f * height;
      mAspectRatio = width / height;
      mIsProjectionDirty = true;
    }

    void setPosition(const glm::vec3 &position) {
      TransformNode::setPosition(position);
      mIsViewDirty = true;
      mAabbDirty = true;
    }

    void setOrientation(const glm::quat &angleAxis) {
      TransformNode::setOrientation(angleAxis);
      mIsViewDirty = true;
    }

    void setFlipY(bool flipY) {
      mFlipY = flipY;
      mIsViewDirty = true;
    }

    void setDirection(const glm::vec3 &direction);
    void applyYaw(float angle) { applyRotation(angle, gVec3UnitZ); }
    void applyPitch(float angle) { applyRotation(angle, gVec3UnitY); }
    void applyRoll(float angle) { applyRotation(angle, gVec3UnitX); }

    WrCameraProjectionMode projectionMode() const { return mProjectionMode; }
    float aspectRatio() const { return mAspectRatio; }
    float nearDistance() const { return mNear; }
    float farDistance() const { return mFar; }
    float fovy() const { return mFovy; }
    float height() const { return 2.0f * mHalfHeight; }
    bool flipY() const { return mFlipY; }
    const primitive::Aabb &aabb() override;

    void updateUniforms() const;
    bool isAabbVisible(const primitive::Aabb &aabb) const;
    bool isBoundingSphereVisible(const primitive::Sphere &boundingSphere) const;

    const glm::mat4 &view() const {
      updateView();
      return mMatrices.mView;
    }

    const glm::mat4 &projection() const {
      updateProjection();
      return mMatrices.mProjection;
    }

    const Frustum &frustum() const {
      update();
      return mFrustum;
    }

    const glm::vec3 &right() const {
      updateView();
      return mRight;
    }

    const glm::vec3 &up() const {
      updateView();
      return mUp;
    }

    const glm::vec3 &forward() const {
      updateView();
      return mForward;
    }

    void applyTranslation(const glm::vec3 &translation) override {
      TransformNode::applyTranslation(translation);
      mIsViewDirty = true;
    }

    void applyRotation(const glm::quat &rotation) override {
      TransformNode::applyRotation(rotation);
      mIsViewDirty = true;
    }

    void applyRotation(float angle, const glm::vec3 &axis) override {
      TransformNode::applyRotation(angle, axis);
      mIsViewDirty = true;
    }

    void updateFromParent() override;
    void update() const override;

  private:
    Camera();
    virtual ~Camera() {}

    void updateView() const;
    void updateProjection() const;

    // May be modified when getting view/projection matrices, thus mutable
    mutable bool mIsViewDirty;
    mutable bool mIsProjectionDirty;
    mutable Frustum mFrustum;
    mutable GlslLayout::CameraTransforms mMatrices;
    mutable glm::dmat4 mDpProjectionMatrix;  // double precision matrix (used for frustum culling)
    mutable glm::vec3 mRight;
    mutable glm::vec3 mUp;
    mutable glm::vec3 mForward;

    WrCameraProjectionMode mProjectionMode;
    float mAspectRatio;
    float mNear;
    float mFar;
    float mFovy;        // only used in perpective projection mode
    float mHalfHeight;  // only used in orthographic projection mode

    bool mFlipY;  // render camera image upside down

    primitive::Aabb mNearAabb;  // AABB containing camera and its near plane
    bool mAabbDirty;
  };

}  // namespace wren

#endif  // CAMERA_HPP
