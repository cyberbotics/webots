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

#ifndef TRANSFORM_NODE_HPP
#define TRANSFORM_NODE_HPP

#include "Constants.hpp"
#include "Node.hpp"

namespace wren {

  // Base class for scene tree objects that require a transformation.
  // A TransformNode keeps track of its position/scale/orientation on an absolute scale and
  // in relation to a parent Transform. Additionally, it keeps track of its local coordinate system.
  // Can be translated, rotated and scaled using the canonical axes or its local axes.
  class TransformNode : public Node {
  public:
    void setPosition(const glm::vec3 &position) {
      mPositionRelative = position;
      setMatrixDirty();
    }

    void setAbsolutePosition(const glm::vec3 &position);

    void setOrientation(const glm::quat &orientation) {
      mOrientationRelative = orientation;
      setMatrixDirty();
    }

    void setAbsoluteOrientation(const glm::quat &orientation);

    void setAbsoluteOrientation(float angle, const glm::vec3 &axis) { setAbsoluteOrientation(glm::angleAxis(angle, axis)); }

    void setPositionAndOrientation(const glm::vec3 &position, const glm::quat &orientation) {
      mPositionRelative = position;
      mOrientationRelative = orientation;
      setMatrixDirty();
    }

    void setOrientation(float angle, const glm::vec3 &axis) { setOrientation(glm::angleAxis(angle, axis)); }

    void setAbsoluteScale(const glm::vec3 &scale);

    void setScale(const glm::vec3 &scale) {
      mScaleRelative = scale;
      setMatrixDirty();
    }

    const bool isMatrixDirty() const { return mIsMatrixDirty; }

    const glm::vec3 &relativePosition() const { return mPositionRelative; }

    const glm::quat &relativeOrientation() const { return mOrientationRelative; }

    const glm::vec3 &relativeScale() const { return mScaleRelative; }

    // Calling any of these methods will recompute the transform if mIsMatrixDirty is true
    const glm::vec3 &position() const {
      TransformNode::update();
      return mPositionAbsolute;
    }

    const glm::quat &orientation() const {
      TransformNode::update();
      return mOrientationAbsolute;
    }

    const glm::vec3 &scale() const {
      TransformNode::update();
      return mScaleAbsolute;
    }

    const glm::mat4 &matrix() const {
      TransformNode::update();
      return mMatrix;
    }

    const glm::mat4 relativeMatrix() const;

    virtual void applyTranslation(const glm::vec3 &translation) {
      mPositionRelative += translation;
      mIsMatrixDirty = true;
    }

    virtual void applyRotation(const glm::quat &rotation) {
      mOrientationRelative = rotation * mOrientationRelative;
      mIsMatrixDirty = true;
    }

    virtual void applyRotation(float angle, const glm::vec3 &axis) {
      mOrientationRelative = glm::angleAxis(angle, glm::normalize(axis)) * mOrientationRelative;
      mIsMatrixDirty = true;
    }

    virtual void applyScale(const glm::vec3 &scale) {
      mScaleRelative *= scale;
      mIsMatrixDirty = true;
    }

    void setParent(Transform *parent) override;
    void setMatrixDirty() const override {
      if (mIsMatrixDirty)
        return;
      Node::setMatrixDirty();
      mIsMatrixDirty = true;
    }

    void update() const override;

  protected:
    TransformNode();
    explicit TransformNode(TransformNode *source);
    ~TransformNode() {}

  private:
    // May be modified when getting position/scale/orientation/matrix, thus mutable
    mutable bool mIsMatrixDirty;
    mutable glm::mat4 mMatrix;
    mutable glm::vec3 mPositionAbsolute;
    mutable glm::quat mOrientationAbsolute;
    mutable glm::vec3 mScaleAbsolute;

    glm::vec3 mPositionRelative;
    glm::quat mOrientationRelative;
    glm::vec3 mScaleRelative;
  };

}  // namespace wren

#endif  // TRANSFORM_NODE_HPP
