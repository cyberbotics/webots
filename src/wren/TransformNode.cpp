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

#include "TransformNode.hpp"

#include "Transform.hpp"

namespace wren {

  void TransformNode::setParent(Transform *parent) {
    if (parent == mParent)
      return;

    Node::setParent(parent);

    mIsMatrixDirty = true;
  }

  void TransformNode::setAbsolutePosition(const glm::vec3 &position) {
    if (!mParent)
      mPositionRelative = position;
    else
      mPositionRelative = (glm::conjugate(mParent->orientation()) * (position - mParent->position())) / mParent->scale();

    setMatrixDirty();
  }

  void TransformNode::setAbsoluteOrientation(const glm::quat &orientation) {
    if (!mParent)
      mOrientationRelative = orientation;
    else
      mOrientationRelative = glm::conjugate(mParent->orientation()) * orientation;

    setMatrixDirty();
  }

  void TransformNode::setAbsoluteScale(const glm::vec3 &scale) {
    if (!mParent)
      mScaleRelative = scale;
    else
      mScaleRelative = scale / mParent->scale();

    setMatrixDirty();
  }

  void TransformNode::update() const {
    if (!mIsMatrixDirty)
      return;

    if (mParent) {
      mMatrix = mParent->matrix() * relativeMatrix();
      // extract position from transform matrix
      mPositionAbsolute = glm::vec3(mMatrix[3]);
      // compute scale and orientation
      mScaleAbsolute = glm::vec3(length(mMatrix[0]), length(mMatrix[1]), length(mMatrix[2]));
      mOrientationAbsolute = glm::quat_cast(glm::mat4(mMatrix[0][0] / mScaleAbsolute.x, mMatrix[0][1] / mScaleAbsolute.y,
                                                      mMatrix[0][2] / mScaleAbsolute.z, 0.0f, mMatrix[1][0] / mScaleAbsolute.x,
                                                      mMatrix[1][1] / mScaleAbsolute.y, mMatrix[1][2] / mScaleAbsolute.z, 0.0f,
                                                      mMatrix[2][0] / mScaleAbsolute.x, mMatrix[2][1] / mScaleAbsolute.y,
                                                      mMatrix[2][2] / mScaleAbsolute.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));
    } else {
      mPositionAbsolute = mPositionRelative;
      mOrientationAbsolute = mOrientationRelative;
      mScaleAbsolute = mScaleRelative;
      mMatrix = relativeMatrix();
    }

    mIsMatrixDirty = false;
  }

  const glm::mat4 TransformNode::relativeMatrix() const {
    // matrix = translationMatrix * rotationMatrix * scaleMatrix
    glm::mat4 m = glm::mat4(mScaleRelative.x, 0.0f, 0.0f, 0.0f, 0.0f, mScaleRelative.y, 0.0f, 0.0f, 0.0f, 0.0f,
                            mScaleRelative.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);

    m = glm::mat4_cast(mOrientationRelative) * m;

    m[3][0] = mPositionRelative.x;
    m[3][1] = mPositionRelative.y;
    m[3][2] = mPositionRelative.z;

    return m;
  }

  TransformNode::TransformNode() :
    mIsMatrixDirty(false),
    mMatrix(gMat4Identity),
    mPositionAbsolute(gVec3Zeros),
    mOrientationAbsolute(),
    mScaleAbsolute(glm::vec3(1.0)),
    mPositionRelative(gVec3Zeros),
    mOrientationRelative(),
    mScaleRelative(glm::vec3(1.0)) {
  }

  TransformNode::TransformNode(TransformNode *source) :
    Node(source),
    mIsMatrixDirty(source->mIsMatrixDirty),
    mPositionRelative(source->mPositionRelative),
    mOrientationRelative(source->mOrientationRelative),
    mScaleRelative(source->mScaleRelative) {
    if (source->mIsMatrixDirty)
      source->update();

    mMatrix = source->mMatrix;
    mPositionAbsolute = source->mPositionAbsolute;
    mOrientationAbsolute = source->mOrientationAbsolute;
    mScaleAbsolute = source->mScaleAbsolute;
    mParent = NULL;
  }

}  // namespace wren
