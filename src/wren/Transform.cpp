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

#include "Transform.hpp"

#include "Camera.hpp"
#include "ContainerUtils.hpp"
#include "Debug.hpp"
#include "Scene.hpp"
#include "Viewport.hpp"

#include <wren/transform.h>

#include <algorithm>

namespace wren {

  void Transform::attachChild(Node *child) {
    assert(std::find(mChildren.cbegin(), mChildren.cend(), child) == mChildren.cend());

    if (child->parent())
      child->parent()->detachChild(child);

    mChildren.push_back(child);

    child->setParent(this);

    setBoundingVolumeDirty();
  }

  void Transform::detachChild(Node *child) {
    assert(child->parent() == this);

    containerutils::removeElementFromVector(mChildren, child);

    child->setParent(NULL);
  }

  int Transform::computeChildCount() const {
    int count = mChildren.size();
    for (Node *n : mChildren)
      count += n->computeChildCount();

    return count;
  }

  Transform::Transform() {
  }

  Transform::Transform(Transform *source) : TransformNode(source) {
  }

  Transform::~Transform() {
    for (Node *child : mChildren)
      child->setParent(NULL);
  }

}  // namespace wren

// C interface implementation
WrTransform *wr_transform_new() {
  return reinterpret_cast<WrTransform *>(wren::Transform::createTransform());
}

WrTransform *wr_transform_copy(WrTransform *transform) {
  return reinterpret_cast<WrTransform *>(wren::Transform::copyTransform(reinterpret_cast<wren::Transform *>(transform)));
}

const float *wr_transform_get_matrix(WrTransform *transform) {
  return glm::value_ptr(reinterpret_cast<wren::TransformNode *>(transform)->matrix());
}

void wr_transform_attach_child(WrTransform *transform, WrNode *child) {
  reinterpret_cast<wren::Transform *>(transform)->attachChild(reinterpret_cast<wren::Node *>(child));
}

void wr_transform_detach_child(WrTransform *transform, WrNode *child) {
  reinterpret_cast<wren::Transform *>(transform)->detachChild(reinterpret_cast<wren::Node *>(child));
}

void wr_transform_set_position(WrTransform *transform, const float *position) {
  reinterpret_cast<wren::TransformNode *>(transform)->setPosition(glm::make_vec3(position));
}

void wr_transform_set_absolute_position(WrTransform *transform, const float *position) {
  reinterpret_cast<wren::TransformNode *>(transform)->setAbsolutePosition(glm::make_vec3(position));
}

void wr_transform_set_orientation(WrTransform *transform, const float *angle_axis) {
  reinterpret_cast<wren::TransformNode *>(transform)->setOrientation(
    glm::angleAxis(angle_axis[0], glm::make_vec3(&angle_axis[1])));
}

void wr_transform_set_absolute_orientation(WrTransform *transform, const float *angle_axis) {
  reinterpret_cast<wren::TransformNode *>(transform)->setAbsoluteOrientation(
    glm::angleAxis(angle_axis[0], glm::make_vec3(&angle_axis[1])));
}

void wr_transform_set_scale(WrTransform *transform, const float *scale) {
  reinterpret_cast<wren::TransformNode *>(transform)->setScale(glm::make_vec3(scale));
}

void wr_transform_set_position_and_orientation(WrTransform *transform, const float *position, const float *angle_axis) {
  reinterpret_cast<wren::Transform *>(transform)->setPositionAndOrientation(
    glm::make_vec3(position), glm::angleAxis(angle_axis[0], glm::make_vec3(&angle_axis[1])));
}
