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

#include "Node.hpp"

#include "Transform.hpp"

#include <wren/node.h>

namespace wren {

  void Node::deleteNode(Node *node) {
    if (!node)
      return;

    if (node->mParent)
      node->mParent->detachChild(node);

    delete node;
  }

  const primitive::Aabb &Node::aabb() {
    if (mIsAabbDirty) {
      recomputeAabb();
      mIsAabbDirty = false;
    }

    return mAabb;
  }

  const primitive::Sphere &Node::boundingSphere() const {
    if (mIsBoundingSphereDirty) {
      recomputeBoundingSphere();
      mIsBoundingSphereDirty = false;
    }

    return mBoundingSphere;
  }

  void Node::setBoundingVolumeDirty() const {
    if (mIsAabbDirty && mIsBoundingSphereDirty)
      return;
    mIsAabbDirty = true;
    mIsBoundingSphereDirty = true;

    if (mParent)
      mParent->setBoundingVolumeDirty();
  }

  Node::Node() : mIsVisible(true), mIsAabbDirty(true), mIsBoundingSphereDirty(true), mParent(NULL) {
  }

  Node::Node(Node *source) :
    mIsVisible(source->mIsVisible),
    mIsAabbDirty(source->mIsAabbDirty),
    mIsBoundingSphereDirty(source->mIsBoundingSphereDirty),
    mAabb(source->mAabb),
    mBoundingSphere(source->mBoundingSphere),
    mParent(source->mParent) {
  }

  void Node::recomputeAabb() const {
    mAabb = primitive::Aabb(glm::vec3(0.0f), glm::vec3(0.0f));

    if (mParent) {
      mAabb.mBounds[0] += mParent->position();
      mAabb.mBounds[1] += mParent->position();
    }
  }

  void Node::recomputeBoundingSphere() const {
    mBoundingSphere = primitive::Sphere(glm::vec3(0.0f), 0.0f);

    if (mParent)
      mBoundingSphere.mCenter += mParent->position();
  }

}  // namespace wren

// C interface implementation
void wr_node_delete(WrNode *node) {
  wren::Node::deleteNode(reinterpret_cast<wren::Node *>(node));
}

WrTransform *wr_node_get_parent(WrNode *node) {
  return reinterpret_cast<WrTransform *>(reinterpret_cast<wren::Node *>(node)->parent());
}

void wr_node_set_visible(WrNode *node, bool is_visible) {
  reinterpret_cast<wren::Node *>(node)->setVisible(is_visible);
}

bool wr_node_is_visible(WrNode *node) {
  return reinterpret_cast<wren::Node *>(node)->isVisible();
}
