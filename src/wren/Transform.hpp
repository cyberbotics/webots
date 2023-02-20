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

#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <vector>

#include "TransformNode.hpp"

namespace wren {

  // Extends TransformNode by additionally being able to contain a list of child Nodes.
  // During traversal of the scene tree, will update its transform if necessary and propagate changes to its children.
  class Transform : public TransformNode {
  public:
    // Encapsulate memory management
    static Transform *createTransform() { return new Transform(); }
    static Transform *copyTransform(Transform *source) { return new Transform(source); }

    void attachChild(Node *child);
    void detachChild(Node *child);

    const std::vector<Node *> &children() const { return mChildren; }

    // Updates transforms and propagates to children
    void updateFromParent() override {
      if (isVisible()) {
        for (Node *n : mChildren)
          n->updateFromParent();
      }
    }

    int computeChildCount() const override;
    void setMatrixDirty() const override {
      TransformNode::setMatrixDirty();
      for (Node *n : mChildren)
        n->setMatrixDirty();
    }

  protected:
    Transform();
    explicit Transform(Transform *source);
    virtual ~Transform();

  private:
    void recomputeAabb() const override {
      if (mChildren.size()) {
        std::vector<primitive::Aabb> aabbs;
        aabbs.reserve(mChildren.size());
        for (Node *child : mChildren)
          aabbs.push_back(child->aabb());

        mAabb = primitive::Aabb(aabbs);
      } else
        mAabb = primitive::Aabb(position(), position());
    }

    void recomputeBoundingSphere() const override {
      if (mChildren.size()) {
        std::vector<primitive::Sphere> spheres;
        spheres.reserve(mChildren.size());
        for (Node *child : mChildren)
          spheres.push_back(child->boundingSphere());

        mBoundingSphere = primitive::mergeBoundingSpheres(spheres);
      } else
        mBoundingSphere = primitive::Sphere(position(), 0.0f);
    }

    std::vector<Node *> mChildren;
  };

}  // namespace wren

#endif  // TRANSFORM_HPP
