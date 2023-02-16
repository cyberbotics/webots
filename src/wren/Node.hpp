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

#ifndef NODE_HPP
#define NODE_HPP

#include "Constants.hpp"
#include "Primitive.hpp"

namespace wren {

  class Transform;

  // Base class for objects in the scene tree. Can be attached to a Transform.
  class Node {
  public:
    // Encapsulate memory management
    static void deleteNode(Node *node);

    Transform *parent() const { return mParent; }

    // Internal, use Transform::attachChild or Transform::detachChild instead
    virtual void setParent(Transform *parent) {
      mParent = parent;
      mIsAabbDirty = mIsBoundingSphereDirty = true;
    }

    void setVisible(bool isVisible) {
      if (!mIsVisible && isVisible)
        setMatrixDirty();

      mIsVisible = isVisible;
    }

    bool isVisible() const { return mIsVisible; }

    virtual void updateFromParent() {}
    virtual void update() const {}
    virtual int computeChildCount() const { return 0; }
    virtual const primitive::Aabb &aabb();
    virtual const primitive::Sphere &boundingSphere() const;
    virtual void setMatrixDirty() const { setBoundingVolumeDirty(); }
    virtual void setBoundingVolumeDirty() const;

  protected:
    Node();
    explicit Node(Node *source);
    virtual ~Node() {}

    virtual void recomputeAabb() const;
    virtual void recomputeBoundingSphere() const;

    bool mIsVisible;
    mutable bool mIsAabbDirty;
    mutable bool mIsBoundingSphereDirty;
    mutable primitive::Aabb mAabb;
    mutable primitive::Sphere mBoundingSphere;

    Transform *mParent;
  };

}  // namespace wren

#endif  // NODE_HPP
