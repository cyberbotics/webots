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

#ifndef SKELETON_BONE_HPP
#define SKELETON_BONE_HPP

#include "DynamicMesh.hpp"
#include "Transform.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace wren {

  class Skeleton;

  class SkeletonBone : public Transform {
  public:
    static SkeletonBone *createSkeletonBone(Skeleton *skeleton, const char *name) { return new SkeletonBone(skeleton, name); }

    const char *name() const { return mName.c_str(); }

    void setVertexWeight(DynamicMesh *mesh, unsigned int index, float weight) { mWeights[mesh][index] = weight; }

    float vertexWeight(DynamicMesh *mesh, unsigned int index) { return mWeights[mesh][index]; }

    void setMatrixDirty() const override;

    void applyBindingPose() {
      mBoundMatrix = glm::inverse(matrix());
      mFinalTransformDirty = true;
    }

    const glm::mat4 finalTransform() {
      if (mFinalTransformDirty)
        mFinalTransform = matrix() * mBoundMatrix;

      return mFinalTransform;
    }

  private:
    SkeletonBone(Skeleton *skeleton, const char *name);
    virtual ~SkeletonBone() {}

    std::string mName;
    std::unordered_map<DynamicMesh *, std::unordered_map<unsigned int, float>> mWeights;
    Skeleton *mSkeleton;

    glm::mat4 mBoundMatrix;
    glm::mat4 mFinalTransform;

    mutable bool mFinalTransformDirty;
  };

}  // namespace wren

#endif  // SKELETON_BONE_HPP
