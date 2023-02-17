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

#include "Skeleton.hpp"

#include "Debug.hpp"
#include "DynamicMesh.hpp"
#include "SkeletonBone.hpp"

#include <wren/skeleton.h>
#include <string>

namespace wren {
  Skeleton::~Skeleton() {
    for (SkeletonBone *bone : mBones)
      Node::deleteNode(bone);
  }

  void Skeleton::addMesh(DynamicMesh *mesh) {
    mMeshes.push_back(mesh);

    mVertexBones[mesh].assign(mesh->coords().size(), std::vector<size_t>());

    mesh->setSkeleton(this);
  }

  void Skeleton::attachVertexToBone(DynamicMesh *mesh, unsigned int vertex, SkeletonBone *bone, float weight) {
    for (size_t i = 0; i < mBones.size(); ++i) {
      if (mBones[i] == bone) {
        mVertexBones[mesh][vertex].push_back(i);
        bone->setVertexWeight(mesh, vertex, weight);
        return;
      }
    }
  }

  glm::mat4 Skeleton::vertexMatrix(DynamicMesh *mesh, unsigned int vertexIndex) {
    glm::mat4 m(0.0f);
    for (unsigned int boneIndex : mVertexBones[mesh][vertexIndex]) {
      SkeletonBone *bone = mBones[boneIndex];
      const float weight = bone->vertexWeight(mesh, vertexIndex);
      m += weight * bone->finalTransform();
    }
    return m;
  }

  glm::mat4 Skeleton::matrix() const {
    if (parent())
      return parent()->matrix();
    else
      return glm::mat4(1.0f);
  }

  void Skeleton::updateOffset() {
    if (parent()) {
      for (DynamicMesh *mesh : mMeshes)
        mesh->updateSkeletonOffset(mOffsetMatrix * parent()->relativeMatrix());
    }
  }

  SkeletonBone *Skeleton::getBoneByName(const char *name) {
    for (SkeletonBone *bone : mBones) {
      if (std::string(name) == bone->name())
        return bone;
    }
    return NULL;
  }

  void Skeleton::normalizeWeights() {
    for (DynamicMesh *mesh : mMeshes) {
      for (size_t i = 0; i < mesh->coords().size(); ++i) {
        float totalWeight = 0.0f;
        for (unsigned int boneIndex : mVertexBones[mesh][i])
          totalWeight += mBones[boneIndex]->vertexWeight(mesh, i);

        assert(totalWeight > 0.0f);
        for (unsigned int boneIndex : mVertexBones[mesh][i]) {
          const float boneWeight = mBones[boneIndex]->vertexWeight(mesh, i);
          mBones[boneIndex]->setVertexWeight(mesh, i, boneWeight / totalWeight);
        }
      }
    }
  }

  void Skeleton::applyBindingPose() {
    for (DynamicMesh *mesh : mMeshes)
      mesh->applySkeletonOffset();

    for (SkeletonBone *bone : mBones)
      bone->applyBindingPose();

    if (parent())
      mOffsetMatrix = matrix() * glm::inverse(parent()->relativeMatrix());
  }

  void Skeleton::notifyDirty() {
    for (DynamicMesh *mesh : mMeshes)
      mesh->notifySkeletonDirty();
  }

  float *Skeleton::computeBoundingSphere(int &meshCount) {
    meshCount = mMeshes.size();
    if (meshCount == 0)
      return NULL;
    float *result = new float[4 * meshCount];
    int index = 0;
    for (DynamicMesh *mesh : mMeshes) {
      const primitive::Sphere &bs = mesh->recomputeBoundingSphere();
      result[index] = bs.mCenter.x;
      result[index + 1] = bs.mCenter.y;
      result[index + 2] = bs.mCenter.z;
      result[index + 3] = bs.mRadius;
      index += 4;
    }
    return result;
  }
}  // namespace wren

WrSkeleton *wr_skeleton_new() {
  return reinterpret_cast<WrSkeleton *>(wren::Skeleton::createSkeleton());
}

int wr_skeleton_get_bone_count(WrSkeleton *skeleton) {
  return reinterpret_cast<wren::Skeleton *>(skeleton)->getBoneCount();
}

WrSkeletonBone *wr_skeleton_get_bone_by_index(WrSkeleton *skeleton, int index) {
  return reinterpret_cast<WrSkeletonBone *>(reinterpret_cast<wren::Skeleton *>(skeleton)->getBoneByIndex(index));
}

WrSkeletonBone *wr_skeleton_get_bone_by_name(WrSkeleton *skeleton, const char *name) {
  return reinterpret_cast<WrSkeletonBone *>(reinterpret_cast<wren::Skeleton *>(skeleton)->getBoneByName(name));
}

void wr_skeleton_apply_binding_pose(WrSkeleton *skeleton) {
  reinterpret_cast<wren::Skeleton *>(skeleton)->applyBindingPose();
}

void wr_skeleton_update_offset(WrSkeleton *skeleton) {
  reinterpret_cast<wren::Skeleton *>(skeleton)->updateOffset();
}

float *wr_skeleton_compute_bounding_spheres(WrSkeleton *skeleton, int &count) {
  return reinterpret_cast<wren::Skeleton *>(skeleton)->computeBoundingSphere(count);
}
