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

#ifndef SKELETON_HPP
#define SKELETON_HPP

#include "Constants.hpp"
#include "Node.hpp"

#include <unordered_map>
#include <vector>

namespace wren {
  class DynamicMesh;
  class SkeletonBone;
  class TransformNode;

  class Skeleton : public Node {
  public:
    static Skeleton *createSkeleton() { return new Skeleton(); }

    void addBone(SkeletonBone *bone) { mBones.push_back(bone); }
    void addMesh(DynamicMesh *mesh);
    void attachVertexToBone(DynamicMesh *mesh, unsigned int vertex, SkeletonBone *bone, float weight);
    void applyBindingPose();

    int getBoneCount() const { return mBones.size(); }
    SkeletonBone *getBoneByIndex(unsigned int index) { return index < mBones.size() ? mBones[index] : NULL; }
    SkeletonBone *getBoneByName(const char *name);

    glm::mat4 vertexMatrix(DynamicMesh *mesh, unsigned int vertexIndex);
    glm::mat4 matrix() const;
    void updateOffset();

    void normalizeWeights();
    void notifyDirty();

    DynamicMesh *createMeshFromBones();

    float *computeBoundingSphere(int &meshCount);

  private:
    Skeleton(){};
    virtual ~Skeleton();

    std::vector<SkeletonBone *> mBones;
    std::vector<DynamicMesh *> mMeshes;
    // Contains the list of bones for each vertex, for each mesh
    std::unordered_map<DynamicMesh *, std::vector<std::vector<size_t>>> mVertexBones;

    glm::mat4 mOffsetMatrix;
  };

}  // namespace wren

#endif  // SKELETON_HPP
