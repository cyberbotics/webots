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

#include "SkeletonBone.hpp"

#include "Debug.hpp"
#include "Skeleton.hpp"

#include <wren/skeleton_bone.h>

namespace wren {
  SkeletonBone::SkeletonBone(Skeleton *skeleton, const char *name) :
    mName(name),
    mSkeleton(skeleton),
    mFinalTransformDirty(true) {
    mSkeleton->addBone(this);
  }

  void SkeletonBone::setMatrixDirty() const {
    Transform::setMatrixDirty();
    mSkeleton->notifyDirty();
    mFinalTransformDirty = true;
  }
}  // namespace wren

const char *wr_skeleton_bone_get_name(WrSkeletonBone *bone) {
  return reinterpret_cast<wren::SkeletonBone *>(bone)->name();
}

void wr_skeleton_bone_get_position(const WrSkeletonBone *bone, bool absolute, float *outputPosition) {
  glm::vec3 position;
  if (absolute)
    position = reinterpret_cast<const wren::SkeletonBone *>(bone)->position();
  else
    position = reinterpret_cast<const wren::SkeletonBone *>(bone)->relativePosition();

  outputPosition[0] = position[0];
  outputPosition[1] = position[1];
  outputPosition[2] = position[2];
}

void wr_skeleton_bone_get_orientation(const WrSkeletonBone *bone, bool absolute, float *outputOrientation) {
  glm::quat orientation;
  if (absolute)
    orientation = reinterpret_cast<const wren::SkeletonBone *>(bone)->orientation();
  else
    orientation = reinterpret_cast<const wren::SkeletonBone *>(bone)->relativeOrientation();

  outputOrientation[0] = glm::angle(orientation);
  glm::vec3 axis = glm::axis(orientation);
  outputOrientation[1] = axis[0];
  outputOrientation[2] = axis[1];
  outputOrientation[3] = axis[2];
}
