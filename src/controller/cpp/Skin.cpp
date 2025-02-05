// Copyright 1996-2024 Cyberbotics Ltd.
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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/skin.h>
#include <webots/Skin.hpp>

using namespace std;
using namespace webots;

void Skin::setBoneOrientation(int index, const double *orientation, bool absolute) {
  wb_skin_set_bone_orientation(getTag(), index, orientation, absolute);
}

void Skin::setBonePosition(int index, const double *position, bool absolute) {
  wb_skin_set_bone_position(getTag(), index, position, absolute);
}

int Skin::getBoneCount() const {
  return wb_skin_get_bone_count(getTag());
}

const string Skin::getBoneName(int index) const {
  const char *name_str = wb_skin_get_bone_name(getTag(), index);
  const string name(name_str);
  return name;
}

const double *Skin::getBoneOrientation(int index, bool absolute) const {
  return wb_skin_get_bone_orientation(getTag(), index, absolute);
}

const double *Skin::getBonePosition(int index, bool absolute) const {
  return wb_skin_get_bone_position(getTag(), index, absolute);
}
