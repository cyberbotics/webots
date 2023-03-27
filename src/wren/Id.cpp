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

#include "Id.hpp"

namespace wren {

  // The id 0 is reserved (see glstate::cActiveMaterial)
  uint16_t IdPhongMaterial::cCounter = 1;
  uint16_t IdPbrMaterial::cCounter = 1;
  uint16_t IdMesh::cCounter = 1;

  Id::Id(uint16_t id) : mId(id) {
  }

  IdPhongMaterial::IdPhongMaterial() : Id(IdPhongMaterial::cCounter++) {
  }
  IdPbrMaterial::IdPbrMaterial() : Id(IdPbrMaterial::cCounter++) {
  }

  IdMesh::IdMesh() : Id(IdMesh::cCounter++) {
  }

}  // namespace wren
