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

#ifndef POINT_LIGHT_HPP
#define POINT_LIGHT_HPP

#include "Constants.hpp"
#include "PositionalLight.hpp"

namespace wren {

  // Represents a light source which radiates isotropically.
  class PointLight : public PositionalLight {
  public:
    // Encapsulate memory management
    static PointLight *createPointLight() { return new PointLight(); }

    LightNode::Type type() override { return TYPE_POINT; }

  private:
    PointLight();
    virtual ~PointLight();
  };

}  // namespace wren

#endif  // POINT_LIGHT_HPP
