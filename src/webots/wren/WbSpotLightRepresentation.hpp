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

#ifndef WB_SPOTLIGHT_REPRESENTATION_HPP
#define WB_SPOTLIGHT_REPRESENTATION_HPP

//
// Description: class that extends WbLightRepresentation to draw spot cone
//
#include "WbLightRepresentation.hpp"
#include "WbVector3.hpp"

struct WrTransform;
struct WrStaticMesh;
struct WrRenderable;
struct WrMaterial;

class WbSpotLightRepresentation : public WbLightRepresentation {
public:
  // Constructor and destructor
  WbSpotLightRepresentation(WrTransform *parent, const WbVector3 &position, float radius, float cutOffAngle,
                            const WbVector3 &direction);
  ~WbSpotLightRepresentation();

  // Setters
  void setRadius(float radius) {
    mRadius = radius;
    updateMesh();
  }
  void setCutOffAngle(float cutOffAngle) {
    mCutOffAngle = cutOffAngle;
    updateMesh();
  }
  void setDirection(const WbVector3 &direction) {
    mDirection = direction.normalized();
    updateMesh();
  }

  void setVisible(bool visible) override;
  void setPosition(const WbVector3 &position) override;

private:
  void updateMesh();

  float mRadius;
  float mCutOffAngle;
  WbVector3 mDirection;

  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrMaterial *mMaterial;
  WrStaticMesh *mMesh;
};

#endif
