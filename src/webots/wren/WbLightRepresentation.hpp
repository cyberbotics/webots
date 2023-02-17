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

#ifndef WB_LIGHT_REPRESENTATION_HPP
#define WB_LIGHT_REPRESENTATION_HPP

//
// Description: class handling the optional rendering "Show lights position" (used by PointLight & SpotLight)
//
#include "WbVector3.hpp"

class QImage;

struct WrTransform;
struct WrTexture2d;
struct WrStaticMesh;
struct WrRenderable;
struct WrMaterial;

class WbLightRepresentation {
public:
  // Called when window in resized
  static void updateScreenScale(int width, int height);

  // Constructor and destructor
  WbLightRepresentation(WrTransform *parent, const WbVector3 &position);
  virtual ~WbLightRepresentation();

  // Setters
  virtual void setPosition(const WbVector3 &position);
  virtual void setVisible(bool visible);

protected:
  WbVector3 mPosition;

private:
  WbLightRepresentation(const WbLightRepresentation &);  // non construction-copyable
  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrMaterial *mMaterial;
  WrStaticMesh *mMesh;
  WrTexture2d *mTexture;

  QImage *mQImage;
};

#endif
