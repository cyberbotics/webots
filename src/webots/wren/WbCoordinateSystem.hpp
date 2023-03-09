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

#ifndef WB_COORDINATE_SYSTEM_HPP
#define WB_COORDINATE_SYSTEM_HPP

#include <QtCore/QObject>

#include <wren/drawable_texture.h>
#include <wren/font.h>
#include <wren/material.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

class WbWrenRenderingContext;
class WbQuaternion;

class WbCoordinateSystem : public QObject {
  Q_OBJECT

public:
  explicit WbCoordinateSystem(WbWrenRenderingContext *context);
  ~WbCoordinateSystem();

  void setVisible(bool b);
  void setOrientation(const WbQuaternion &quaternion);

private:
  void deleteWrenObjects();

  WrFont *mFont;
  WrTransform *mTransform;
  WrTransform *mLabelsTransform[3];
  WrRenderable *mRenderables[3][2];
  WrStaticMesh *mAxesMesh[3];
  WrStaticMesh *mLabelsMesh;
  WrMaterial *mAxesMaterial[3];
  WrMaterial *mLabelsMaterial[3];
  WrDrawableTexture *mLabelsTexture[3];
};

#endif  // WB_COORDINATE_SYSTEM_HPP
