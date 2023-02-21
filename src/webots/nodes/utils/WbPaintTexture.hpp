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

#ifndef WB_PAINT_TEXTURE_HPP
#define WB_PAINT_TEXTURE_HPP

#include <QtCore/QObject>

#include "WbVector2.hpp"

struct WrDrawableTexture;

class WbRay;
class WbRgb;
class WbShape;

// this class is used to simulate texture painting with the Pen device and ink evaporation
class WbPaintTexture : public QObject {
  Q_OBJECT

public:
  explicit WbPaintTexture(const WbShape *shape);
  ~WbPaintTexture();

  void paint(const WbRay &ray, float leadSize, const WbRgb &color, float density);

  const WbShape *shape() { return mShape; }

  static bool isPaintable(const WbShape *shape);

  void pickColor(const WbVector2 &uv, WbRgb &pickedColor, float *pickedDensity = NULL) const;
  void clearTexture();

  // simulate ink evaporation
  static void prePhysicsStep(double ms);

  // get paint texture, create if not found
  static WbPaintTexture *paintTexture(const WbShape *shape);

  // find existing paint texture, return NULL if not found
  static WbPaintTexture *findPaintTexture(const WbShape *shape);

  // initialize list of paint textures
  static void init();

  // free all paint textures and associated memory
  static void cleanup();

  static void clearAllTextures();

private:
  const WbShape *mShape;
  WrDrawableTexture *mTexture;
  float *mData;          // texture data
  double *mEvaporation;  // texture evaporation
  WbVector2 mOriginalTextureSize;
  WbVector2 mTextureSize;

  WbVector2 computeTextureSize(int imageTextureWidth, int imageTextureHeight);
  WbVector2 computeDefaultTextureSize();

  // static functions for managind evaporation
  void evaporateInk(double evaporationFactor, bool &isDone);

private slots:
  void restoreWrenTexture();
};

#endif
