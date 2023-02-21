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

#ifndef WB_WREN_FULL_SCREEN_HPP
#define WB_WREN_FULL_SCREEN_HPP

#include <QtCore/QObject>

struct WrOverlay;
struct WrViewport;
struct WrTexture;
struct WrDrawableTexture;

class WbWrenFullScreenOverlay : public QObject {
  Q_OBJECT

public:
  WbWrenFullScreenOverlay(const QString &text, int fontSize, bool onTop);
  virtual ~WbWrenFullScreenOverlay();

  void adjustSize();
  void attachToViewport(WrViewport *viewport);

  void setExternalTexture(WrTexture *texture);

  void setVisible(bool visible);
  bool isVisible() { return mIsVisible; }

  WrTexture *overlayTexture();

  void render();

private:
  void setupTexture(const QString &text, int fontSize);

  bool mIsVisible;
  int mTextureWidth;
  int mTextureHeight;

  WrOverlay *mOverlay;
  WrViewport *mViewport;

  WrDrawableTexture *mTexture;
};

#endif
