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

#ifndef WB_WREN_TEXTURE_OVERLAY_HPP
#define WB_WREN_TEXTURE_OVERLAY_HPP

#include "WbVector2.hpp"

#include <QtCore/QObject>

struct WrOverlay;
struct WrTexture;
struct WrTexture2d;

class WbWrenTextureOverlay : public QObject {
  Q_OBJECT

public:
  enum TextureType { TEXTURE_TYPE_BGRA, TEXTURE_TYPE_DEPTH };

  enum OverlayType { OVERLAY_TYPE_CAMERA, OVERLAY_TYPE_RANGE_FINDER, OVERLAY_TYPE_DISPLAY, OVERLAY_TYPE_COUNT };

  static void updateOverlayDimensions();
  static void setElementsVisible(OverlayType type, bool visible);

  WbWrenTextureOverlay(void *data, int width, int height, TextureType textureType, OverlayType overlayType,
                       WrTexture *texture = NULL, double maxRange = 1.0, bool rangeCamera = false,
                       bool needTransparency = true);
  virtual ~WbWrenTextureOverlay();

  // get pixel functions
  void convertMousePositionToIndex(int x, int y, int &u, int &v, bool &resizeArea) const;

  // update function
  void requestUpdateTexture() { mRequestUpdateTexture = true; }

  // getters
  // overlay visible in the scene
  bool isVisible() const;
  // isEnabled() corresponds to the check status in the Robot menu:
  // could differ from isVisible() because of global settings
  // or because already open in external window
  bool isEnabled() const;
  WrTexture *texture() const { return mWrenTexture; }
  WrTexture *foregroundTexture() const { return mWrenForegroundTexture; }
  void setBackgroundTexture(WrTexture *backgroundTexture);
  void unsetBackgroundTexture() { setBackgroundTexture(NULL); }
  void setMaskTexture(WrTexture *texture);
  void unsetMaskTexture() { setMaskTexture(NULL); }
  WrTexture2d *createForegroundTexture();
  void deleteForegroundTexture(bool restoreMainTexture = false);
  int textureGLId() const;
  int backgroundTextureGLId() const;
  int maskTextureGLId() const;
  int foregroundTextureGLId() const;
  bool isInside(int x, int y) const;
  bool isInsideResizeArea(int x, int y) const;
  bool isInsideCloseButton(int x, int y) const;
  int zOrder() const;
  double pixelSize() const { return mPixelSize; }
  // return current texture size in pixels
  void size(int &width, int &height) const {
    width = mWidth * mPixelSize;
    height = mHeight * mPixelSize;
  }

  // setters
  void setMaxRange(double r);
  void putOnTop() const;

  bool resize(double pixelSize, bool showIfNeeded = true);
  void translateInPixels(int dx, int dy);
  void updatePercentagePosition(double x, double y);
  void setVisible(bool visible, bool globalOverlaysEnabled);

  // enable panel overlay showing the effective size of the texture
  void enableDefaultSizeBackground(bool enabled);

  QStringList perspective() const;
  void restorePerspective(QStringList &perspective, bool globalOverlaysEnabled);

public slots:
  void updateTexture();

signals:
  void textureUpdated();

private:
  // Methods related to the WrTexture *mWrenTexture data member
  void setTextureSizes(int width, int height);  // (re)sets mSize and mSizeP2
  void allocateBlackImageIntoData();
  void applyChangesToWren();
  void applyPositionToWren();
  bool applyDimensionsToWren(bool showIfNeeded);

  // Accessors for mElement
  int left() const;
  int top() const;
  int width() const;
  int height() const;

  WrTexture2d *createIconTexture(QString filePath);
  void copyDataToTexture(void *data, TextureType type, int x, int y, int width, int height);

  // Cleanup methods
  void deleteWren();

  TextureType mTextureType;
  OverlayType mOverlayType;

  void *mData;
  bool mOwnTexture;  // define if the texture was created by the overlay or passed in argument
  int mWidth;
  int mHeight;
  double mMaxRange;
  bool mRequestUpdateTexture;
  double mPixelSize;
  bool mDataHasBeenAllocated;
  WbVector2 mPercentagePosition;  // position expressed as a percentage of the 3D view size

  WrTexture *mWrenTexture;
  WrTexture *mWrenBackgroundTexture;  // never owned by overlay
  WrTexture *mWrenMaskTexture;        // never owned by overlay
  WrTexture *mWrenForegroundTexture;  // always owned by overlay
  WrTexture2d *mWrenCloseIconTexture;
  WrTexture2d *mWrenResizeIconTexture;
  WrOverlay *mWrenOverlay;
};

#endif
