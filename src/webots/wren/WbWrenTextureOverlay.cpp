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

#include "WbWrenTextureOverlay.hpp"

#include "WbLog.hpp"
#include "WbMathsUtilities.hpp"
#include "WbPerformanceLog.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/overlay.h>
#include <wren/scene.h>
#include <wren/static_mesh.h>
#include <wren/texture.h>
#include <wren/texture_2d.h>
#include <wren/texture_rtt.h>
#include <wren/viewport.h>

#include <QtCore/QFileInfo>
#include <QtCore/QHash>
#include <QtGui/QImageReader>

static int cResizeIconSize = -1;
static int cCloseIconSize = -1;
static int cBorderSizeHorizontal = 1;
static int cBorderSizeVertical = 1;

// Color values taken from textures in resources/ogre/textures
static float cBorderColors[WbWrenTextureOverlay::OVERLAY_TYPE_COUNT][4] = {{1.0f, 0.0f, 1.0f, 1.0f},
                                                                           {1.0f, 0.815f, 0.0f, 1.0f},
                                                                           {0.0f, 1.0f, 1.0f, 1.0f}};

// map storing status of overlay elements:
// TRUE: overlay enabled
// FALSE: overlay disabled, i.e. already open in an external window
static QHash<WrOverlay *, std::pair<WbWrenTextureOverlay *, bool>> cOverlayStatusMap;

////////////////////////////////////////
//  Constructor  and initializations  //
////////////////////////////////////////

WbWrenTextureOverlay::WbWrenTextureOverlay(void *data, int width, int height, TextureType textureType, OverlayType overlayType,
                                           WrTexture *texture, double maxRange, bool rangeCamera, bool needTransparency) :
  mTextureType(textureType),
  mOverlayType(overlayType),
  mData(data),
  mWidth(width),
  mHeight(height),
  mMaxRange(maxRange),
  mRequestUpdateTexture(true),
  mPixelSize(1.0),
  mDataHasBeenAllocated(false),
  mWrenBackgroundTexture(NULL),
  mWrenMaskTexture(NULL),
  mWrenForegroundTexture(NULL) {
  if (!mData)
    allocateBlackImageIntoData();

  WbWrenOpenGlContext::makeWrenCurrent();

  if (texture) {
    mRequestUpdateTexture = false;
    mOwnTexture = false;
    mWrenTexture = texture;
  } else {
    mOwnTexture = true;

    WrTexture2d *texture2d = wr_texture_2d_new();
    wr_texture_set_size(WR_TEXTURE(texture2d), mWidth, mHeight);
    wr_texture_set_translucent(WR_TEXTURE(texture2d), needTransparency);
    wr_texture_2d_set_data(texture2d, reinterpret_cast<const char *>(mData));
    wr_texture_setup(WR_TEXTURE(texture2d));

    mWrenTexture = WR_TEXTURE(texture2d);
  }

  QString filePrefix;
  switch (overlayType) {
    case OVERLAY_TYPE_CAMERA:
      filePrefix = "magenta_";
      break;
    case OVERLAY_TYPE_RANGE_FINDER:
      filePrefix = "yellow_";
      break;
    case OVERLAY_TYPE_DISPLAY:
      filePrefix = "cyan_";
      break;
    default:
      assert(false);
  }

  mWrenCloseIconTexture = createIconTexture("gl:textures/" + filePrefix + "close_symbol.png");
  mWrenResizeIconTexture = createIconTexture("gl:textures/" + filePrefix + "resize_symbol.png");
  cCloseIconSize = wr_texture_get_width(WR_TEXTURE(mWrenCloseIconTexture));
  cResizeIconSize = wr_texture_get_width(WR_TEXTURE(mWrenResizeIconTexture));

  mWrenOverlay = wr_overlay_new();
  wr_viewport_attach_overlay(wr_scene_get_viewport(wr_scene_get_instance()), mWrenOverlay);

  wr_overlay_set_texture(mWrenOverlay, mWrenTexture);
  wr_overlay_add_additional_texture(mWrenOverlay, WR_TEXTURE(mWrenCloseIconTexture));
  wr_overlay_add_additional_texture(mWrenOverlay, WR_TEXTURE(mWrenResizeIconTexture));
  wr_overlay_set_program(mWrenOverlay, WbWrenShaders::overlayShader());
  wr_overlay_set_border_active(mWrenOverlay, true);
  wr_overlay_set_border_color(mWrenOverlay, cBorderColors[overlayType]);
  wr_overlay_set_translucency(mWrenOverlay, needTransparency);
  // Webots rendering device textures are upside down w.r.t OpenGL convention
  wr_overlay_set_texture_flip_vertical(mWrenOverlay, true);
  applyChangesToWren();

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::mainRenderingStarted, this,
          &WbWrenTextureOverlay::updateTexture);

  updateTexture();
  updatePercentagePosition(0.0, 0.0);

  cOverlayStatusMap.insert(mWrenOverlay, std::pair<WbWrenTextureOverlay *, bool>(this, true));
  setVisible(false, true);

  WbWrenOpenGlContext::doneWren();
}

////////////////////////////////////////
//  Destructor and cleanup methods    //
////////////////////////////////////////

WbWrenTextureOverlay::~WbWrenTextureOverlay() {
  cOverlayStatusMap.remove(mWrenOverlay);

  if (mDataHasBeenAllocated)
    free(mData);

  deleteWren();
}

void WbWrenTextureOverlay::deleteWren() {
  wr_viewport_detach_overlay(wr_scene_get_viewport(wr_scene_get_instance()), mWrenOverlay);
  wr_overlay_delete(mWrenOverlay);

  if (mOwnTexture)
    wr_texture_delete(mWrenTexture);

  wr_texture_delete(mWrenForegroundTexture);
  wr_texture_delete(WR_TEXTURE(mWrenCloseIconTexture));
  wr_texture_delete(WR_TEXTURE(mWrenResizeIconTexture));
}

bool WbWrenTextureOverlay::resize(double pixelSize, bool showIfNeeded) {
  if (pixelSize <= 0.0)
    return true;

  mPixelSize = pixelSize;
  return applyDimensionsToWren(showIfNeeded);
}

void WbWrenTextureOverlay::translateInPixels(int dx, int dy) {
  mPercentagePosition +=
    WbVector2(static_cast<double>(dx) / wr_viewport_get_width(wr_scene_get_viewport(wr_scene_get_instance())),
              static_cast<double>(dy) / wr_viewport_get_height(wr_scene_get_viewport(wr_scene_get_instance())));

  applyPositionToWren();
}

void WbWrenTextureOverlay::updatePercentagePosition(double x, double y) {
  mPercentagePosition = WbVector2(x, y);

  if (mWrenOverlay)
    applyPositionToWren();
}

bool WbWrenTextureOverlay::applyDimensionsToWren(bool showIfNeeded) {
  if (!mWrenOverlay)
    return false;

  const float minSize = 32.0f;

  const float view3dWidth = static_cast<float>(wr_viewport_get_width(wr_scene_get_viewport(wr_scene_get_instance())));
  const float view3dHeight = static_cast<float>(wr_viewport_get_height(wr_scene_get_viewport(wr_scene_get_instance())));

  // Check viewport size is valid
  if (view3dWidth <= 1 || view3dHeight <= 1)
    return true;

  float overlayWidth = mPixelSize * mWidth;
  float overlayHeight = mPixelSize * mHeight;
  const float overlayRatio = overlayWidth / overlayHeight;

  // enforce horizontal constraint
  if (mPercentagePosition.x() * view3dWidth + overlayWidth > view3dWidth) {
    overlayWidth = view3dWidth - mPercentagePosition.x() * view3dWidth - 2.0f * cBorderSizeHorizontal;
    overlayHeight = overlayWidth / overlayRatio;
    mPixelSize = overlayWidth / mWidth;
  }
  // enforce vertical constraint
  if (mPercentagePosition.y() * view3dHeight + overlayHeight > view3dHeight) {
    overlayHeight = view3dHeight - mPercentagePosition.y() * view3dHeight - 2.0f * cBorderSizeVertical;
    overlayWidth = overlayHeight * overlayRatio;
    mPixelSize = overlayHeight / mHeight;
  }
  // enforce minimal size constraint
  if (overlayWidth < minSize || overlayHeight < minSize) {
    if (overlayWidth < overlayHeight) {
      overlayWidth = minSize;
      overlayHeight = overlayWidth / overlayRatio;
      mPixelSize = minSize / mWidth;
    } else {
      overlayHeight = minSize;
      overlayWidth = overlayHeight * overlayRatio;
      mPixelSize = minSize / mHeight;
    }
  }

  if (showIfNeeded && !wr_overlay_is_visible(mWrenOverlay))
    wr_overlay_set_visible(mWrenOverlay, true);

  wr_overlay_set_size(mWrenOverlay, overlayWidth / view3dWidth, overlayHeight / view3dHeight);
  wr_overlay_set_default_size(mWrenOverlay, mWidth / view3dWidth, mHeight / view3dHeight);
  wr_overlay_set_border_size(mWrenOverlay, cBorderSizeHorizontal / view3dWidth, cBorderSizeVertical / view3dHeight);

  return true;
}

void WbWrenTextureOverlay::applyPositionToWren() {
  if (!mWrenOverlay)
    return;

  const float view3dWidth = static_cast<float>(wr_viewport_get_width(wr_scene_get_viewport(wr_scene_get_instance())));
  const float view3dHeight = static_cast<float>(wr_viewport_get_height(wr_scene_get_viewport(wr_scene_get_instance())));
  const float overlayWidth = mPixelSize * mWidth + 2.0f * cBorderSizeHorizontal;
  const float overlayHeight = mPixelSize * mHeight + 2.0f * cBorderSizeVertical;
  mPercentagePosition.setXy(qBound(0.0f, static_cast<float>(mPercentagePosition.x()), 1.0f - overlayWidth / view3dWidth),
                            qBound(0.0f, static_cast<float>(mPercentagePosition.y()), 1.0f - overlayHeight / view3dHeight));

  wr_overlay_set_position(mWrenOverlay, mPercentagePosition.x(), mPercentagePosition.y());
}

void WbWrenTextureOverlay::applyChangesToWren() {
  if (!applyDimensionsToWren(false))
    return;

  applyPositionToWren();
}

void WbWrenTextureOverlay::updateTexture() {
  if (!mRequestUpdateTexture)
    return;

  copyDataToTexture(mData, mTextureType, 0, 0, mWidth, mHeight);
  mRequestUpdateTexture = false;
  emit textureUpdated();
}

WrTexture2d *WbWrenTextureOverlay::createIconTexture(QString filePath) {
  assert(QFileInfo(filePath).isFile());

  // don't read the image from disk if it's already in the cache
  WrTexture2d *imageTexture = wr_texture_2d_copy_from_cache(filePath.toUtf8().constData());
  if (imageTexture)
    return imageTexture;

  QImageReader imageReader(filePath);
  QImage image = imageReader.read().mirrored(false, true);  // account for inverted Y axis in OpenGL
  const bool isTranslucent = image.pixelFormat().alphaUsage() == QPixelFormat::UsesAlpha;

  imageTexture = wr_texture_2d_new();
  wr_texture_set_size(WR_TEXTURE(imageTexture), image.width(), image.height());
  wr_texture_set_translucent(WR_TEXTURE(imageTexture), isTranslucent);
  wr_texture_2d_set_data(imageTexture, reinterpret_cast<const char *>(image.bits()));
  wr_texture_2d_set_file_path(imageTexture, filePath.toUtf8().constData());
  wr_texture_setup(WR_TEXTURE(imageTexture));

  return imageTexture;
}

void WbWrenTextureOverlay::copyDataToTexture(void *data, TextureType type, int x, int y, int width, int height) {
  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::GPU_MEMORY_TRANSFER);

  WbWrenOpenGlContext::makeWrenCurrent();

  if (type == TEXTURE_TYPE_DEPTH) {
    int *processedData = new int[width * height];
    float *originalData = static_cast<float *>(data);
    const float multiplier = 255.0f / mMaxRange;
    for (int i = 0; i < width * height; ++i) {
      unsigned char v = (unsigned char)(multiplier * originalData[i]);
      processedData[i] = 0xFF000000 + (v << 16) + (v << 8) + v;
    }

    wr_texture_change_data(mWrenTexture, processedData, x, y, width, height);

    delete[] processedData;
  } else if (type == TEXTURE_TYPE_BGRA)
    wr_texture_change_data(mWrenTexture, data, x, y, width, height);

  WbWrenOpenGlContext::doneWren();

  if (log)
    log->stopMeasure(WbPerformanceLog::GPU_MEMORY_TRANSFER);
}

void WbWrenTextureOverlay::allocateBlackImageIntoData() {
  assert(!mData);

  int imageSize = mWidth * mHeight;
  switch (mTextureType) {
    case TEXTURE_TYPE_BGRA: {
      mData = malloc(4 * imageSize);

      int *dataInt = static_cast<int *>(mData);
      for (int i = 0; i < imageSize; i++)
        dataInt[i] = 0xFF000000;
      break;
    }
    case TEXTURE_TYPE_DEPTH: {
      mData = malloc(sizeof(float) * imageSize);

      float *dataFloat = static_cast<float *>(mData);
      for (int i = 0; i < imageSize; i++)
        dataFloat[i] = 0.0f;
      break;
    }
    default:
      assert(0);
      break;
  }

  mDataHasBeenAllocated = true;
}

int WbWrenTextureOverlay::textureGLId() const {
  return wr_texture_get_gl_name(mWrenTexture);
}

int WbWrenTextureOverlay::backgroundTextureGLId() const {
  if (mWrenBackgroundTexture)
    return wr_texture_get_gl_name(mWrenBackgroundTexture);
  else
    return 0;
}

int WbWrenTextureOverlay::maskTextureGLId() const {
  if (mWrenMaskTexture)
    return wr_texture_get_gl_name(mWrenMaskTexture);
  else
    return 0;
}

int WbWrenTextureOverlay::foregroundTextureGLId() const {
  if (mWrenForegroundTexture)
    return wr_texture_get_gl_name(mWrenForegroundTexture);
  else
    return 0;
}

/////////////////////////////////////
// Accessors for mElement features //
/////////////////////////////////////

int WbWrenTextureOverlay::left() const {
  return wr_overlay_get_x(mWrenOverlay) * wr_viewport_get_width(wr_scene_get_viewport(wr_scene_get_instance())) +
         cBorderSizeHorizontal;
}

int WbWrenTextureOverlay::top() const {
  return wr_overlay_get_y(mWrenOverlay) * wr_viewport_get_height(wr_scene_get_viewport(wr_scene_get_instance())) +
         cBorderSizeVertical;
}

int WbWrenTextureOverlay::width() const {
  return wr_overlay_get_width(mWrenOverlay) * wr_viewport_get_width(wr_scene_get_viewport(wr_scene_get_instance()));
}

int WbWrenTextureOverlay::height() const {
  return wr_overlay_get_height(mWrenOverlay) * wr_viewport_get_height(wr_scene_get_viewport(wr_scene_get_instance()));
}

bool WbWrenTextureOverlay::isVisible() const {
  return (mWrenOverlay && wr_overlay_is_visible(mWrenOverlay));
}

bool WbWrenTextureOverlay::isEnabled() const {
  return (mWrenOverlay && cOverlayStatusMap.contains(mWrenOverlay) && cOverlayStatusMap[mWrenOverlay].second);
}

void WbWrenTextureOverlay::setBackgroundTexture(WrTexture *backgroundTexture) {
  mWrenBackgroundTexture = backgroundTexture;
  wr_overlay_set_background_texture(mWrenOverlay, mWrenBackgroundTexture);
}

void WbWrenTextureOverlay::setMaskTexture(WrTexture *texture) {
  mWrenMaskTexture = texture;
  wr_overlay_set_mask_texture(mWrenOverlay, mWrenMaskTexture);
}

WrTexture2d *WbWrenTextureOverlay::createForegroundTexture() {
  WrTexture2d *texture2d = wr_texture_2d_new();
  wr_texture_set_size(WR_TEXTURE(texture2d), mWidth, mHeight);
  wr_texture_set_translucent(WR_TEXTURE(texture2d), true);
  wr_texture_2d_set_data(texture2d, NULL);
  // make context active to generate immediately the foreground texture name
  WbWrenOpenGlContext::makeWrenCurrent();
  wr_texture_setup(WR_TEXTURE(texture2d));
  WbWrenOpenGlContext::doneWren();

  mWrenForegroundTexture = WR_TEXTURE(texture2d);
  wr_overlay_set_foreground_texture(mWrenOverlay, mWrenForegroundTexture);

  return texture2d;
}

void WbWrenTextureOverlay::deleteForegroundTexture(bool restoreMainTexture) {
  wr_texture_delete(mWrenForegroundTexture);
  mWrenForegroundTexture = NULL;

  wr_overlay_set_foreground_texture(mWrenOverlay, mWrenForegroundTexture);
}

int WbWrenTextureOverlay::zOrder() const {
  return wr_overlay_get_order(mWrenOverlay);
}

void WbWrenTextureOverlay::setMaxRange(double r) {
  mMaxRange = r;
  wr_overlay_set_max_range(mWrenOverlay, mMaxRange);
}

void WbWrenTextureOverlay::putOnTop() const {
  wr_overlay_put_on_top(mWrenOverlay);
}

void WbWrenTextureOverlay::setVisible(bool visible, bool globalOverlaysEnabled) {
  if (!mWrenOverlay)
    return;

  if (visible && globalOverlaysEnabled) {
    if (mPixelSize <= 0)
      resize(1.0, true);

    wr_overlay_set_visible(mWrenOverlay, true);
  } else
    wr_overlay_set_visible(mWrenOverlay, false);

  assert(cOverlayStatusMap.contains(mWrenOverlay));
  cOverlayStatusMap[mWrenOverlay].second = visible;
}

void WbWrenTextureOverlay::enableDefaultSizeBackground(bool enabled) {
  wr_overlay_show_default_size(mWrenOverlay, enabled);
}

QStringList WbWrenTextureOverlay::perspective() const {
  QStringList perspective;
  perspective << (cOverlayStatusMap[mWrenOverlay].second ? "1" : "0");
  perspective << QString::number(mPixelSize);
  perspective << QString::number(mPercentagePosition.x());
  perspective << QString::number(mPercentagePosition.y());
  return perspective;
}

void WbWrenTextureOverlay::restorePerspective(QStringList &perspective, bool globalOverlaysEnabled) {
  assert(perspective.size() >= 4);

  bool visible = perspective.takeFirst() == "1";
  // cppcheck-suppress duplicateAssignExpression
  double pixelsSize = perspective.takeFirst().toDouble();
  // cppcheck-suppress duplicateAssignExpression
  double x = perspective.takeFirst().toDouble();
  // cppcheck-suppress duplicateAssignExpression
  double y = perspective.takeFirst().toDouble();
  resize(pixelsSize);
  updatePercentagePosition(x, y);
  setVisible(visible, globalOverlaysEnabled);
}

/////////////////////////////////////////////////
// Utility function used to display pixel info //
/////////////////////////////////////////////////

void WbWrenTextureOverlay::convertMousePositionToIndex(int x, int y, int &u, int &v, bool &resizeArea) const {
  if (isInside(x, y)) {
    u = (x - left()) * mWidth / width();
    v = (y - top()) * mHeight / height();
    resizeArea = isInsideResizeArea(x, y);
  } else {
    u = -1;
    v = -1;
    resizeArea = false;
  }
}

bool WbWrenTextureOverlay::isInside(int x, int y) const {
  if (!wr_overlay_is_visible(mWrenOverlay))
    return false;

  return (x >= left() && x < left() + width() && y >= top() && y < top() + height());
}

bool WbWrenTextureOverlay::isInsideResizeArea(int x, int y) const {
  if (!wr_overlay_is_visible(mWrenOverlay))
    return false;

  return ((left() + width() - x) < cResizeIconSize) && ((top() + height() - y) < cResizeIconSize);
}

bool WbWrenTextureOverlay::isInsideCloseButton(int x, int y) const {
  if (!wr_overlay_is_visible(mWrenOverlay))
    return false;

  const int right = left() + width();
  return (x >= (right - cCloseIconSize) && x < right && y >= top() && y < (top() + cCloseIconSize));
}

//////////////////////////////////////////////////////
// Static methods for the rendering devices overlay //
//////////////////////////////////////////////////////

void WbWrenTextureOverlay::updateOverlayDimensions() {
  for (const std::pair<WbWrenTextureOverlay *, bool> &p : cOverlayStatusMap)
    p.first->applyChangesToWren();
}

void WbWrenTextureOverlay::setElementsVisible(OverlayType type, bool visible) {
  for (const std::pair<WbWrenTextureOverlay *, bool> &p : cOverlayStatusMap) {
    if (p.first->mOverlayType == type && p.second)  // skip explicitly closed overlays
      wr_overlay_set_visible(p.first->mWrenOverlay, visible);
  }
}
