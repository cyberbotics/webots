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

#include "WbWrenFullScreenOverlay.hpp"

#include "WbStandardPaths.hpp"
#include "WbWrenShaders.hpp"

#include <wren/drawable_texture.h>
#include <wren/font.h>
#include <wren/overlay.h>
#include <wren/texture.h>
#include <wren/viewport.h>

#include <cassert>

WbWrenFullScreenOverlay::WbWrenFullScreenOverlay(const QString &text, int fontSize, bool onTop) :
  mIsVisible(false),
  mTextureWidth(0),
  mTextureHeight(0),
  mViewport(NULL),
  mTexture(NULL) {
  mOverlay = wr_overlay_new();
  wr_overlay_set_visible(mOverlay, false);
  if (onTop)
    wr_overlay_set_order(mOverlay, 10000);
  else
    wr_overlay_set_order(mOverlay, 0);
  wr_overlay_set_default_size(mOverlay, 1, 1);
  wr_overlay_set_size(mOverlay, 1, 1);
  wr_overlay_set_position(mOverlay, 0, 0);
  wr_overlay_enable_interpolation(mOverlay, true);
  wr_overlay_set_program(mOverlay, WbWrenShaders::overlayShader());

  const float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  wr_overlay_set_background_color(mOverlay, black);

  wr_overlay_set_border_active(mOverlay, true);
  wr_overlay_set_border_color(mOverlay, black);
  wr_overlay_set_border_size(mOverlay, 0, 0);

  wr_overlay_set_cache_persistency(mOverlay, true);
  wr_overlay_set_translucency(mOverlay, true);

  setupTexture(text, fontSize);
}

WbWrenFullScreenOverlay::~WbWrenFullScreenOverlay() {
  wr_texture_delete(WR_TEXTURE(mTexture));
  if (mViewport)
    wr_viewport_detach_overlay(mViewport, mOverlay);

  wr_overlay_delete(mOverlay);
}

void WbWrenFullScreenOverlay::attachToViewport(WrViewport *viewport) {
  if (mViewport)
    wr_viewport_detach_overlay(mViewport, mOverlay);

  mViewport = viewport;
  wr_viewport_attach_overlay(mViewport, mOverlay);
}

void WbWrenFullScreenOverlay::setVisible(bool visible) {
  mIsVisible = visible;
  wr_overlay_set_visible(mOverlay, visible);

  if (visible)
    adjustSize();
}

WrTexture *WbWrenFullScreenOverlay::overlayTexture() {
  return WR_TEXTURE(mTexture);
}

void WbWrenFullScreenOverlay::setExternalTexture(WrTexture *texture) {
  WrTexture *usedTexture = texture ? texture : WR_TEXTURE(mTexture);
  mTextureWidth = wr_texture_get_width(usedTexture);
  mTextureHeight = wr_texture_get_height(usedTexture);
  wr_overlay_set_texture(mOverlay, usedTexture);
  if (mViewport)
    attachToViewport(mViewport);
  adjustSize();
}

void WbWrenFullScreenOverlay::adjustSize() {
  if (!mViewport)
    return;

  const int viewportWidth = wr_viewport_get_width(mViewport);
  const int viewportHeight = wr_viewport_get_height(mViewport);

  float overlayWidth = mTextureWidth / (float)viewportWidth;
  float overlayHeight = mTextureHeight / (float)viewportHeight;
  if (viewportWidth < mTextureWidth) {
    overlayWidth = 1.0f;
    overlayHeight = (mTextureHeight / (float)viewportHeight) * (viewportWidth / (float)mTextureWidth);
  } else if (viewportHeight < mTextureHeight) {
    overlayHeight = 1.0f;
    overlayWidth = (mTextureHeight / (float)viewportHeight) * (viewportHeight / (float)mTextureHeight);
  }

  wr_overlay_set_size(mOverlay, overlayWidth, overlayHeight);
  wr_overlay_set_border_size(mOverlay, (1.0f - overlayWidth) / 2.0f, (1.0f - overlayHeight) / 2.0f);
}

void WbWrenFullScreenOverlay::setupTexture(const QString &text, int fontSize) {
  // Prepare font
  WrFont *font = wr_font_new();
  wr_font_set_face(font, (WbStandardPaths::fontsPath() + "Ariali.ttf").toUtf8().constData());
  wr_font_set_size(font, fontSize);

  assert(wr_font_get_error(font) == WR_FONT_ERROR_NONE);

  wr_font_get_bounding_box(font, text.toStdString().c_str(), &mTextureWidth, &mTextureHeight);
  mTextureWidth += 10;  // since we are using an italic font the size computed using freetype is underestimated

  if (mTexture)
    wr_texture_delete(WR_TEXTURE(mTexture));
  mTexture = wr_drawable_texture_new();
  wr_drawable_texture_set_antialasing(mTexture, true);
  wr_texture_set_size(WR_TEXTURE(mTexture), mTextureWidth, mTextureHeight);
  wr_texture_set_translucent(WR_TEXTURE(mTexture), false);
  wr_texture_setup(WR_TEXTURE(mTexture));

  float white[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  wr_drawable_texture_set_color(mTexture, white);
  wr_drawable_texture_clear(mTexture);
  wr_drawable_texture_set_font(mTexture, font);
  wr_drawable_texture_draw_text(mTexture, text.toStdString().c_str(), 0, 0);

  wr_overlay_set_texture(mOverlay, WR_TEXTURE(mTexture));

  wr_font_delete(font);
}

void WbWrenFullScreenOverlay::render() {
  wr_viewport_render_overlay(mViewport, mOverlay);
}
