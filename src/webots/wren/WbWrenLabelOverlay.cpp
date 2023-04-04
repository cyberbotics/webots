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

#include "WbWrenLabelOverlay.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/drawable_texture.h>
#include <wren/scene.h>
#include <wren/texture.h>
#include <wren/viewport.h>

QList<WbWrenLabelOverlay *> WbWrenLabelOverlay::cLabelOverlays;
const float WbWrenLabelOverlay::HORIZONTAL_MARGIN = 0.15f;  // in % of height

WbWrenLabelOverlay *WbWrenLabelOverlay::createOrRetrieve(int id, const QString &font) {
  foreach (WbWrenLabelOverlay *overlay, cLabelOverlays) {
    if (overlay->id() == id) {
      // If the font changed, a new label overlay is created
      if (overlay->font() == font)
        return overlay;
      else
        delete overlay;
    }
  }
  return new WbWrenLabelOverlay(id, font);
}

WbWrenLabelOverlay *WbWrenLabelOverlay::retrieveById(int id) {
  foreach (WbWrenLabelOverlay *overlay, cLabelOverlays) {
    if (overlay->id() == id)
      return overlay;
  }
  return NULL;
}

void WbWrenLabelOverlay::updateOverlaysDimensions() {
  foreach (WbWrenLabelOverlay *overlay, cLabelOverlays)
    overlay->updateOverlayDimensions();
}

void WbWrenLabelOverlay::cleanup() {
  removeAllLabels();
}

void WbWrenLabelOverlay::removeAllLabels() {
  while (!cLabelOverlays.isEmpty())
    delete cLabelOverlays.takeFirst();
}

void WbWrenLabelOverlay::removeLabel(int id) {
  for (int i = 0, size = cLabelOverlays.size(); i < size; ++i) {
    WbWrenLabelOverlay *overlay = cLabelOverlays[i];
    if (overlay->id() == id) {
      cLabelOverlays.removeAll(overlay);
      delete overlay;
      return;
    }
  }
}

void WbWrenLabelOverlay::setText(const QString &text) {
  if (mText == text)
    return;
  mTextNeedRedraw = true;
  mText = text;
  mLinesCount = 1 + mText.count('\n');
}

void WbWrenLabelOverlay::moveToPosition(float x, float y) {
  setPosition(x, y);
  wr_overlay_set_position(mOverlay, mX, mY);
}

void WbWrenLabelOverlay::updateText(const QString &text) {
  if (text == mText)
    return;

  setText(text);

  WbWrenOpenGlContext::makeWrenCurrent();
  updateTextureSize();
  drawText();
  WbWrenOpenGlContext::doneWren();
  updateOverlayDimensions();
}

void WbWrenLabelOverlay::applyChangesToWren() {
  if (!mTexture)
    createTexture();
  else {
    updateTextureSize();
    drawText();
  }

  wr_overlay_set_position(mOverlay, mX, mY);
  wr_overlay_set_background_color(mOverlay, mBackgroundColor);

  updateOverlayDimensions();
}

WbWrenLabelOverlay::WbWrenLabelOverlay(int id, const QString &font) :
  mId(id),
  mFontName(font),
  mX(0.0f),
  mY(0.0f),
  mSize(0.0f),
  mLinesCount(1),
  mTextNeedRedraw(false),
  mOverlay(NULL),
  mTexture(NULL) {
  cLabelOverlays.append(this);
  mOverlay = wr_overlay_new();
  wr_overlay_set_program(mOverlay, WbWrenShaders::overlayShader());
  wr_overlay_set_use_premultiplied_alpha_textures(mOverlay, true);
  wr_overlay_enable_mip_maps(mOverlay, true);
  wr_overlay_enable_interpolation(mOverlay, true);
  wr_overlay_set_translucency(mOverlay, true);

  wr_viewport_attach_overlay(wr_scene_get_viewport(wr_scene_get_instance()), mOverlay);

  mWrenFont = wr_font_new();
  wr_font_set_face(mWrenFont, font.toUtf8().constData());

  // Font size is fixed, overlay is resized to match required size
  wr_font_set_size(mWrenFont, 32);

  setColor(0x0);
  setBackgroundColor(0xFF000000);
}

WbWrenLabelOverlay::~WbWrenLabelOverlay() {
  cLabelOverlays.removeAll(this);
  deleteOverlay();
}

void WbWrenLabelOverlay::colorToArray(float *dest, int color) {
  dest[0] = (float)(color & 0xFF) / 255.0f;
  dest[1] = (float)((color >> 8) & 0xFF) / 255.0f;
  dest[2] = (float)((color >> 16) & 0xFF) / 255.0f;
  dest[3] = 1.0f - ((float)((color >> 24) & 0xFF) / 255.0f);
}

QString WbWrenLabelOverlay::getFontError() const {
  switch (wr_font_get_error(mWrenFont)) {
    case WR_FONT_ERROR_UNKNOWN_FILE_FORMAT:
      return "The font file could be opened and read, but its font format is unsupported.";
    case WR_FONT_ERROR_FONT_LOADING:
      return "The font file could not be opened or read, or is broken.";
    case WR_FONT_ERROR_FONT_SIZE:
      return "An error occurred while setting the font size.";
    case WR_FONT_ERROR_CHARACTER_LOADING:
      return "An error occurred while loading a character.";
    case WR_FONT_ERROR_FREETYPE_LOADING:
      return "An error occurred while initializing the freetype library.";
    default:
      return "";
  }
}

void WbWrenLabelOverlay::createTexture() {
  mTexture = wr_drawable_texture_new();
  wr_drawable_texture_set_font(mTexture, mWrenFont);
  wr_drawable_texture_set_use_premultiplied_alpha(mTexture, true);
  updateTextureSize();

  WrTexture *texture = WR_TEXTURE(mTexture);
  wr_texture_set_translucent(texture, true);
  drawText();
  wr_texture_setup(texture);
  wr_overlay_set_texture(mOverlay, texture);
}

void WbWrenLabelOverlay::updateTextureSize() {
  int fontWidth, fontHeight;
  wr_font_get_bounding_box(mWrenFont, mText.toUtf8().constData(), &fontWidth, &fontHeight);

  fontWidth += 2.0f * (fontHeight / (float)(mLinesCount)) * HORIZONTAL_MARGIN;
  wr_texture_set_size(WR_TEXTURE(mTexture), fontWidth, fontHeight);
}

void WbWrenLabelOverlay::deleteOverlay() {
  wr_texture_delete(WR_TEXTURE(mTexture));

  wr_viewport_detach_overlay(wr_scene_get_viewport(wr_scene_get_instance()), mOverlay);
  wr_overlay_delete(mOverlay);
  wr_font_delete(mWrenFont);

  mOverlay = NULL;
  mTexture = NULL;
}

void WbWrenLabelOverlay::drawText() {
  if (!mTextNeedRedraw)
    return;
  mTextNeedRedraw = false;
  wr_drawable_texture_clear(mTexture);
  wr_drawable_texture_set_color(mTexture, mColor);
  const float marginOffset = HORIZONTAL_MARGIN * (wr_texture_get_height(WR_TEXTURE(mTexture)) / (float)(mLinesCount));
  wr_drawable_texture_draw_text(mTexture, mText.toUtf8().constData(), marginOffset, 0);
}

void WbWrenLabelOverlay::updateOverlayDimensions() {
  if (!mTexture)
    return;

  int textureWidth = wr_texture_get_width(WR_TEXTURE(mTexture));
  int textureHeight = wr_texture_get_height(WR_TEXTURE(mTexture));
  float ratio = textureWidth / (float)(textureHeight);

  WrViewport *viewport = wr_scene_get_viewport(wr_scene_get_instance());
  int viewportHeight = wr_viewport_get_height(viewport);

  float overlayHeight = mLinesCount * mSize * 0.5f;
  float overlayWidth = ((viewportHeight * overlayHeight) * ratio) / wr_viewport_get_width(viewport);

  wr_overlay_set_size(mOverlay, overlayWidth, overlayHeight);
}
