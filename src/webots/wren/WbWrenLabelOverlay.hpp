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

#ifndef WB_WREN_LABEL_OVERLAY_HPP
#define WB_WREN_LABEL_OVERLAY_HPP

//
// Description: helper class managing the textual labels
//

#include <QtCore/QList>
#include <QtCore/QString>

#include <wren/drawable_texture.h>
#include <wren/font.h>
#include <wren/overlay.h>

class WbWrenLabelOverlay {
public:
  static void cleanup();
  static void updateOverlaysDimensions();

  static WbWrenLabelOverlay *createOrRetrieve(int id, const QString &font);
  static WbWrenLabelOverlay *retrieveById(int id);
  static void removeAllLabels();
  static void removeLabel(int id);
  static int movieCaptionOverlayId() { return 65535; }
  static int dragCaptionOverlayId() { return 65534; }
  static int cameraCaptionOverlayId() { return 65533; }
  static void colorToArray(float *dest, int color);

  void setText(const QString &text);
  void setPosition(double x, double y) {
    mX = x;
    mY = y;
  }
  void setSize(double size) { mSize = size; }
  void setColor(int color) {
    colorToArray(mColor, color);
    mTextNeedRedraw = true;
  }
  void setBackgroundColor(int color) { colorToArray(mBackgroundColor, color); }

  int id() const { return mId; };
  const QString &text() const { return mText; }
  const QString &font() const { return mFontName; }
  double size() const { return mSize; }
  double x() const { return mX; }
  double y() const { return mY; }
  const float *color() const { return mColor; }
  void position(double &x, double &y) const {
    x = mX;
    y = mY;
  }
  void color(int &r, int &g, int &b, float &alpha) const {
    b = mColor[0] * 255;
    g = mColor[1] * 255;
    r = mColor[2] * 255;
    alpha = mColor[3];
  }

  // change properties of Wren object after it has been created
  // i.e. after 'applyChangesToWren' is called
  void moveToPosition(float x, float y);
  void updateText(const QString &text);

  void applyChangesToWren();

  float width() const { return wr_overlay_get_width(mOverlay); }
  float height() const { return wr_overlay_get_height(mOverlay); }
  QString getFontError() const;

private:
  static QList<WbWrenLabelOverlay *> cLabelOverlays;
  static const float HORIZONTAL_MARGIN;

  WbWrenLabelOverlay(int id, const QString &font);
  ~WbWrenLabelOverlay();

  void createTexture();
  void updateTextureSize();
  void deleteOverlay();

  void drawText();

  void updateOverlayDimensions();

  int mId;
  QString mText;
  QString mFontName;
  double mX;
  double mY;
  double mSize;
  float mColor[4];
  float mBackgroundColor[4];
  int mLinesCount;
  bool mTextNeedRedraw;

  WrOverlay *mOverlay;
  WrDrawableTexture *mTexture;
  WrFont *mWrenFont;
};

#endif
