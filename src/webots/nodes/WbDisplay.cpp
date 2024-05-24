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

#include "WbDisplay.hpp"

#include "WbAppearance.hpp"
#include "WbCamera.hpp"
#include "WbDataStream.hpp"
#include "WbDisplayFont.hpp"
#include "WbImageTexture.hpp"
#include "WbMFNode.hpp"
#include "WbPbrAppearance.hpp"
#include "WbPerformanceLog.hpp"
#include "WbProject.hpp"
#include "WbRobot.hpp"
#include "WbShape.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenTextureOverlay.hpp"

#include "wren/texture.h"

#include <climits>
#include <cmath>
#include "../../../include/controller/c/webots/display.h"  // contains the definitions of the image format
#include "../../controller/c/messages.h"  // contains the definitions for the macros C_DISPLAY_SET_COLOR, C_DISPLAY_SET_ALPHA, C_DISPLAY_SET_OPACITY, ...

#include <QtCore/QDataStream>

#ifdef _WIN32
#include <windows.h>
#endif

#define SHIFT(value, shift) (((value) >> (shift)) & 0xFF)

class WbDisplayImage {
public:
  WbDisplayImage(int id, int width, int height, unsigned int *image, bool isTransparent) :
    mId(id),
    mWidth(width),
    mHeight(height),
    mImage(image),
    mIsTransparent(isTransparent) {}
  WbDisplayImage(const WbDisplayImage &d) :
    mId(d.id()),
    mWidth(d.width()),
    mHeight(d.height()),
    mIsTransparent(d.isTransparent()) {
    mImage = new unsigned int[mWidth * mHeight];
    memcpy(mImage, d.image(), mWidth * mHeight * sizeof(unsigned int));
  }
  virtual ~WbDisplayImage() { delete[] mImage; }
  int id() const { return mId; };
  int width() const { return mWidth; };
  int height() const { return mHeight; };
  unsigned int *image() const { return mImage; };
  bool isTransparent() const { return mIsTransparent; };
  virtual void setId(int id) { mId = id; };
  virtual void setWidth(int width) { mWidth = width; };
  virtual void setHeight(int height) { mHeight = height; };
  virtual void setImage(unsigned int *image) { mImage = image; };

private:
  int mId;
  int mWidth;
  int mHeight;
  unsigned int *mImage;
  bool mIsTransparent;

  WbDisplayImage &operator=(const WbDisplayImage &);  // non copyable
};

void WbDisplay::init() {
  mImage = NULL;
  mColor = 0xFFFFFF;  // default: white
  mAlpha = 0xFF;      // default: visible
  mOpacity = 0xFF;    // default: override completely
  mAntiAliasing = true;
  mIsTextureTransparent = false;
  mUpdateRequired = false;
  mRequestImages = false;
  mAttachedCamera = NULL;
  mNeedToSetExternalTextures = false;

  mDisplayFont = new WbDisplayFont();
  QString error = mDisplayFont->error();
  if (!error.isEmpty())
    warn(error);
  setFont(const_cast<char *>("Lucida Console"), 8);
}

WbDisplay::WbDisplay(WbTokenizer *tokenizer) : WbRenderingDevice("Display", tokenizer) {
  init();
}

WbDisplay::WbDisplay(const WbDisplay &other) : WbRenderingDevice(other) {
  init();
}

WbDisplay::WbDisplay(const WbNode &other) : WbRenderingDevice(other) {
  init();
}

WbDisplay::~WbDisplay() {
  delete[] mImage;
  mImage = NULL;

  delete mDisplayFont;
  mDisplayFont = NULL;

  clearImages();

  // clear save_orders
  for (int i = 0; i < mSaveOrders.size(); ++i)
    delete mSaveOrders.at(i);
  mSaveOrders.clear();

  detachCamera();
  clearImageTextures();
}

void WbDisplay::clearImages() {
  for (int i = 0; i < mImages.size(); i++)
    delete mImages.at(i);
  mImages.clear();
}

void WbDisplay::preFinalize() {
  WbRenderingDevice::preFinalize();
  findImageTextures();
}

int WbDisplay::channelNumberFromPixelFormat(int pixelFormat) {
  switch (pixelFormat) {
    case WB_IMAGE_RGB:
      return 3;
    case WB_IMAGE_RGBA:
    case WB_IMAGE_ARGB:
    case WB_IMAGE_BGRA:
    case WB_IMAGE_ABGR:
      return 4;
    default:
      assert(0);
  }
  return 0;
}

void WbDisplay::findImageTextures() {
  mNeedToSetExternalTextures = true;

  clearImageTextures();

  if (children().size() < 1)
    return;

  WbNode *firstChild = children().item(0);
  WbShape *shape = dynamic_cast<WbShape *>(firstChild);
  if (shape) {
    WbAppearance *appearance = shape->appearance();
    WbPbrAppearance *pbrAppearance = shape->pbrAppearance();
    if (appearance) {
      WbImageTexture *theTexture = appearance->texture();
      if (theTexture)
        mImageTextures.push_back(theTexture);
    } else if (pbrAppearance) {
      WbImageTexture *theTexture = pbrAppearance->baseColorMap();
      if (theTexture)
        mImageTextures.push_back(theTexture);
      theTexture = pbrAppearance->emissiveColorMap();
      if (theTexture)
        mImageTextures.push_back(theTexture);
    }
  } else {
    WbGroup *group = dynamic_cast<WbGroup *>(firstChild);
    if (group)
      findImageTextures(group);
  }

  for (int i = 0; i < mImageTextures.size(); ++i)
    connect(mImageTextures.at(i), &QObject::destroyed, this, &WbDisplay::removeImageTexture);

  // debug code - print the found materials
  // foreach (WbImageTexture *texture, mImageTextures)
  //   parsingWarn(QString("found image texture %1").arg(texture->usefulName()));
}

void WbDisplay::removeImageTexture(QObject *object) {
  mImageTextures.removeAll(static_cast<WbImageTexture *>(object));
}

void WbDisplay::clearImageTextures() {
  removeExternalTextures();
  mImageTextures.clear();
}

void WbDisplay::findImageTextures(WbGroup *group) {
  WbMFNode::Iterator i(group->children());
  while (i.hasNext()) {
    WbNode *node = i.next();
    WbShape *shape = dynamic_cast<WbShape *>(node);
    if (shape) {
      WbAppearance *appearance = shape->appearance();
      WbPbrAppearance *pbrAppearance = shape->pbrAppearance();
      if (appearance) {
        WbImageTexture *theTexture = appearance->texture();
        if (theTexture)
          mImageTextures.push_back(theTexture);
      } else if (pbrAppearance) {
        WbImageTexture *theTexture = pbrAppearance->baseColorMap();
        if (theTexture)
          mImageTextures.push_back(theTexture);
        theTexture = pbrAppearance->emissiveColorMap();
        if (theTexture)
          mImageTextures.push_back(theTexture);
      }
    } else {
      WbGroup *g = dynamic_cast<WbGroup *>(node);
      if (g)
        findImageTextures(g);
    }
  }
}

QString WbDisplay::pixelInfo(int x, int y) const {
  return QString::asprintf("pixel(%d,%d)=#%02X%02X%02X%02X", x, y, red(x, y), green(x, y), blue(x, y), alpha(x, y));
}

void WbDisplay::handleMessage(QDataStream &stream) {
  short size, x, y, w, h;
  int *px = NULL, *py = NULL;
  int id, channel;
  bool blend = false;
  unsigned char uc, format;
  char *img;
  unsigned short int cameraTag;

  unsigned char command;
  stream >> command;
  switch (command) {
    case C_DISPLAY_ATTACH_CAMERA:
      stream >> cameraTag;
      setTransparentTextureIfNeeded();
      attachCamera(cameraTag);
      break;
    case C_DISPLAY_DETACH_CAMERA:
      detachCamera();
      mUpdateRequired = true;
      break;
    case C_DISPLAY_SET_COLOR:
      stream >> mColor;
      break;
    case C_DISPLAY_SET_ALPHA:
      stream >> mAlpha;
      break;
    case C_DISPLAY_SET_OPACITY:
      stream >> mOpacity;
      break;
    case C_DISPLAY_SET_FONT: {
      unsigned int fontSize;
      // cppcheck-suppress unassignedVariable
      bool antiAliasing;
      stream >> fontSize;
      stream >> antiAliasing;
      // cppcheck-suppress knownConditionTrueFalse
      mAntiAliasing = antiAliasing == 1;
      stream >> size;
      char font[size];
      stream.readRawData(font, size);
      setFont(font, fontSize);
      break;
    }
    case C_DISPLAY_DRAW_PIXEL:
    case C_DISPLAY_DRAW_LINE:
    case C_DISPLAY_DRAW_TEXT:
    case C_DISPLAY_DRAW_RECTANGLE:
    case C_DISPLAY_DRAW_OVAL:
    case C_DISPLAY_DRAW_POLYGON:
      stream >> size;
      px = new int[size];
      py = new int[size];
      stream.readRawData(reinterpret_cast<char *>(px), size * sizeof(int));
      stream.readRawData(reinterpret_cast<char *>(py), size * sizeof(int));
      switch (command) {
        case C_DISPLAY_DRAW_PIXEL:
          drawPixel(px[0], py[0]);
          break;
        case C_DISPLAY_DRAW_LINE:
          drawLine(px[0], py[0], px[1], py[1]);
          break;
        case C_DISPLAY_DRAW_TEXT: {
          QByteArray txt;
          do {
            stream >> uc;
            txt.append(uc);
          } while (uc != 0);
          drawText(txt.constData(), px[0], py[0]);
          break;
        }
        case C_DISPLAY_DRAW_RECTANGLE:
          stream >> uc;
          drawRectangle(px[0], py[0], px[1], py[1], static_cast<bool>(uc));
          break;
        case C_DISPLAY_DRAW_OVAL:
          stream >> uc;
          drawOval(px[0], py[0], px[1], py[1], static_cast<bool>(uc));
          break;
        case C_DISPLAY_DRAW_POLYGON:
          stream >> uc;
          drawPolygon(px, py, size, static_cast<bool>(uc));
          break;
        default:
          assert(0);
          break;
      }
      delete[] px;
      delete[] py;
      mUpdateRequired = true;
      break;
    case C_DISPLAY_IMAGE_COPY: {
      stream >> id;
      stream >> x;
      stream >> y;
      stream >> w;
      stream >> h;
      // get copied data and clipped width and height
      unsigned int *data = imageCopy(x, y, w, h);
      if (data != NULL)
        mImages.push_back(new WbDisplayImage(id, w, h, data, mIsTextureTransparent));
      break;
    }
    case C_DISPLAY_IMAGE_PASTE:
      stream >> id;
      stream >> x;
      stream >> y;
      stream >> blend;
      imagePaste(id, x, y, blend == 1);
      mUpdateRequired = true;
      break;
    case C_DISPLAY_IMAGE_LOAD:
      stream >> id;
      stream >> w;
      stream >> h;
      stream >> format;
      channel = channelNumberFromPixelFormat(format);
      img = new char[channel * w * h];
      stream.readRawData(img, channel * w * h);
      imageLoad(id, w, h, img, format);
      delete[] img;
      break;
    case C_DISPLAY_IMAGE_SAVE: {
      stream >> id;
      if (id == 0) {
        // save current display image
        w = width();
        h = height();
        // get the image data and the clipped width and height
        unsigned int *data = imageCopy(0, 0, w, h);
        mSaveOrders.push_back(new WbDisplayImage(0, w, h, data, mIsTextureTransparent));
        break;
      }
      const WbDisplayImage *di = imageFind(id);
      if (di)
        mSaveOrders.push_back(new WbDisplayImage(*di));
      break;
    }
    case C_DISPLAY_IMAGE_DELETE:
      stream >> id;
      imageDelete(id);
      break;
    case C_DISPLAY_IMAGE_GET_ALL:
      mRequestImages = true;
      break;
    default:
      assert(0);
      break;
  }
}

void WbDisplay::writeAnswer(WbDataStream &stream) {
  if (mRequestImages) {
    mRequestImages = false;

    stream << tag();
    stream << (unsigned char)C_DISPLAY_IMAGE_GET_ALL;

    quint32 number = mImages.size();
    stream << number;

    stream << (quint16)width();
    stream << (quint16)height();
    stream.writeRawData(reinterpret_cast<const char *>(mImage), 4 * width() * height());

    for (unsigned i = 0; i < number; i++) {
      WbDisplayImage *di = mImages.at(i);
      stream << (qint32)di->id();
      stream << (quint16)di->width();
      stream << (quint16)di->height();
      stream.writeRawData(reinterpret_cast<const char *>(di->image()), 4 * di->width() * di->height());
    }

    stream << (qint32)mColor;
    stream << (quint8)mAlpha;
    stream << (quint8)mOpacity;
  }
  while (!mSaveOrders.empty()) {
    WbDisplayImage *di = mSaveOrders.back();
    stream << tag();
    stream << (unsigned char)C_DISPLAY_IMAGE_SAVE;
    stream << di->id();
    stream << di->width();
    stream << di->height();
    stream.writeRawData(reinterpret_cast<const char *>(di->image()), 4 * di->width() * di->height());

    mSaveOrders.pop_back();
    delete di;
  }
}

void WbDisplay::writeConfigure(WbDataStream &stream) {
  setup();

  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (unsigned short)width();
  stream << (unsigned short)height();
}

void WbDisplay::setup() {
  WbRenderingDevice::setup();

  // make sure to detach the camera and clear previous images (in case the controller has restarted)
  detachCamera();
  clearImages();

  if (width() > 0 && height() > 0) {
    delete[] mImage;
    mImage = new unsigned int[width() * height()];

    for (int j = 0; j < height(); j++)
      for (int i = 0; i < width(); i++)
        mImage[j * width() + i] = 0xFF000000;  // init with visible black color
  }
  createWrenOverlay();
}

// color in RGB format
// image in RGBA format
// drawPixel performs an "over" operation:
//  for the color channels: linear approximation cr = (1-alpha)*cr + alpha*color
//  for the alpha channel:  alpha
void WbDisplay::drawPixel(int x, int y) {
  if (mImage && x >= 0 && x < width() && y >= 0 && y < height()) {
    int offset = y * width() + x;
    fastDrawPixel(offset);
  }
}

// same than 'drawPixel' but without check and with the offset instead of x and y
void WbDisplay::fastDrawPixel(int offset) {
  if (mAlpha != 0xFF)
    setTransparentTextureIfNeeded();
  unsigned int opacityB = 0xFF - mOpacity;
  mImage[offset] = (((opacityB * SHIFT(mImage[offset], 24) + mOpacity * mAlpha) / 0xFF) << 24) +             // a
                   (((opacityB * SHIFT(mImage[offset], 16) + mOpacity * SHIFT(mColor, 16)) / 0xFF) << 16) +  // r
                   (((opacityB * SHIFT(mImage[offset], 8) + mOpacity * SHIFT(mColor, 8)) / 0xFF) << 8) +     // g
                   (((opacityB * (mImage[offset] & 0xFF) + mOpacity * (mColor & 0xFF)) / 0xFF));             // b
}

// Bresenham's line drawing algorithm with low-level optimizations
void WbDisplay::drawLine(int x0, int y0, int x1, int y1) {
  if (qMax(x0, x1) < 0 || qMax(y0, y1) < 0 || qMin(x0, x1) >= width() || qMin(y0, y1) >= height())
    return;  // out of bounds

  int dy = y1 - y0;
  int dx = x1 - x0;
  int stepx, stepy;

  if (dy < 0) {
    dy = -dy;
    stepy = -1;
  } else
    stepy = 1;

  if (dx < 0) {
    dx = -dx;
    stepx = -1;
  } else
    stepx = 1;

  dy <<= 1;  // dy is now 2*dy
  dx <<= 1;  // dx is now 2*dx

  drawPixel(x0, y0);
  if (dx > dy) {
    int fraction = dy - (dx >> 1);  // same as 2*dy - dx
    while (x0 != x1) {
      if (fraction >= 0) {
        y0 += stepy;
        fraction -= dx;  // same as fraction -= 2*dx
      }
      x0 += stepx;
      fraction += dy;  // same as fraction -= 2*dy
      drawPixel(x0, y0);
    }
  } else {
    int fraction = dx - (dy >> 1);
    while (y0 != y1) {
      if (fraction >= 0) {
        x0 += stepx;
        fraction -= dy;
      }
      y0 += stepy;
      fraction += dx;
      drawPixel(x0, y0);
    }
  }
}

// Only the visible pixels are drawn
void WbDisplay::drawRectangle(int x, int y, int w, int h, bool fill) {
  if (mAlpha != 0xFF)
    setTransparentTextureIfNeeded();
  int displayWidth = width();
  int displayHeight = height();
#ifndef NDEBUG
  int size = displayWidth * displayHeight;
#endif
  if (fill) {
    if (x < 0)
      x = 0;
    if (y < 0)
      y = 0;
    if (x + w > displayWidth)
      w = displayWidth - x;
    if (y + h > displayHeight)
      h = displayHeight - y;
    int nw = x + w;
    int nh = y + h;
    if (mOpacity != 0xFF)
      for (int j = y; j < nh; j++)
        for (int i = x; i < nw; i++)
          fastDrawPixel(j * displayWidth + i);
    else {
      if (mAlpha == 0) {
        if (w == displayWidth && h == displayHeight)  // clear the entire display
          memset(mImage, 0, w * h * sizeof(int));
        else {  // clear some part of the display
          int index = y * displayWidth + x;
          for (int j = y; j < nh; j++, index += displayWidth)
            memset(&mImage[index], 0, w * sizeof(int));
        }
      } else {
        unsigned int color = (mAlpha << 24) + mColor;
        for (int j = y; j < nh; j++)
          for (int i = x; i < nw; i++)
            mImage[j * displayWidth + i] = color;
      }
    }
  } else {  // rectangle frame only
    int nw = x + w - 1;
    int nh = y + h - 1;
    if (mOpacity != 0xFF) {
      int min = qMin(nw + 1, displayWidth);
      for (int i = qMax(0, x); i < min; i++) {
        if (y >= 0 && y < displayHeight)
          fastDrawPixel(y * displayWidth + i);
        if (nh >= 0 && nh < displayHeight)
          fastDrawPixel(nh * displayWidth + i);
      }
      min = qMin(nh, displayHeight);
      for (int i = qMax(0, y + 1); i < min; i++) {
        if (x >= 0 && x < displayWidth)
          fastDrawPixel(i * displayWidth + x);
        if (nw >= 0 && nw < displayWidth)
          fastDrawPixel(i * displayWidth + nw);
      }
    } else {
      unsigned int color = (mAlpha << 24) + mColor;
      int hStart, hEnd;
      if (x < 0)
        hStart = 0;
      else if (x >= displayWidth)
        hStart = -1;
      else
        hStart = x;
      if (nw <= 0)
        hEnd = -1;
      else if (nw >= displayWidth)
        hEnd = displayWidth - 1;
      else
        hEnd = nw;
      if (hStart >= 0 && hEnd >= 0) {
        if (y >= 0 && y < displayHeight) {  // top line
          int index = y * displayWidth;
          for (int i = hStart; i <= hEnd; i++) {
            assert(index + i < size && index + i >= 0);
            mImage[index + i] = color;
          }
        }
        if (nh > 0 && nh < displayHeight) {  // bottom line
          int index = nh * displayWidth;
          for (int i = hStart; i <= hEnd; i++) {
            assert(index + i < size && index + i >= 0);
            mImage[index + i] = color;
          }
        }
      }
      int vStart, vEnd;
      if (y < 0)
        vStart = 0;
      else if (y >= displayHeight)
        vStart = -1;
      else
        vStart = y;
      if (nh < 0)
        vEnd = -1;
      else if (nh >= displayHeight)
        vEnd = displayHeight - 1;
      else
        vEnd = nh;
      if (vStart >= 0 && vEnd >= 0) {
        if (x >= 0 && x < displayWidth) {  // left line
          for (int i = vStart; i <= vEnd; i++) {
            assert(x + i * displayWidth < size && x + i * displayWidth >= 0);
            mImage[x + i * displayWidth] = color;
          }
        }
        if (nw >= 0 && nw < displayWidth) {  // right line
          for (int i = vStart; i <= vEnd; i++) {
            assert(nw + i * displayWidth < size && nw + i * displayWidth >= 0);
            mImage[nw + i * displayWidth] = color;
          }
        }
      }
    }
  }
}

// Bresenham's midpoint circle algorithm
void WbDisplay::drawOval(int cx, int cy, int a, int b, bool fill) {
  if (a < 1 || b < 1 || cx + a < 0 || cx - a >= width() || cy + b < 0 || cy - b >= height())
    return;  // out of bounds

  // use the qint64 type here:
  //   Some values (notably r) can increase quickly the regular
  //   integer range.
  //   Otherwise, some issues can be observed already with circles
  //   having a radius of around 520
  qint64 x = 0, mx1 = 0, mx2 = 0, my1 = 0, my2 = 0, old_my2, aq, bq, dx, dy, r, rx, ry;
  mx1 = cx - a;
  my1 = cy;
  mx2 = cx + a;
  my2 = cy;
  aq = a * a;
  bq = b * b;
  dx = aq << 1;
  dy = bq << 1;
  r = a * bq;
  rx = r << 1;
  ry = 0;
  x = a;
  old_my2 = -2;

  if (fill)
    drawLine(cx - a, cy, cx + a, cy);
  else {
    drawPixel(cx + a, cy);
    drawPixel(cx - a, cy);
  }
  while (x > 0) {
    if (r > 0) {
      my1++;
      my2--;
      ry += dx;
      r -= ry;
    }
    if (r <= 0) {
      x--;
      mx1++;
      mx2--;
      rx -= dy;
      r += rx;
    }
    if (fill) {
      if (old_my2 != my2) {
        drawLine(mx1, my1, mx2, my1);
        drawLine(mx1, my2, mx2, my2);
      }
      old_my2 = my2;
    } else {
      drawPixel(mx1, my1);
      drawPixel(mx1, my2);
      if (x != 0) {
        drawPixel(mx2, my1);
        drawPixel(mx2, my2);
      }
    }
  }
}

void WbDisplay::drawText(const char *txt, int x, int y) {
  if (x >= width() || y >= height())
    return;  // out of the uppest bounds
  int cx = x;
  int cy = y;
  int l = strlen(txt);
  wchar_t *text = new wchar_t[l + 1];
#ifdef _WIN32  // mbstowcs doesn't work properly on Windows
  l = MultiByteToWideChar(CP_UTF8, 0, txt, -1, text, l + 1) - 1;
#else
  // cppcheck-suppress uninitdata
  l = mbstowcs(text, txt, l + 1);
#endif
  int fontSize = mDisplayFont->fontSize();
  for (int i = 0; i < l; i++) {
    if (text[i] == L'\n') {  // carriage return
      cy += mDisplayFont->verticalSpace();
      cx = x;
    } else {  // draw char
      int size = fontSize;
      if (cx + fontSize >= 0 && cx < width() && cy + fontSize >= 0 && cy < height()) {  // if char is visible
        size = drawChar(text[i], cx, cy);
      }
      cx += size;
    }
  }
  delete[] text;
}

int WbDisplay::drawChar(unsigned long c, int x, int y) {
  int characterWidth, characterHeight, verticalOffset, horizontalOffset, transparencyFactor, horizontalAdvance, pitch;
  unsigned char *buffer = mDisplayFont->generateCharBuffer(c, mAntiAliasing, &characterWidth, &characterHeight, &verticalOffset,
                                                           &horizontalOffset, &transparencyFactor, &horizontalAdvance, &pitch);
  if (c == ' ')
    return horizontalAdvance;
  if (!buffer) {
    warn(tr("Error while generating character '%1'.").arg(c));
    return 0;
  }
  const int w = width();
  const int max = w * height();
  x = x + horizontalOffset;
  y = y - verticalOffset + mDisplayFont->fontSize();

  const int boundedCharacterWidth = qMin(characterWidth, w - x);
  const int boundedCharacterHeight = qMin(characterHeight, height() - y);

  unsigned char originalOpacity = mOpacity;
  const int offset = y * w + x;

  for (int j = 0; j < boundedCharacterHeight; ++j) {
    int t = offset + j * w;
    const int bufferOffset = j * characterWidth;
    for (int i = 0; i < boundedCharacterWidth; ++i, ++t) {
      if (t < 0)
        continue;
      if (t >= max)
        break;

      if (mAntiAliasing) {
        const unsigned char pixelValue = buffer[bufferOffset + i];
        if (pixelValue > 0) {
          mOpacity = originalOpacity * pixelValue / transparencyFactor;
          fastDrawPixel(t);
        }
      } else {
        // get bit from current byte corresponding to current pixel
        const unsigned char *row = &buffer[pitch * j];
        const unsigned char cValue = row[i >> 3];
        if ((cValue & (128 >> (i & 7))) != 0)
          fastDrawPixel(t);
      }
    }
  }
  mOpacity = originalOpacity;
  return horizontalAdvance;
}

void WbDisplay::setFont(char *font, unsigned int size) {
  bool fileFound = false;
  QString filename = WbStandardPaths::fontsPath() + QString(font) + ".ttf";
  if (QFile::exists(filename))
    fileFound = true;
  else {
    filename = WbProject::current()->path() + "fonts/" + QString(font) + ".ttf";
    if (QFile::exists(filename))
      fileFound = true;
  }

  if (fileFound) {
    mDisplayFont->setFont(filename, size);
    if (!mDisplayFont->error().isEmpty())
      warn(mDisplayFont->error());
  } else
    warn(tr("Invalid '%1' font.").arg(font));
}

void WbDisplay::drawPolygon(const int *px, const int *py, int size, bool fill) {
  int minX = INT_MAX, minY = INT_MAX, maxX = INT_MIN, maxY = INT_MIN, i;
  for (i = 0; i < size; i++) {
    if (minX > px[i])
      minX = px[i];
    if (minY > py[i])
      minY = py[i];
    if (maxX < px[i])
      maxX = px[i];
    if (maxY < py[i])
      maxY = py[i];
  }
  if (minX >= width() || minY >= height() || maxX < 0 || maxY < 0)
    return;  // out of bounds

  int nodeX[size], pixelY, swap;
  int horizontalEdgesCounter = 0, horizontalEdges[3 * size];

  // Loop through visible rows of the polygon
  for (pixelY = qMax(0, minY); pixelY <= qMin(height() - 1, maxY); pixelY++) {
    //  Build a list of nodes.
    int nodes = 0;
    int j = size - 1;
    for (i = 0; i < size; i++) {
      const bool edgeCrossesTheHorizontalLine = (py[i] < pixelY && py[j] >= pixelY) || (py[j] < pixelY && py[i] >= pixelY);
      if (edgeCrossesTheHorizontalLine)
        nodeX[nodes++] = px[i] + (pixelY - py[i]) * (px[j] - px[i]) / (py[j] - py[i]);
      else if (py[i] == pixelY && py[j] == pixelY) {  // the edge i->j is horizontal
        horizontalEdges[horizontalEdgesCounter++] = pixelY;
        horizontalEdges[horizontalEdgesCounter++] = qMin(px[i], px[j]);
        horizontalEdges[horizontalEdgesCounter++] = qMax(px[i], px[j]);
      }
      j = i;
    }

    //  Sort the nodes, via a simple “Bubble” sort.
    i = 0;
    while (i < nodes - 1) {
      if (nodeX[i] > nodeX[i + 1]) {
        swap = nodeX[i];
        nodeX[i] = nodeX[i + 1];
        nodeX[i + 1] = swap;
        if (i)
          i--;
      } else
        i++;
    }

    // Merge adjacent pairs
    if (nodes > 2) {
      i = 2;
      do {
        if (nodeX[i - 1] == nodeX[i])
          nodeX[i]++;
        i += 2;
      } while (i < nodes);
    }

    //  Fill the pixels between node pairs.
    for (i = 0; i < nodes; i += 2)
      if (fill)
        drawRectangle(nodeX[i], pixelY, nodeX[i + 1] - nodeX[i] + 1, 1, true);
      else {
        drawPixel(nodeX[i], pixelY);
        if (nodeX[i] != nodeX[i + 1])
          drawPixel(nodeX[i + 1], pixelY);
      }
  }
  // Draws the horizontal edges
  const int numberOfEdges = horizontalEdgesCounter / 3;
  for (i = 0; i < numberOfEdges; i++) {
    int k = 3 * i;
    const int y = horizontalEdges[k++];
    const int leftBound = horizontalEdges[k++];
    const int rightBound = horizontalEdges[k];
    for (int j = leftBound; j <= rightBound; j++)
      drawPixel(j, y);
  }
}

unsigned int *WbDisplay::imageCopy(short int x, short int y, short int &w, short int &h) {
  int clippedX = qMax((short int)0, x);
  int clippedY = qMax((short int)0, y);
  w = qMin(width() - clippedX, w - (clippedX - x));
  h = qMin(height() - clippedY, h - (clippedY - y));
  if (clippedX >= width() || clippedY >= height() || w < 1 || h < 1)
    return NULL;
  const int size = w * h * sizeof(unsigned int);
  unsigned int *clippedImage = new unsigned int[w * h];
  if (mAttachedCamera) {
    // blend camera background image and display image
    const unsigned int *const cameraImage = reinterpret_cast<const unsigned int *>(mAttachedCamera->constImage());
    int destIndex = 0;
    int displayPixel, displayAlpha, oneMinusDisplayAlpha, cameraPixel;
    for (int j = 0; j < h; j++) {
      int srcRowIndex = (clippedY + j) * width();
      for (int i = 0; i < w; i++, destIndex++) {
        int srcPixelIndex = srcRowIndex + clippedX + i;
        displayPixel = mImage[srcPixelIndex];
        displayAlpha = SHIFT(displayPixel, 24);
        oneMinusDisplayAlpha = 0xFF - displayAlpha;
        cameraPixel = cameraImage[srcPixelIndex];
        clippedImage[srcPixelIndex] = 0xFF000000;
        for (int k = 0; k < 3; k++)
          clippedImage[srcPixelIndex] +=
            (((oneMinusDisplayAlpha * SHIFT(cameraPixel, 8 * k) + displayAlpha * SHIFT(displayPixel, 8 * k)) / 0xFF) << 8 * k);
      }
    }
  } else if (clippedX == 0 && w == width()) {
    // special case: strict copy
    const unsigned int *srcImage = mImage;
    if (clippedY > 0)
      srcImage += clippedY * width();
    memcpy(clippedImage, srcImage, size);
  } else {
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++)
        clippedImage[j * w + i] = mImage[((clippedY + j) * width()) + (clippedX + i)];
    }
  }
  return clippedImage;
}

void WbDisplay::imagePaste(int id, int x, int y, bool blend) {
  if (x >= width() || y >= height())
    return;
  WbDisplayImage *subImage = NULL;
  for (int i = 0; i < mImages.size(); i++)
    if (mImages.at(i)->id() == id) {
      subImage = mImages.at(i);
      break;
    }
  if (!subImage || !subImage->image())
    return;

  if (subImage->isTransparent())
    setTransparentTextureIfNeeded();

  int clippedX = qMax(0, x);
  int clippedY = qMax(0, y);
  int clippedWidth = qMin(width() - clippedX, subImage->width() - (clippedX - x));
  int clippedHeight = qMin(height() - clippedY, subImage->height() - (clippedY - y));
  if (clippedX >= width() || clippedY >= height() || clippedWidth < 1 || clippedHeight < 1)
    return;

  int offsetMainImage, offsetNewImage;
  unsigned int *subImageValues = subImage->image();
  int subImageWidth = subImage->width();
  int w = width();

  if (blend) {
    for (int j = 0; j < clippedHeight; j++) {
      for (int i = 0; i < clippedWidth; i++) {
        offsetMainImage = (clippedY + j) * w + clippedX + i;
        offsetNewImage = ((clippedY - y) + j) * subImageWidth + (clippedX - x) + i;
        int oldPixel = mImage[offsetMainImage];
        unsigned char oldAlpha = SHIFT(oldPixel, 24);
        int newPixel = subImageValues[offsetNewImage];
        unsigned char newAlpha = SHIFT(newPixel, 24);
        unsigned char oneMinusNewAlpha = 0xFF - newAlpha;
        unsigned char alphaValue = qMin(0xFF, newAlpha + oldAlpha);
        mImage[offsetMainImage] = alphaValue << 24;
        for (int k = 0; k < 3; k++)
          mImage[offsetMainImage] +=
            (((oneMinusNewAlpha * SHIFT(oldPixel, 8 * k) + newAlpha * SHIFT(newPixel, 8 * k)) / 0xFF) << 8 * k);
      }
    }
  } else {
    if (subImage->width() == w && x == 0) {
      // strict copy of the image
      unsigned int *destImage = mImage;
      unsigned int *sourceImage = subImageValues;
      if (y > 0)
        destImage += w * y;
      else if (y < 0)
        sourceImage -= w * y;
      memcpy(destImage, sourceImage, sizeof(unsigned int) * w * clippedHeight);
    } else {
      for (int j = 0; j < clippedHeight; j++) {
        for (int i = 0; i < clippedWidth; i++) {
          offsetMainImage = (clippedY + j) * w + clippedX + i;
          offsetNewImage = ((clippedY - y) + j) * subImageWidth + (clippedX - x) + i;
          mImage[offsetMainImage] = subImageValues[offsetNewImage];
        }
      }
    }
  }
}

void WbDisplay::imageLoad(int id, int w, int h, void *data, int format) {
  const int nbPixel = w * h;
  unsigned int *clippedImage = new unsigned int[nbPixel];
  bool isTransparent = false;

  // convert to BGRA
  if (format == WB_IMAGE_BGRA)
    memcpy(clippedImage, data, nbPixel * 4);
  else if (format == WB_IMAGE_ARGB) {
    const unsigned char *dataUC = static_cast<unsigned char *>(data);
    for (int i = 0; i < nbPixel; i++) {
      const int offset = 4 * i;
      isTransparent = (dataUC[offset] & 0XFF) != 0xFF;
      clippedImage[i] = (dataUC[offset] << 24) | (dataUC[offset + 1] << 16) | (dataUC[offset + 2] << 8) | dataUC[offset + 3];
    }
  } else if (format == WB_IMAGE_RGB) {
    const unsigned char *dataUC = static_cast<unsigned char *>(data);
    for (int i = 0; i < nbPixel; i++) {
      const int offset = 3 * i;
      clippedImage[i] = 0xFF000000 | (dataUC[offset] << 16) | (dataUC[offset + 1] << 8) | dataUC[offset + 2];
    }
  } else if (format == WB_IMAGE_RGBA) {
    const unsigned char *dataUC = static_cast<unsigned char *>(data);
    for (int i = 0; i < nbPixel; i++) {
      const int offset = 4 * i;
      isTransparent = (dataUC[offset + 3] & 0xFF) != 0xFF;
      clippedImage[i] = (dataUC[offset + 3] << 24) | (dataUC[offset] << 16) | (dataUC[offset + 1] << 8) | dataUC[offset + 2];
    }
  } else if (format == WB_IMAGE_ABGR) {
    const unsigned char *dataUC = static_cast<unsigned char *>(data);
    for (int i = 0; i < nbPixel; i++) {
      const int offset = 4 * i;
      isTransparent = (dataUC[offset] & 0xFF) != 0xFF;
      clippedImage[i] = (dataUC[offset] << 24) | (dataUC[offset + 3] << 16) | (dataUC[offset + 2] << 8) | dataUC[offset + 1];
    }
  } else
    assert(0);

  mImages.push_back(new WbDisplayImage(id, w, h, clippedImage, isTransparent));
  // cppcheck-suppress memleak
}

void WbDisplay::imageDelete(int id) {
  for (int i = 0; i < mImages.size(); i++) {
    WbDisplayImage *di = mImages.at(i);
    if (di->id() == id) {
      mImages.remove(i);
      delete di;
      return;
    }
  }
}

WbDisplayImage *WbDisplay::imageFind(int id) {
  for (int i = 0; i < mImages.size(); i++)
    if (mImages.at(i)->id() == id)
      return mImages.at(i);
  return NULL;
}

void WbDisplay::createWrenObjects() {
  WbRenderingDevice::createWrenObjects();
  createWrenOverlay();
}

void WbDisplay::createWrenOverlay() {
  WbWrenOpenGlContext::makeWrenCurrent();

  QStringList previousSettings;
  if (mOverlay)
    previousSettings = mOverlay->perspective();
  delete mOverlay;

  mNeedToSetExternalTextures = true;
  mOverlay = new WbWrenTextureOverlay(mImage, width(), height(), WbWrenTextureOverlay::TEXTURE_TYPE_BGRA,
                                      WbWrenTextureOverlay::OVERLAY_TYPE_DISPLAY, NULL, 1.0, false, mIsTextureTransparent);
  connect(mOverlay, &WbWrenTextureOverlay::textureUpdated, this, &WbRenderingDevice::textureUpdated);
  connect(mOverlay, &QObject::destroyed, this, &WbDisplay::removeExternalTextures);

  if (!previousSettings.isEmpty())
    mOverlay->restorePerspective(previousSettings, areOverlaysEnabled());
  else
    mOverlay->setVisible(true, areOverlaysEnabled());

  emit textureIdUpdated(mOverlay->textureGLId(), MAIN_TEXTURE);

  WbWrenOpenGlContext::doneWren();
}

void WbDisplay::removeExternalTextures() {
  // first remove all the references to deleted external textures
  for (int i = 0; i < mImageTextures.size(); ++i)
    mImageTextures.at(i)->removeExternalTexture();
  // then, trigger the appearance update
  // two steps needed for PBRAppearance nodes if both baseColorMap and emissiveColorMap are defined
  for (int i = 0; i < mImageTextures.size(); ++i)
    emit mImageTextures.at(i)->changed();
}

void WbDisplay::setTransparentTextureIfNeeded() {
  if (!mIsTextureTransparent) {
    mIsTextureTransparent = true;
    if (areWrenObjectsInitialized())
      createWrenOverlay();
  }
}

void WbDisplay::attachCamera(WbDeviceTag cameraTag) {
  WbDevice *device = robot()->findDevice(cameraTag);
  assert(device);
  WbCamera *camera = static_cast<WbCamera *>(device);
  assert(camera);
  WrTexture *texture = camera->getWrenTexture();
  if (texture != NULL && mAttachedCamera != camera) {
    if (isWindowActive()) {
      if (mAttachedCamera)
        mAttachedCamera->enableExternalWindowForAttachedCamera(false);
      camera->enableExternalWindowForAttachedCamera(true);
    }
    emit attachedCameraChanged(mAttachedCamera, camera);
    mAttachedCamera = camera;
    connect(mAttachedCamera, &WbCamera::destroyed, this, &WbDisplay::detachCamera);
    mOverlay->setBackgroundTexture(texture);

    foreach (WbImageTexture *imageTexture, mImageTextures)
      imageTexture->setBackgroundTexture(texture);

    emit textureIdUpdated(mOverlay->backgroundTextureGLId(), BACKGROUND_TEXTURE);
    // clear the alpha channel so that the background image is visible
    const int size = width() * height();
    for (int i = 0; i < size; i++) {
      if (mImage[i] == 0xFF000000)  // default value: black
        mImage[i] = 0x0;            // new value: transparent
    }

    mUpdateRequired = true;
  }
}

void WbDisplay::detachCamera() {
  if (mAttachedCamera != NULL) {
    mOverlay->unsetBackgroundTexture();

    foreach (WbImageTexture *imageTexture, mImageTextures)
      imageTexture->unsetBackgroundTexture();

    if (isWindowActive()) {
      mAttachedCamera->enableExternalWindowForAttachedCamera(false);
      emit attachedCameraChanged(mAttachedCamera, NULL);
    }
    emit textureIdUpdated(0, BACKGROUND_TEXTURE);
    mAttachedCamera = NULL;
  }
}

void WbDisplay::enableExternalWindow(bool enabled) {
  if (mAttachedCamera)
    mAttachedCamera->enableExternalWindowForAttachedCamera(enabled);
  WbRenderingDevice::enableExternalWindow(enabled);
}

int WbDisplay::shiftedChannel(int x, int y, int shift) const {
  if (mImage) {
    int index = y * width() + x;
    return SHIFT(mImage[index], shift);
  }
  return 0;
}

void WbDisplay::postPhysicsStep() {
  WbSolidDevice::postPhysicsStep();

  if (!hasBeenSetup() || !mUpdateRequired)
    return;

  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::DEVICE_RENDERING, deviceName());

  mOverlay->requestUpdateTexture();

  // if the texture is displayed in the 3D view, the
  // copy should be done at this moment.
  // for example, camera should see the texture modifications,
  // even in fast mode
  if (mImageTextures.size() > 0)
    mOverlay->updateTexture();

  if (mNeedToSetExternalTextures) {
    mNeedToSetExternalTextures = false;
    foreach (WbImageTexture *texture, mImageTextures)
      texture->setExternalTexture(mOverlay->texture(), reinterpret_cast<unsigned char *>(mImage),
                                  static_cast<double>(width()) / wr_texture_get_width(mOverlay->texture()),
                                  static_cast<double>(height()) / wr_texture_get_height(mOverlay->texture()));
  }

  if (log)
    log->stopMeasure(WbPerformanceLog::DEVICE_RENDERING, deviceName());

  mUpdateRequired = false;
}

void WbDisplay::reset(const QString &id) {
  WbRenderingDevice::reset(id);

  delete[] mImage;
  mImage = NULL;
  mColor = 0xFFFFFF;
  mAlpha = 0xFF;
  mOpacity = 0xFF;
  mAntiAliasing = true;
  mUpdateRequired = true;
  mRequestImages = false;

  setup();
}
