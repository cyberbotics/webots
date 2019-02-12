// Copyright 1996-2018 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbImageTexture.hpp"

#include "WbAppearance.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbLog.hpp"
#include "WbMFString.hpp"
#include "WbMathsUtilities.hpp"
#include "WbPreferences.hpp"
#include "WbRgb.hpp"
#include "WbSFBool.hpp"
#include "WbUrl.hpp"
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"

#include <QtGui/QImageReader>

#include <wren/gl_state.h>
#include <wren/material.h>
#include <wren/texture.h>
#include <wren/texture_2d.h>
#include <wren/texture_transform.h>

void WbImageTexture::init() {
  mWrenTexture = NULL;
  mWrenBackgroundTexture = NULL;
  mWrenTextureTransform = NULL;
  mExternalTexture = false;
  mExternalTextureRatio.setXy(1.0, 1.0);
  mExternalTextureData = NULL;
  mContainerField = "";
  mImage = NULL;
  mWrenTextureIndex = 0;
  mIsMainTextureTransparent = true;

  mUrl = findMFString("url");
  mRepeatS = findSFBool("repeatS");
  mRepeatT = findSFBool("repeatT");
  mFiltering = findSFInt("filtering");
}

WbImageTexture::WbImageTexture(WbTokenizer *tokenizer) : WbBaseNode("ImageTexture", tokenizer) {
  init();
}

WbImageTexture::WbImageTexture(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbImageTexture::WbImageTexture(const WbImageTexture &other) : WbBaseNode(other) {
  init();
}

WbImageTexture::~WbImageTexture() {
  destroyWrenTexture();
}

void WbImageTexture::preFinalize() {
  WbBaseNode::preFinalize();

  updateUrl();
  updateRepeatS();
  updateRepeatT();
  updateFiltering();
}

void WbImageTexture::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mUrl, &WbMFString::changed, this, &WbImageTexture::updateUrl);
  connect(mRepeatS, &WbSFBool::changed, this, &WbImageTexture::updateRepeatS);
  connect(mRepeatT, &WbSFBool::changed, this, &WbImageTexture::updateRepeatT);
  connect(mFiltering, &WbSFInt::changed, this, &WbImageTexture::updateFiltering);

  if (!WbWorld::instance()->isLoading())
    emit changed();
}

static void blurred(QImage &result, int radius) {
  // blur filter source: https://stackoverflow.com/a/7936933/2210777
  int tab[] = {14, 10, 8, 6, 5, 5, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2};
  int alpha = (radius < 1) ? 16 : (radius > 17) ? 1 : tab[radius - 1];

  const int r1 = 0;
  const int r2 = result.height() - 1;
  const int c1 = 0;
  const int c2 = result.width() - 1;
  const int bpl = result.bytesPerLine();
  int rgba[4];
  unsigned char *p;

  for (int col = c1; col <= c2; ++col) {
    p = result.scanLine(r1) + col * 4;
    for (int i = 0; i < 4; ++i)
      rgba[i] = p[i] << 4;

    p += bpl;
    for (int j = r1; j < r2; ++j, p += bpl)
      for (int i = 0; i < 4; ++i)
        p[i] = (rgba[i] += ((p[i] << 4) - rgba[i]) * alpha / 16) >> 4;
  }

  for (int row = r1; row <= r2; ++row) {
    p = result.scanLine(row) + c1 * 4;
    for (int i = 0; i < 4; ++i)
      rgba[i] = p[i] << 4;

    p += 4;
    for (int j = c1; j < c2; ++j, p += 4)
      for (int i = 0; i < 4; ++i)
        p[i] = (rgba[i] += ((p[i] << 4) - rgba[i]) * alpha / 16) >> 4;
  }

  for (int col = c1; col <= c2; ++col) {
    p = result.scanLine(r2) + col * 4;
    for (int i = 0; i < 4; ++i)
      rgba[i] = p[i] << 4;

    p -= bpl;
    for (int j = r1; j < r2; ++j, p -= bpl)
      for (int i = 0; i < 4; ++i)
        p[i] = (rgba[i] += ((p[i] << 4) - rgba[i]) * alpha / 16) >> 4;
  }

  for (int row = r1; row <= r2; ++row) {
    p = result.scanLine(row) + c2 * 4;
    for (int i = 0; i < 4; ++i)
      rgba[i] = p[i] << 4;

    p -= 4;
    for (int j = c1; j < c2; ++j, p -= 4)
      for (int i = 0; i < 4; ++i)
        p[i] = (rgba[i] += ((p[i] << 4) - rgba[i]) * alpha / 16) >> 4;
  }
}

void WbImageTexture::updateWrenTexture() {
  destroyWrenTexture();

  QString filePath(path());
  if (filePath.isEmpty())
    return;

  // Only load the image from disk if the texture isn't already in the cache
  WrTexture2d *texture = wr_texture_2d_copy_from_cache(filePath.toUtf8().constData());
  if (!texture) {
    QImageReader imageReader(filePath);
    QSize textureSize = imageReader.size();
    const int imageWidth = textureSize.width();
    const int imageHeight = textureSize.height();
    int width = WbMathsUtilities::nextPowerOf2(imageWidth);
    int height = WbMathsUtilities::nextPowerOf2(imageHeight);
    if (width != imageWidth || height != imageHeight)
      WbLog::warning(tr("Texture image size of '%1' is not a power of two: rescaling it from %2x%3 to %4x%5.")
                       .arg(filePath)
                       .arg(imageWidth)
                       .arg(imageHeight)
                       .arg(width)
                       .arg(height));

    const int quality = WbPreferences::instance()->value("OpenGL/textureQuality", 2).toInt();
    const int divider = 4 * pow(0.5, quality);      // 0: 4, 1: 2, 2: 1
    const int minResolution = pow(2, 9 + quality);  // 0: 512, 1: 1024, 2: 2048
    if (divider != 1) {
      if (width >= minResolution)
        width /= divider;
      if (height >= minResolution)
        height /= divider;
    }

    delete mImage;
    mImage = new QImage();
    if (imageReader.read(mImage)) {
      mIsMainTextureTransparent = mImage->pixelFormat().alphaUsage() == QPixelFormat::UsesAlpha;

      if (mImage->format() != QImage::Format_ARGB32) {
        QImage tmp = mImage->convertToFormat(QImage::Format_ARGB32);
        mImage->swap(tmp);
      }

      if (mImage->width() != width || mImage->height() != height) {
        // Qt::SmoothTransformation alterates the alpha channel.
        // Qt::FastTransformation creates alias effects.
        // Blurring the image before scaling it without filter (Qt::FastTransformation) is the solution given in litterature,
        // for example here:
        // - https://stackoverflow.com/questions/49879466/downscaling-images-using-bilinear-and-bicubic-algorithms
        const int blurRatio = qMax(0, qMax(mImage->width() / width, mImage->height() / height) - 1);
        blurred(*mImage, blurRatio);
        QImage tmp = mImage->scaled(width, height, Qt::IgnoreAspectRatio, Qt::FastTransformation);
        mImage->swap(tmp);
      }

      WbWrenOpenGlContext::makeWrenCurrent();

      texture = wr_texture_2d_new();
      wr_texture_set_size(WR_TEXTURE(texture), mImage->width(), mImage->height());
      wr_texture_set_translucent(WR_TEXTURE(texture), mIsMainTextureTransparent);
      wr_texture_2d_set_data(texture, reinterpret_cast<const char *>(mImage->bits()));
      wr_texture_2d_set_file_path(texture, filePath.toUtf8().constData());
      wr_texture_setup(WR_TEXTURE(texture));

      WbWrenOpenGlContext::doneWren();
    } else
      warn(tr("Cannot load texture '%1': %2.").arg(filePath).arg(imageReader.errorString()));
  } else
    mIsMainTextureTransparent = wr_texture_is_translucent(WR_TEXTURE(texture));

  mWrenTexture = WR_TEXTURE(texture);
}

void WbImageTexture::destroyWrenTexture() {
  if (!mExternalTexture)
    wr_texture_delete(WR_TEXTURE(mWrenTexture));

  wr_texture_transform_delete(mWrenTextureTransform);

  mWrenTexture = NULL;
  mWrenTextureTransform = NULL;

  delete mImage;
  mImage = NULL;
}

void WbImageTexture::updateUrl() {
  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  int n = mUrl->size();
  for (int i = 0; i < n; i++) {
    QString item = mUrl->item(i);
    mUrl->setItem(i, item.replace("\\", "/"));
  }

  updateWrenTexture();

  if (isPostFinalizedCalled())
    emit changed();
}

void WbImageTexture::updateRepeatS() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbImageTexture::updateRepeatT() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbImageTexture::updateFiltering() {
  if (WbFieldChecker::checkIntIsNonNegative(this, mFiltering, 0))
    return;

  int maxHardwareAfLevel = wr_gl_state_max_texture_anisotropy();
  int maxFiltering = 1;
  // Find integer log2 of maxHardwareAfLevel to transcribe to user filtering level
  while (maxHardwareAfLevel >>= 1)
    ++maxFiltering;
  // The filtering level has an upper bound defined by the maximum supported anisotropy level.
  // A warning is not produced here because the maximum anisotropy level is not up to the user
  // and may be repeatedly shown even though a minimum requirement warning was already given.
  int filtering = mFiltering->value();
  if (filtering > maxFiltering) {
    mFiltering->blockSignals(true);
    mFiltering->setValue(std::min(4, maxFiltering));
    mFiltering->blockSignals(false);
  }

  if (isPostFinalizedCalled())
    emit changed();
}

void WbImageTexture::modifyWrenMaterial(WrMaterial *wrenMaterial, const int mainTextureIndex,
                                        const int backgroundTextureIndex) {
  if (!wrenMaterial)
    return;

  mWrenTextureIndex = mainTextureIndex;
  wr_material_set_texture(wrenMaterial, WR_TEXTURE(mWrenTexture), mWrenTextureIndex);
  if (mWrenTexture) {
    wr_texture_set_translucent(WR_TEXTURE(mWrenTexture), mIsMainTextureTransparent);
    wr_material_set_texture_wrap_s(
      wrenMaterial, mRepeatS->value() ? WR_TEXTURE_WRAP_MODE_REPEAT : WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, mWrenTextureIndex);
    wr_material_set_texture_wrap_t(
      wrenMaterial, mRepeatT->value() ? WR_TEXTURE_WRAP_MODE_REPEAT : WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, mWrenTextureIndex);
    wr_material_set_texture_anisotropy(wrenMaterial, 1 << (mFiltering->value() - 1), mWrenTextureIndex);
    wr_material_set_texture_enable_interpolation(wrenMaterial, mFiltering->value(), mWrenTextureIndex);
    wr_material_set_texture_enable_mip_maps(wrenMaterial, mFiltering->value(), mWrenTextureIndex);

    if (mExternalTexture && !(static_cast<WbAppearance *>(parent())->textureTransform())) {
      wr_texture_transform_delete(mWrenTextureTransform);
      mWrenTextureTransform = wr_texture_transform_new();
      wr_texture_transform_set_scale(mWrenTextureTransform, mExternalTextureRatio.x(), mExternalTextureRatio.y());
      wr_material_set_texture_transform(wrenMaterial, mWrenTextureTransform);
    }
  }

  wr_material_set_texture(wrenMaterial, WR_TEXTURE(mWrenBackgroundTexture), backgroundTextureIndex);
  if (mWrenBackgroundTexture) {
    // background texture can't be transparent
    wr_texture_set_translucent(mWrenBackgroundTexture, false);

    // if there's an opaque background texture, we can't treat the foreground texture as opaque, as we're going to alpha blend
    // them in the shader anyway
    if (mWrenTexture)
      wr_texture_set_translucent(WR_TEXTURE(mWrenTexture), false);

    wr_material_set_texture_wrap_s(wrenMaterial,
                                   mRepeatS->value() ? WR_TEXTURE_WRAP_MODE_REPEAT : WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE,
                                   backgroundTextureIndex);
    wr_material_set_texture_wrap_t(wrenMaterial,
                                   mRepeatT->value() ? WR_TEXTURE_WRAP_MODE_REPEAT : WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE,
                                   backgroundTextureIndex);
    wr_material_set_texture_enable_interpolation(wrenMaterial, false, backgroundTextureIndex);
    wr_material_set_texture_enable_mip_maps(wrenMaterial, false, backgroundTextureIndex);
  }
}

void WbImageTexture::setExternalTexture(WrTexture *texture, unsigned char *image, double ratioX, double ratioY) {
  destroyWrenTexture();

  mExternalTexture = true;
  mExternalTextureRatio.setXy(ratioX, ratioY);
  mExternalTextureData = image;

  mWrenTexture = texture;

  emit changed();
}

void WbImageTexture::removeExternalTexture() {
  destroyWrenTexture();

  mExternalTexture = false;
  mExternalTextureRatio.setXy(1.0, 1.0);

  updateWrenTexture();

  emit changed();
}

void WbImageTexture::setBackgroundTexture(WrTexture *backgroundTexture) {
  mWrenBackgroundTexture = backgroundTexture;
  emit changed();
}

void WbImageTexture::unsetBackgroundTexture() {
  mWrenBackgroundTexture = NULL;
  emit changed();
}

bool WbImageTexture::isTransparent() const {
  if (mWrenTexture)
    return wr_texture_is_translucent(WR_TEXTURE(mWrenTexture));
  return false;
}

int WbImageTexture::width() const {
  if (mWrenTexture)
    return wr_texture_get_width(WR_TEXTURE(mWrenTexture));
  return 0;
}

int WbImageTexture::height() const {
  if (mWrenTexture)
    return wr_texture_get_height(WR_TEXTURE(mWrenTexture));
  return 0;
}

int WbImageTexture::filtering() const {
  return mFiltering->value();
}

void WbImageTexture::pickColor(WbRgb &pickedColor, const WbVector2 &uv) const {
  if (!mWrenTexture)
    return;

  int w = width();
  int h = height();

  const unsigned char *data = NULL;
  if (mExternalTexture) {
    w *= mExternalTextureRatio.x();
    h *= mExternalTextureRatio.y();
    data = mExternalTextureData;
  } else if (mImage)
    data = mImage->bits();
  else {
    pickedColor.setValue(1.0, 1.0, 1.0);
    return;
  }

  double u = uv.x();
  double v = uv.y();

  // bound uv into 0.0 and 1.0 according to the repeatX fields
  if (mRepeatS->value()) {
    u = fmod(u, 1.0);
    if (u < 0.0)
      u += 1.0;
  } else
    u = qBound(0.0, u, 1.0);

  if (mRepeatT->value()) {
    v = fmod(v, 1.0);
    if (v < 0.0)
      v += 1.0;
  } else
    v = qBound(0.0, v, 1.0);

  int index = 4 * ((int)(v * h) * w + (int)(u * w));
  pickedColor.setByteValue((int)data[index + 2], (int)data[index + 1], (int)data[index]);

  // debug
  // printf("pickedColor (u=%f, v=%f): (r=%f g=%f b=%f)\n", u, v, pickedColor.red(), pickedColor.green(), pickedColor.blue());
}

QString WbImageTexture::path() {
  return WbUrl::computePath(this, "url", mUrl, 0);
}

void WbImageTexture::setContainerField(QString &field) {
  if (mContainerField.isEmpty())
    mContainerField = QString(field);
}

void WbImageTexture::exportNodeFields(WbVrmlWriter &writer) const {
  // export to ./textures folder relative to writer path
  WbField urlFieldCopy(*findField("url", true));
  for (int i = 0; i < mUrl->size(); ++i) {
    writer.addTextureToList(mUrl->item(i), WbUrl::computePath(this, "url", mUrl, i));
    if (writer.isWritingToFile()) {
      QString newUrl = WbUrl::exportTexture(this, mUrl, i, writer);
      dynamic_cast<WbMFString *>(urlFieldCopy.value())->setItem(i, newUrl);
    } else if (writer.isProto())
      dynamic_cast<WbMFString *>(urlFieldCopy.value())->setItem(i, WbUrl::computePath(this, "url", mUrl, i));
  }
  urlFieldCopy.write(writer);
  findField("repeatS", true)->write(writer);
  findField("repeatT", true)->write(writer);

  if (writer.isX3d())
    writer << " containerField=\'" << mContainerField << "\' origChannelCount=\'3\'";
}

void WbImageTexture::exportNodeSubNodes(WbVrmlWriter &writer) const {
  int filtering = mFiltering->value();
  if (writer.isX3d() && filtering > 0)
    writer << "<TextureProperties anisotropicDegree=\"" << (1 << (filtering - 1))
           << "\" generateMipMaps=\"true\" minificationFilter=\"AVG_PIXEL\" magnificationFilter=\"AVG_PIXEL\"/>";
  WbBaseNode::exportNodeSubNodes(writer);
}
