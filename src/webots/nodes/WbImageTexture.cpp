// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbAbstractAppearance.hpp"
#include "WbApplication.hpp"
#include "WbApplicationInfo.hpp"
#include "WbDownloader.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbImage.hpp"
#include "WbLog.hpp"
#include "WbMFString.hpp"
#include "WbMathsUtilities.hpp"
#include "WbNetwork.hpp"
#include "WbPreferences.hpp"
#include "WbRgb.hpp"
#include "WbSFBool.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"

#include <QtCore/QFileInfo>
#include <QtCore/QIODevice>
#include <QtGui/QImageReader>

#include <wren/gl_state.h>
#include <wren/material.h>
#include <wren/texture.h>
#include <wren/texture_2d.h>
#include <wren/texture_transform.h>

#include <assimp/material.h>

#include <utility>

QSet<QString> WbImageTexture::cQualityChangedTexturesList;
static QMap<QString, std::pair<const QImage *, int>> gImagesMap;

void WbImageTexture::init() {
  mWrenTexture = NULL;
  mWrenBackgroundTexture = NULL;
  mWrenTextureTransform = NULL;
  mExternalTexture = false;
  mExternalTextureRatio.setXy(1.0, 1.0);
  mExternalTextureData = NULL;
  mImage = NULL;
  mUsedFiltering = 0;
  mWrenTextureIndex = 0;
  mIsMainTextureTransparent = true;
  mRole = "";
  mDownloader = NULL;
}

void WbImageTexture::initFields() {
  mUrl = findMFString("url");
  mRepeatS = findSFBool("repeatS");
  mRepeatT = findSFBool("repeatT");
  mFiltering = findSFInt("filtering");
}

WbImageTexture::WbImageTexture(WbTokenizer *tokenizer) : WbBaseNode("ImageTexture", tokenizer) {
  init();
  initFields();
}

WbImageTexture::WbImageTexture(const WbNode &other) : WbBaseNode(other) {
  init();
  initFields();
}

WbImageTexture::WbImageTexture(const WbImageTexture &other) : WbBaseNode(other) {
  init();
  initFields();
}

WbImageTexture::WbImageTexture(const aiMaterial *material, aiTextureType textureType, QString parentPath) :
  WbBaseNode("ImageTexture", material) {
  init();

  assert(!parentPath.endsWith("/"));

  aiString path("");
  material->GetTexture(textureType, 0, &path);
  // generate url of texture from url of collada/wavefront file
  QString relativePath = QString(path.C_Str());
  relativePath.replace("\\", "/");  // use cross-platform forward slashes
  while (relativePath.startsWith("../")) {
    parentPath = parentPath.left(parentPath.lastIndexOf("/"));
    relativePath.remove(0, 3);
  }

  if (!relativePath.startsWith("/"))
    relativePath.insert(0, '/');

  mUrl = new WbMFString(QStringList(WbUrl::computePath(this, "url", parentPath + relativePath, false)));
  // init remaining variables with default wrl values
  mRepeatS = new WbSFBool(true);
  mRepeatT = new WbSFBool(true);
  mFiltering = new WbSFInt(4);
}

WbImageTexture::~WbImageTexture() {
  destroyWrenTexture();

  if (mIsShallowNode) {
    delete mUrl;
    delete mRepeatS;
    delete mRepeatT;
    delete mFiltering;
  }
}

void WbImageTexture::downloadAssets() {
  if (mUrl->size() == 0)
    return;

  const QString completeUrl = WbUrl::computePath(this, "url", mUrl->item(0), false);
  if (!WbUrl::isWeb(completeUrl) || WbNetwork::instance()->isCached(completeUrl))
    return;

  if (mDownloader && mDownloader->hasFinished())
    delete mDownloader;

  mDownloader = new WbDownloader(this);
  if (!WbWorld::instance()->isLoading() || mIsShallowNode)  // URL changed from the scene tree or supervisor
    connect(mDownloader, &WbDownloader::complete, this, &WbImageTexture::downloadUpdate);

  mDownloader->download(QUrl(completeUrl));
}

void WbImageTexture::downloadUpdate() {
  updateUrl();
  WbWorld::instance()->viewpoint()->emit refreshRequired();
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
  connect(WbPreferences::instance(), &WbPreferences::changedByUser, this, &WbImageTexture::updateFiltering);

  if (!WbWorld::instance()->isLoading())
    emit changed();
}

bool WbImageTexture::loadTexture() {
  const QString &completeUrl = WbUrl::computePath(this, "url", mUrl->item(0), false);
  const bool isWebAsset = WbUrl::isWeb(completeUrl);
  if (isWebAsset && !WbNetwork::instance()->isCached(completeUrl))
    return false;

  const QString filePath = isWebAsset ? WbNetwork::instance()->get(completeUrl) : path(true);
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) {
    warn(tr("Texture file could not be read: '%1'").arg(filePath));
    return false;
  }
  const bool r = loadTextureData(&file);
  file.close();
  return r;
}

bool WbImageTexture::loadTextureData(QIODevice *device) {
  QImageReader imageReader(device);
  QSize textureSize = imageReader.size();
  const int imageWidth = textureSize.width();
  const int imageHeight = textureSize.height();
  int width = WbMathsUtilities::nextPowerOf2(imageWidth);
  int height = WbMathsUtilities::nextPowerOf2(imageHeight);
  if (width != imageWidth || height != imageHeight)
    warn(tr("Texture image size of '%1' is not a power of two: rescaling it from %2x%3 to %4x%5.")
           .arg(path())
           .arg(imageWidth)
           .arg(imageHeight)
           .arg(width)
           .arg(height));

  const int quality = WbPreferences::instance()->value("OpenGL/textureQuality", 2).toInt();
  const int divider = 4 * pow(0.5, quality);      // 0: 4, 1: 2, 2: 1
  const int maxResolution = pow(2, 9 + quality);  // 0: 512, 1: 1024, 2: 2048
  if (divider != 1) {
    if (width >= maxResolution)
      width /= divider;
    if (height >= maxResolution)
      height /= divider;
  }

  mImage = new QImage();

  if (!imageReader.read(mImage)) {
    warn(tr("Cannot load texture '%1': %2.").arg(path()).arg(imageReader.errorString()));
    return false;
  }

  mIsMainTextureTransparent = mImage->pixelFormat().alphaUsage() == QPixelFormat::UsesAlpha;

  if (mImage->format() != QImage::Format_ARGB32) {
    QImage tmp = mImage->convertToFormat(QImage::Format_ARGB32);
    mImage->swap(tmp);
  }

  if (mImage->width() != width || mImage->height() != height) {
    // Qt::SmoothTransformation alterates the alpha channel.
    // Qt::FastTransformation creates ugly aliasing effects.
    // A custom scale with gaussian blur is the best tradeoff found between quality and loading performance.
    WbImage *image = new WbImage((unsigned char *)mImage->constBits(), mImage->width(), mImage->height());
    WbImage *downscaledImage =
      image->downscale(width, height, qMax(0, mImage->width() / width - 1), qMax(0, mImage->height() / height - 1));
    delete image;
    QImage tmp(downscaledImage->data(), width, height, mImage->format());
    delete downscaledImage;
    mImage->swap(tmp);

    if (WbWorld::isX3DStreaming()) {
      const QString &tmpFileName = WbStandardPaths::webotsTmpPath() + QFileInfo(path()).fileName();
      if (mImage->save(tmpFileName))
        cQualityChangedTexturesList.insert(path());
      else
        warn(tr("Cannot save texture with reduced quality to temporary file '%1'.").arg(tmpFileName));
    }
  }

  return true;
}

void WbImageTexture::updateWrenTexture() {
  // Calling destroyWrenTexture() decreases the count of gImagesMap, so if it is called before a node is finalized,
  // previously loaded images (in gImagesMap) would be deleted which results in an incorrect initialization of the node
  // because the texture is available in the cache but no reference to it remains as the only reference was immediately
  // deleted
  if (isPostFinalizedCalled())
    destroyWrenTexture();

  QString filePath(path());
  if (filePath.isEmpty())
    return;

  // Only load the image from disk if the texture isn't already in the cache
  WrTexture2d *texture = wr_texture_2d_copy_from_cache(filePath.toUtf8().constData());
  if (!texture) {
    if (loadTexture()) {
      WbWrenOpenGlContext::makeWrenCurrent();

      texture = wr_texture_2d_new();
      wr_texture_set_size(WR_TEXTURE(texture), mImage->width(), mImage->height());
      wr_texture_set_translucent(WR_TEXTURE(texture), mIsMainTextureTransparent);
      wr_texture_2d_set_data(texture, reinterpret_cast<const char *>(mImage->bits()));
      wr_texture_2d_set_file_path(texture, filePath.toUtf8().constData());
      wr_texture_setup(WR_TEXTURE(texture));

      WbWrenOpenGlContext::doneWren();
      if (mUrl->size() == 0)
        return;
      const QString &url(mUrl->item(0));
      gImagesMap[url] = std::make_pair(mImage, 1);
    }
  } else {  // texture is already available
    if (mUrl->size() == 0)
      return;
    const QString &url(mUrl->item(0));
    std::pair<const QImage *, int> pair = gImagesMap.value(url);
    if (pair.first) {
      mImage = const_cast<QImage *>(pair.first);  // mImage needs to be defined regardless as pickColor relies on it
      gImagesMap[url] = std::make_pair(pair.first, pair.second + 1);
    }

    mIsMainTextureTransparent = wr_texture_is_translucent(WR_TEXTURE(texture));
  }

  mWrenTexture = WR_TEXTURE(texture);
  if (mDownloader != NULL)
    delete mDownloader;
  mDownloader = NULL;
}

void WbImageTexture::destroyWrenTexture() {
  if (!mExternalTexture)
    wr_texture_delete(WR_TEXTURE(mWrenTexture));

  wr_texture_transform_delete(mWrenTextureTransform);

  mWrenTexture = NULL;
  mWrenTextureTransform = NULL;

  if (mUrl->size() == 0)
    return;
  QMapIterator<QString, std::pair<const QImage *, int>> i(gImagesMap);
  while (i.hasNext()) {
    i.next();
    const QImage *image = i.value().first;
    if (image && image == mImage) {
      const QString key = i.key();
      const int instances = i.value().second - 1;
      assert(instances >= 0);
      if (instances == 0) {
        delete mImage;
        gImagesMap.remove(key);
      } else
        gImagesMap[key] = std::make_pair(image, instances);
    }
  }

  mImage = NULL;
}

void WbImageTexture::updateUrl() {
  // check url validity
  if (path().isEmpty())
    return;

  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  const int n = mUrl->size();
  for (int i = 0; i < n; i++) {
    QString item = mUrl->item(i);
    mUrl->blockSignals(true);
    mUrl->setItem(i, item.replace("\\", "/"));
    mUrl->blockSignals(false);
  }

  if (n > 0) {
    const QString completeUrl = WbUrl::computePath(this, "url", mUrl->item(0), false);
    if (WbUrl::isWeb(completeUrl)) {
      if (mDownloader && !mDownloader->error().isEmpty()) {
        warn(mDownloader->error());  // failure downloading or file does not exist (404)
        // since the url is invalid the currently loaded texture should be removed (if any)
        destroyWrenTexture();
        delete mDownloader;
        mDownloader = NULL;
        return;
      }

      if (!WbNetwork::instance()->isCached(completeUrl) && mDownloader == NULL) {
        downloadAssets();  // url was changed from the scene tree or supervisor
        return;
      }
    }
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
  if (WbFieldChecker::resetIntIfNotInRangeWithIncludedBounds(this, mFiltering, 0, 5, 4))
    return;

  // The filtering level has an upper bound defined by the maximum supported anisotropy level.
  // A warning is not produced here because the maximum anisotropy level is not up to the user
  // and may be repeatedly shown even though a minimum requirement warning was already given.
  const int maxFiltering = WbPreferences::instance()->value("OpenGL/textureFiltering").toInt();
  mUsedFiltering = qMin(mFiltering->value(), maxFiltering);

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
    wr_material_set_texture_anisotropy(wrenMaterial, 1 << (mUsedFiltering - 1), mWrenTextureIndex);
    wr_material_set_texture_enable_interpolation(wrenMaterial, mUsedFiltering, mWrenTextureIndex);
    wr_material_set_texture_enable_mip_maps(wrenMaterial, mUsedFiltering, mWrenTextureIndex);

    if (mExternalTexture && !(static_cast<WbAbstractAppearance *>(parentNode())->textureTransform())) {
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
  mExternalTextureData = NULL;

  updateWrenTexture();
}

void WbImageTexture::setBackgroundTexture(WrTexture *backgroundTexture) {
  mWrenBackgroundTexture = backgroundTexture;
  emit changed();
}

void WbImageTexture::unsetBackgroundTexture() {
  mWrenBackgroundTexture = NULL;
  emit changed();
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

void WbImageTexture::pickColor(const WbVector2 &uv, WbRgb &pickedColor) {
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
    if (loadTexture() && mImage)
      data = mImage->bits();
    else {
      pickedColor.setValue(1.0, 1.0, 1.0);
      return;
    }
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

  const int index = 4 * (w * qMin((int)(v * h), h - 1) + qMin((int)(u * w), w - 1));
  pickedColor.setByteValue((int)data[index + 2], (int)data[index + 1], (int)data[index]);

  // debug
  // printf("pickedColor (u=%f, v=%f): (r=%f g=%f b=%f)\n", u, v, pickedColor.red(), pickedColor.green(), pickedColor.blue());
}

const QString WbImageTexture::path(bool warning) const {
  if (mUrl->size() == 0)
    return "";
  if (WbUrl::isWeb(mUrl->item(0)))
    return mUrl->item(0);
  return WbUrl::computePath(this, "url", mUrl, 0, warning);
}

void WbImageTexture::write(WbWriter &writer) const {
  if (!isUseNode() && writer.isProto()) {
    for (int i = 0; i < mUrl->size(); ++i) {
      QString texturePath(WbUrl::computePath(this, "url", mUrl, i));
      const QString &url(mUrl->item(i));
      if (cQualityChangedTexturesList.contains(texturePath))
        texturePath = WbStandardPaths::webotsTmpPath() + QFileInfo(url).fileName();
      writer.addTextureToList(url, texturePath);
    }
  }

  WbBaseNode::write(writer);
}

bool WbImageTexture::exportNodeHeader(WbWriter &writer) const {
  if (!writer.isX3d() || !isUseNode() || mRole.isEmpty())
    return WbBaseNode::exportNodeHeader(writer);

  writer << "<" << x3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
  if (isInvisibleNode())
    writer << " render=\'false\'";
  if (defNode())
    writer << " USE=\'" + QString::number(defNode()->uniqueId()) + "\'";
  writer << " role=\'" << mRole << "\' ></" + x3dName() + ">";
  return true;
}

void WbImageTexture::exportNodeFields(WbWriter &writer) const {
  // export to ./textures folder relative to writer path
  WbField urlFieldCopy(*findField("url", true));
  for (int i = 0; i < mUrl->size(); ++i) {
    QString url = WbUrl::computePath(this, "url", mUrl, i);
    if (WbUrl::isLocalUrl() {
      QString newUrl = mUrl->value()[i];
      newUrl = newUrl.replace("webots://", "https://raw.githubusercontent.com/" + WbApplicationInfo::repo() + "/" +
                                             WbApplicationInfo::branch() + "/");
      dynamic_cast<WbMFString *>(urlFieldCopy.value())->setItem(i, newUrl);
    } else if (WbUrl::isWeb(mUrl->value()[i]))
      continue;
    else {
      QString texturePath(WbUrl::computePath(this, "url", mUrl, i));
      if (writer.isWritingToFile()) {
        QString newUrl = WbUrl::exportTexture(this, mUrl, i, writer);
        dynamic_cast<WbMFString *>(urlFieldCopy.value())->setItem(i, newUrl);
      }

      const QString &url(mUrl->item(i));
      if (cQualityChangedTexturesList.contains(texturePath))
        texturePath = WbStandardPaths::webotsTmpPath() + QFileInfo(url).fileName();
      writer.addTextureToList(url, texturePath);
    }
  }
  urlFieldCopy.write(writer);
  findField("repeatS", true)->write(writer);
  findField("repeatT", true)->write(writer);
  findField("filtering", true)->write(writer);

  if (writer.isX3d()) {
    writer << " isTransparent=\'" << (mIsMainTextureTransparent ? "true" : "false") << "\'";
    if (!mRole.isEmpty())
      writer << " role=\'" << mRole << "\'";
  }
}

void WbImageTexture::exportShallowNode(WbWriter &writer) const {
  if (!writer.isX3d() || mUrl->size() == 0)
    return;

  QString url = mUrl->item(0);
  // note: by the time this point is reached, the url is either a local file or a remote one (https://), in other words any
  // 'webots://' would have been handled already in the constructor of the WbImageTexture instance (to find the url of the
  // image relative to the parent collada/wavefront file)
  if (!url.startsWith("https://")) {  // local path
    if (WbWorld::isX3DStreaming())
      writer.addTextureToList(url, WbUrl::computePath(this, "url", url));
    else {
      url = WbUrl::exportTexture(this, mUrl, 0, writer);
      writer.addTextureToList(mUrl->item(0), url);
    }
  }

  writer << "<ImageTexture";
  writer << " url='\"" << url << "\"'";
  writer << " isTransparent=\'" << (mIsMainTextureTransparent ? "true" : "false") << "\'";
  if (!mRole.isEmpty())
    writer << " role='" << mRole << "'";
  writer << "></ImageTexture>";
}
