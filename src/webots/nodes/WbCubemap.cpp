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

#include "WbCubemap.hpp"

#include "WbNodeUtilities.hpp"
#include "WbSFInt.hpp"
#include "WbSFString.hpp"
#include "WbUrl.hpp"
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#define STB_IMAGE_IMPLEMENTATION  // needed for include to work properly
#include <stb_image.h>

#include <wren/gl_state.h>
#include <wren/material.h>
#include <wren/texture.h>
#include <wren/texture_2d.h>
#include <wren/texture_cubemap.h>
#include <wren/texture_cubemap_baker.h>

#include <QtCore/QFileInfo>
#include <QtGui/QImage>
#include <QtGui/QImageReader>

// The order is not arbitrary: when loading the cubemap textures into
// the GPU via glTexImage2D(GL_TEXTURE_CUBEMAP_..), they are provided in this order:
// right, left, top, bottom, back, front
// But it was changed to match with webots original implementation. (right, left, top, bottom, front, back)
static QString gTextureSuffixes[6] = {"_right", "_left", "_top", "_bottom", "_front", "_back"};

const QString *WbCubemap::textureSuffixes() {
  return gTextureSuffixes;
}

void WbCubemap::init() {
  mTextureBaseName = findSFString("textureBaseName");
  mDirectory = findSFString("directory");
  mDefaultCubeTexture = NULL;
  mDiffuseIrradianceCubeTexture = NULL;
  mSpecularIrradianceCubeTexture = NULL;
  mIsValid = false;
  mIsEquirectangular = false;

  for (int i = 0; i < 6; ++i) {
    mQImages[i] = NULL;
    mTextureUrls[i] = "";
  }
}

WbCubemap::WbCubemap(WbTokenizer *tokenizer) : WbBaseNode("Cubemap", tokenizer) {
  init();
}

WbCubemap::WbCubemap(const WbCubemap &other) : WbBaseNode(other) {
  init();
}

WbCubemap::WbCubemap(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbCubemap::~WbCubemap() {
  clearWrenTexture();

  for (int i = 0; i < 6; ++i)
    delete mQImages[i];
}

void WbCubemap::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mTextureBaseName, &WbSFString::changed, this, &WbCubemap::updateWrenTexture);
  connect(mDirectory, &WbSFString::changed, this, &WbCubemap::updateWrenTexture);

  if (!WbWorld::instance()->isLoading())
    emit changed();
}

void WbCubemap::clearWrenTexture() {
  if (mIsValid) {
    wr_texture_delete(WR_TEXTURE(mDefaultCubeTexture));
    wr_texture_delete(WR_TEXTURE(mDiffuseIrradianceCubeTexture));
    wr_texture_delete(WR_TEXTURE(mSpecularIrradianceCubeTexture));
  }

  mDefaultCubeTexture = NULL;
  mDiffuseIrradianceCubeTexture = NULL;
  mSpecularIrradianceCubeTexture = NULL;

  mIsValid = false;

  emit cubeTexturesDestroyed();
}

void WbCubemap::updateWrenTexture() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbCubemap::loadWrenTexture() {
  // silently exit, not all name fields have values
  if (mDirectory->value().isEmpty() || mTextureBaseName->value().isEmpty()) {
    clearWrenTexture();
    return;
  }

  QString expectedEquirectangularPath =
    WbUrl::computePath(this, "textureBaseName", mDirectory->value() + "/" + mTextureBaseName->value() + ".hdr", false);

  if (!expectedEquirectangularPath.isEmpty()) {
    mIsEquirectangular = true;
    stbi_set_flip_vertically_on_load(true);
    int width, height, nrComponents;
    float *data = stbi_loadf(expectedEquirectangularPath.toUtf8().constData(), &width, &height, &nrComponents, 0);

    WrTexture2d *equirectangularWrenTexture = wr_texture_2d_new();
    wr_texture_set_size(WR_TEXTURE(equirectangularWrenTexture), width, height);
    wr_texture_set_translucent(WR_TEXTURE(equirectangularWrenTexture), false);
    wr_texture_2d_set_data(equirectangularWrenTexture, reinterpret_cast<const char *>(data));
    wr_texture_2d_set_file_path(equirectangularWrenTexture, expectedEquirectangularPath.toUtf8().constData());
    wr_texture_set_internal_format(WR_TEXTURE(equirectangularWrenTexture), WR_TEXTURE_INTERNAL_FORMAT_RGB32F);

    WbWrenOpenGlContext::makeWrenCurrent();
    wr_texture_setup(WR_TEXTURE(equirectangularWrenTexture));
    stbi_image_free(data);
    mDefaultCubeTexture = wr_texture_cubemap_bake_equirectangular_to_cube(equirectangularWrenTexture,
                                                                          WbWrenShaders::iblEquirectangularShader(), 1024);
    wr_texture_delete(WR_TEXTURE(equirectangularWrenTexture));
    WbWrenOpenGlContext::doneWren();

  } else {
    mIsEquirectangular = false;
    QString mSuffix = "";
    bool allTexturesAreDefined = true;
    for (int i = 0; i < 6; ++i) {
      mTextureUrls[i] = WbUrl::computePath(
        this, "textureBaseName", mDirectory->value() + "/" + mTextureBaseName->value() + gTextureSuffixes[i] + ".jpg", false);
      if (mTextureUrls[i].isEmpty() && mSuffix.isEmpty()) {
        mTextureUrls[i] = WbUrl::computePath(
          this, "textureBaseName", mDirectory->value() + "/" + mTextureBaseName->value() + gTextureSuffixes[i] + ".png", false);
        mSuffix = ".png";
      } else
        mSuffix = ".jpg";

      if (mTextureUrls[i].isEmpty())
        allTexturesAreDefined = false;
    }

    if (!allTexturesAreDefined) {
      warn(tr("Impossible to assemble cube texture: not all six faces could be found with base name '%1' in '%2'.")
             .arg(mTextureBaseName->value())
             .arg(mDirectory->value()));
      clearWrenTexture();
      return;
    }

    mDefaultCubeTexture = wr_texture_cubemap_new();

    int edgeLength = 0;
    bool alpha = false;
    QString lastFile;

    for (int i = 0; i < 6; i++) {
      QImageReader imageReader(mTextureUrls[i]);
      QSize textureSize = imageReader.size();

      if (textureSize.width() != textureSize.height()) {
        warn(tr("The texture '%1' is not a square image (its width doesn't equal its height).").arg(imageReader.fileName()));
        clearWrenTexture();
        return;
      }

      if (i > 0 && textureSize.width() != edgeLength) {
        warn(tr("Texture dimension mismatch between '%1' and '%2'").arg(lastFile).arg(imageReader.fileName()));
        clearWrenTexture();
        return;
      }

      edgeLength = textureSize.width();

      delete mQImages[i];

      mQImages[i] = new QImage();

      QImage *image = mQImages[i];

      if (imageReader.read(image)) {
        if (i > 0 && (alpha != image->hasAlphaChannel())) {
          warn(tr("Alpha channel mismatch between '%1' and '%2'").arg(imageReader.fileName()).arg(lastFile));
          clearWrenTexture();
          return;
        }

        alpha = image->hasAlphaChannel();

        if (image->format() != QImage::Format_ARGB32) {
          QImage tmp = image->convertToFormat(QImage::Format_ARGB32);
          image->swap(tmp);
        }

        wr_texture_cubemap_set_data(mDefaultCubeTexture, reinterpret_cast<const char *>(image->bits()),
                                    static_cast<WrTextureOrientation>(i));
      } else {
        warn(tr("Cannot load texture '%1': %2.").arg(imageReader.fileName()).arg(imageReader.errorString()));
        clearWrenTexture();
        return;
      }
      lastFile = imageReader.fileName();
    }
    wr_texture_set_size(WR_TEXTURE(mDefaultCubeTexture), edgeLength, edgeLength);
    wr_texture_set_internal_format(WR_TEXTURE(mDefaultCubeTexture), WR_TEXTURE_INTERNAL_FORMAT_RGBA8);

    WbWrenOpenGlContext::makeWrenCurrent();

    wr_texture_setup(WR_TEXTURE(mDefaultCubeTexture));

    WbWrenOpenGlContext::doneWren();
  }

  WbWrenOpenGlContext::makeWrenCurrent();

  mDiffuseIrradianceCubeTexture =
    wr_texture_cubemap_bake_diffuse_irradiance(mDefaultCubeTexture, WbWrenShaders::iblDiffuseIrradianceBakingShader(), 32);

  mSpecularIrradianceCubeTexture =
    wr_texture_cubemap_bake_specular_irradiance(mDefaultCubeTexture, WbWrenShaders::iblSpecularIrradianceBakingShader());
  wr_texture_cubemap_disable_automatic_mip_map_generation(mSpecularIrradianceCubeTexture);

  WbWrenOpenGlContext::doneWren();
  mIsValid = true;
  emit bakeCompleted();
}

void WbCubemap::modifyWrenMaterial(WrMaterial *material) {
  if (!material)
    return;

  // diffuse irradiance map
  wr_material_set_texture_cubemap(material, mDiffuseIrradianceCubeTexture, 0);
  wr_material_set_texture_cubemap_wrap_r(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_material_set_texture_cubemap_wrap_s(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_material_set_texture_cubemap_wrap_t(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_material_set_texture_cubemap_anisotropy(material, 8, 0);
  wr_material_set_texture_cubemap_enable_interpolation(material, true, 0);

  // specular irradiance map
  wr_material_set_texture_cubemap(material, mSpecularIrradianceCubeTexture, 1);
  wr_material_set_texture_cubemap_wrap_r(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
  wr_material_set_texture_cubemap_wrap_s(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
  wr_material_set_texture_cubemap_wrap_t(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
  wr_material_set_texture_cubemap_anisotropy(material, 8, 1);
  wr_material_set_texture_cubemap_enable_interpolation(material, true, 1);
  wr_material_set_texture_cubemap_enable_mip_maps(material, true, 1);
}
