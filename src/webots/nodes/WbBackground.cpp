// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbBackground.hpp"

#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbGroup.hpp"
#include "WbMFColor.hpp"
#include "WbMFString.hpp"
#include "WbMathsUtilities.hpp"
#include "WbNodeOperations.hpp"
#include "WbPreferences.hpp"
#include "WbSFNode.hpp"
#include "WbUrl.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/gl_state.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/scene.h>
#include <wren/shader_program.h>
#include <wren/static_mesh.h>
#include <wren/texture_cubemap.h>
#include <wren/texture_cubemap_baker.h>
#include <wren/transform.h>
#include <wren/viewport.h>

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtGui/QImage>
#include <QtGui/QImageReader>

#define STB_IMAGE_IMPLEMENTATION  // needed for include to work properly
#include <stb_image.h>

QList<WbBackground *> WbBackground::cBackgroundList;
static QString gUrlNames[6] = {"rightUrl", "leftUrl", "topUrl", "bottomUrl", "frontUrl", "backUrl"};
static QString gTextureSuffixes[6] = {"_right", "_left", "_top", "_bottom", "_front", "_back"};

void WbBackground::init() {
  mSkyColor = findMFColor("skyColor");
  mLuminosity = findSFDouble("luminosity");
  for (int i = 0; i < 6; ++i)
    mUrlFields[i] = findMFString(gUrlNames[i]);

  QString directory;
  QString textureBaseName;
  for (int i = 0; i < 6; ++i) {
    if (!mUrlFields[i] || mUrlFields[i]->size() == 0) {
      // empty url
      if (i != 0)
        warn(tr("Impossible to create the cubemap because not all the url fields are defined."));
      break;
    }
    const QFileInfo &fileInfo(mUrlFields[i]->item(0));
    const QString &currentDirectory = fileInfo.dir().path();
    QString currentTextureBaseName = fileInfo.baseName();

    const int index = currentTextureBaseName.lastIndexOf(gTextureSuffixes[i]);
    if (index < 0) {
      // suffix not found
      warn(tr("Impossible to create the cubemap because the texture file defined in the '%1' field doesn't end with the '%2' "
              "suffix.")
             .arg(gUrlNames[i])
             .arg(gTextureSuffixes[i]));
      break;
    }
    currentTextureBaseName = currentTextureBaseName.left(index);
    if (i == 0) {
      directory = currentDirectory;
      textureBaseName = currentTextureBaseName;
    } else if (directory != currentDirectory || textureBaseName != currentTextureBaseName) {
      // texture are not in the same directory or using the same base name
      warn(tr("Impossible to create the cubemap because the textures defined in the url fields are not in the same folder or "
              "they do not share the same base name."));
      break;
    }
  }

  mSkyboxShaderProgram = NULL;
  mSkyboxRenderable = NULL;
  mSkyboxMaterial = NULL;
  mSkyboxTransform = NULL;
  mSkyboxMesh = NULL;

  mHdrClearShaderProgram = NULL;
  mHdrClearRenderable = NULL;
  mHdrClearMaterial = NULL;
  mHdrClearTransform = NULL;
  mHdrClearMesh = NULL;

  mCubeMapTexture = NULL;
  mDiffuseIrradianceCubeTexture = NULL;
  mSpecularIrradianceCubeTexture = NULL;
}

WbBackground::WbBackground(WbTokenizer *tokenizer) : WbBaseNode("Background", tokenizer) {
  init();
  if (tokenizer == NULL)
    mSkyColor->setItem(0, WbRgb(0.15, 0.45, 1), false);
}

WbBackground::WbBackground(const WbBackground &other) : WbBaseNode(other) {
  init();
}

WbBackground::WbBackground(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbBackground::~WbBackground() {
  const bool firstInstanceDeleted = isFirstInstance();

  cBackgroundList.removeAll(this);

  destroySkyBox();

  if (firstInstanceDeleted) {
    WbBackground *newFirstInstance = firstInstance();
    if (newFirstInstance == NULL)
      // reset to default
      applyColourToWren(WbRgb());
    else
      // activate next Background node
      newFirstInstance->activate();
  }

  if (!WbWorld::instance()->isCleaning())
    emit WbWrenRenderingContext::instance()->backgroundColorChanged();

  wr_scene_set_hdr_clear_quad(wr_scene_get_instance(), NULL);
  // Delete skybox
  // Shader program is not deleted, a singleton instance is kept in WbWrenShaders
  wr_node_delete(WR_NODE(mSkyboxRenderable));

  if (mSkyboxMaterial)
    wr_material_delete(mSkyboxMaterial);

  wr_node_delete(WR_NODE(mSkyboxTransform));
  wr_static_mesh_delete(mSkyboxMesh);

  // Delete skybox
  // Shader program is not deleted, a singleton instance is kept in WbWrenShaders
  wr_node_delete(WR_NODE(mHdrClearRenderable));
  mHdrClearRenderable = NULL;
  wr_scene_set_hdr_clear_quad(wr_scene_get_instance(), mHdrClearRenderable);

  if (mHdrClearMaterial)
    wr_material_delete(mHdrClearMaterial);

  wr_node_delete(WR_NODE(mHdrClearTransform));
  wr_static_mesh_delete(mHdrClearMesh);
}

void WbBackground::preFinalize() {
  WbBaseNode::preFinalize();
  cBackgroundList << this;
}

void WbBackground::postFinalize() {
  WbBaseNode::postFinalize();

  if (isFirstInstance())
    activate();
  else
    warn(tr("Only one Background node is allowed. The current node won't be taken into account."));
}

void WbBackground::activate() {
  if (!areWrenObjectsInitialized())
    createWrenObjects();

  connect(mLuminosity, &WbSFDouble::changed, this, &WbBackground::updateLuminosity);
  connect(mSkyColor, &WbMFColor::changed, this, &WbBackground::updateColor);
  for (int i = 0; i < 6; ++i)
    connect(mUrlFields[i], &WbMFString::changed, this, &WbBackground::updateCubemap);

  updateColor();

  updateCubemap();
}

void WbBackground::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  mSkyboxShaderProgram = WbWrenShaders::skyboxShader();
  mSkyboxMaterial = wr_phong_material_new();
  mSkyboxRenderable = wr_renderable_new();
  mSkyboxMesh = wr_static_mesh_unit_box_new(false);

  wr_material_set_default_program(mSkyboxMaterial, mSkyboxShaderProgram);
  wr_renderable_set_cast_shadows(mSkyboxRenderable, false);
  wr_renderable_set_receive_shadows(mSkyboxRenderable, false);
  wr_renderable_set_mesh(mSkyboxRenderable, WR_MESH(mSkyboxMesh));
  wr_renderable_set_material(mSkyboxRenderable, mSkyboxMaterial, NULL);
  wr_renderable_set_drawing_mode(mSkyboxRenderable, WR_RENDERABLE_DRAWING_MODE_TRIANGLES);
  wr_renderable_set_face_culling(mSkyboxRenderable, false);

  mSkyboxTransform = wr_transform_new();
  wr_transform_attach_child(mSkyboxTransform, WR_NODE(mSkyboxRenderable));

  mHdrClearShaderProgram = WbWrenShaders::hdrClearShader();
  mHdrClearMaterial = wr_phong_material_new();
  mHdrClearRenderable = wr_renderable_new();
  mHdrClearMesh = wr_static_mesh_quad_new();

  wr_material_set_default_program(mHdrClearMaterial, mHdrClearShaderProgram);
  wr_renderable_set_cast_shadows(mHdrClearRenderable, false);
  wr_renderable_set_receive_shadows(mHdrClearRenderable, false);
  wr_renderable_set_mesh(mHdrClearRenderable, WR_MESH(mHdrClearMesh));
  wr_renderable_set_material(mHdrClearRenderable, mHdrClearMaterial, NULL);
  wr_renderable_set_drawing_mode(mHdrClearRenderable, WR_RENDERABLE_DRAWING_MODE_TRIANGLES);

  mHdrClearTransform = wr_transform_new();
  wr_transform_attach_child(mHdrClearTransform, WR_NODE(mHdrClearRenderable));

  if (isFirstInstance())
    applyColourToWren(skyColor());
}

void WbBackground::destroySkyBox() {
  wr_scene_set_skybox(wr_scene_get_instance(), NULL);

  if (mSkyboxMaterial)
    wr_material_set_texture_cubemap(mSkyboxMaterial, NULL, 0);

  if (mCubeMapTexture) {
    wr_texture_delete(WR_TEXTURE(mCubeMapTexture));
    mCubeMapTexture = NULL;
  }

  if (mDiffuseIrradianceCubeTexture) {
    wr_texture_delete(WR_TEXTURE(mDiffuseIrradianceCubeTexture));
    mDiffuseIrradianceCubeTexture = NULL;
  }

  if (mSpecularIrradianceCubeTexture) {
    wr_texture_delete(WR_TEXTURE(mSpecularIrradianceCubeTexture));
    mSpecularIrradianceCubeTexture = NULL;
  }
}

void WbBackground::updateColor() {
  if (WbFieldChecker::resetMultipleColorIfInvalid(this, mSkyColor))
    return;

  if (areWrenObjectsInitialized())
    applyColourToWren(skyColor());

  emit WbWrenRenderingContext::instance()->backgroundColorChanged();
}

void WbBackground::updateCubemap() {
  if (areWrenObjectsInitialized())
    applySkyBoxToWren();
}

void WbBackground::updateLuminosity() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mLuminosity, 1.0))
    return;

  emit luminosityChanged();
}

void WbBackground::applyColourToWren(const WbRgb &color) {
  const float value[] = {static_cast<float>(color.red()), static_cast<float>(color.green()), static_cast<float>(color.blue())};
  wr_viewport_set_clear_color_rgb(wr_scene_get_viewport(wr_scene_get_instance()), value);
  if (areWrenObjectsInitialized()) {
    // use wren's set_diffuse to transform to linear color space
    wr_phong_material_set_diffuse(mHdrClearMaterial, value);

    // de-gamma correct
    float hdrColor[] = {powf(value[0], 2.2), powf(value[1], 2.2), powf(value[2], 2.2)};

    // reverse tone map
    const float exposure = WbWorld::instance()->viewpoint()->exposure()->value();
    for (int i = 0; i < 3; ++i)
      hdrColor[i] = -log(1.000000001 - hdrColor[i]) / exposure;

    wr_phong_material_set_linear_diffuse(mHdrClearMaterial, hdrColor);
    wr_scene_set_hdr_clear_quad(wr_scene_get_instance(), mHdrClearRenderable);
  }
}

void WbBackground::applySkyBoxToWren() {
  destroySkyBox();

  mCubeMapTexture = wr_texture_cubemap_new();

  int edgeLength = 0;
  QString lastFile;

  QString textureUrls[6];
  QVector<float *> hdrImageData;
  QVector<QImage *> regularImageData;

  try {
    QString suffix = "";
    for (int i = 0; i < 6; ++i) {
      if (mUrlFields[i]->size() == 0)
        throw QString();

      textureUrls[i] = WbUrl::computePath(this, "textureBaseName", mUrlFields[i]->item(0), false);

      if (textureUrls[i].isEmpty())
        throw QString();

      QString newSuffix = QFileInfo(textureUrls[i]).suffix();
      if (i > 0 && newSuffix != suffix)
        throw tr("Inconsistent image format.");

      suffix = newSuffix;
    }

    if (suffix == "hdr") {
      wr_texture_set_internal_format(WR_TEXTURE(mCubeMapTexture), WR_TEXTURE_INTERNAL_FORMAT_RGB32F);

      for (int i = 0; i < 6; i++) {
        int width, height, nrComponents;
        float *data = stbi_loadf(textureUrls[i].toUtf8().constData(), &width, &height, &nrComponents, 0);

        if (width != height)
          throw tr("The texture '%1' is not a square image (its width doesn't equal its height).").arg(textureUrls[i]);
        if (i > 0 && width != edgeLength)
          throw tr("Texture dimension mismatch between '%1' and '%2'").arg(lastFile).arg(textureUrls[i]);

        hdrImageData.append(data);
        edgeLength = width;

        wr_texture_cubemap_set_data(mCubeMapTexture, reinterpret_cast<const char *>(data),
                                    static_cast<WrTextureOrientation>(i));
      }
    } else {
      wr_texture_set_internal_format(WR_TEXTURE(mCubeMapTexture), WR_TEXTURE_INTERNAL_FORMAT_RGBA8);

      bool alpha = false;
      for (int i = 0; i < 6; i++) {
        QImageReader imageReader(textureUrls[i]);
        QSize textureSize = imageReader.size();

        if (textureSize.width() != textureSize.height())
          throw tr("The texture '%1' is not a square image (its width doesn't equal its height).").arg(imageReader.fileName());
        if (i > 0 && textureSize.width() != edgeLength)
          throw tr("Texture dimension mismatch between '%1' and '%2'").arg(lastFile).arg(imageReader.fileName());

        edgeLength = textureSize.width();

        QImage *image = new QImage();
        regularImageData.append(image);

        if (imageReader.read(image)) {
          if (i > 0 && (alpha != image->hasAlphaChannel()))
            throw tr("Alpha channel mismatch between '%1' and '%2'").arg(imageReader.fileName()).arg(lastFile);

          alpha = image->hasAlphaChannel();

          if (image->format() != QImage::Format_ARGB32) {
            QImage tmp = image->convertToFormat(QImage::Format_ARGB32);
            image->swap(tmp);
          }

          wr_texture_cubemap_set_data(mCubeMapTexture, reinterpret_cast<const char *>(image->bits()),
                                      static_cast<WrTextureOrientation>(i));
        } else
          throw tr("Cannot load texture '%1': %2.").arg(imageReader.fileName()).arg(imageReader.errorString());

        lastFile = imageReader.fileName();
      }
    }
  } catch (QString &error) {
    if (error.length() > 0)
      warn(error);
    destroySkyBox();
    emit cubemapChanged();
    return;
  }

  wr_texture_set_size(WR_TEXTURE(mCubeMapTexture), edgeLength, edgeLength);

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_texture_setup(WR_TEXTURE(mCubeMapTexture));

  while (hdrImageData.size() > 0)
    stbi_image_free(hdrImageData.takeFirst());
  while (regularImageData.size() > 0)
    delete regularImageData.takeFirst();

  wr_material_set_texture_cubemap(mSkyboxMaterial, mCubeMapTexture, 0);
  wr_material_set_texture_cubemap_wrap_r(mSkyboxMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_material_set_texture_cubemap_wrap_s(mSkyboxMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_material_set_texture_cubemap_wrap_t(mSkyboxMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_scene_set_skybox(wr_scene_get_instance(), mSkyboxRenderable);

  mDiffuseIrradianceCubeTexture =
    wr_texture_cubemap_bake_diffuse_irradiance(mCubeMapTexture, WbWrenShaders::iblDiffuseIrradianceBakingShader(), 32);

  const int quality = WbPreferences::instance()->value("OpenGL/textureQuality", 2).toInt();
  // maps the quality eihter to '0: 64, 1: 128, 2: 256' or in case of HDR to '0: 32, 1: 64, 2: 256'
  const int resolution = 1 << (6 + quality);
  mSpecularIrradianceCubeTexture = wr_texture_cubemap_bake_specular_irradiance(
    mCubeMapTexture, WbWrenShaders::iblSpecularIrradianceBakingShader(), resolution);
  wr_texture_cubemap_disable_automatic_mip_map_generation(mSpecularIrradianceCubeTexture);

  WbWrenOpenGlContext::doneWren();

  emit cubemapChanged();
}

WbRgb WbBackground::skyColor() const {
  return (mSkyColor->size() > 0 ? mSkyColor->item(0) : WbRgb());
}

void WbBackground::exportNodeFields(WbVrmlWriter &writer) const {
  if (writer.isWebots()) {
    WbBaseNode::exportNodeFields(writer);
    return;
  }

  findField("skyColor", true)->write(writer);
  findField("luminosity", true)->write(writer);

  QString outputFileNames[6];
  for (int i = 0; i < 6; ++i) {
    if (mUrlFields[i]->size() == 0)
      continue;
    const QString &url = WbUrl::computePath(this, "textureBaseName", mUrlFields[i]->item(0), false);
    const QFileInfo &cubeInfo(url);
    if (writer.isWritingToFile())
      outputFileNames[i] =
        WbUrl::exportTexture(this, url, url, writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/", writer);
    else
      outputFileNames[i] = writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/" + cubeInfo.fileName();
    writer.addTextureToList(outputFileNames[i], url);
  }

  if (writer.isX3d()) {
    writer << " ";
    for (int i = 0; i < 6; ++i) {
      if (!outputFileNames[i].isEmpty())
        writer << gUrlNames[i] << "='\"" << outputFileNames[i] << "\"' ";
    }
  } else if (writer.isVrml()) {
    for (int i = 0; i < 6; ++i) {
      if (!outputFileNames[i].isEmpty()) {
        writer.indent();
        writer << gUrlNames[i] << " [ \"" << outputFileNames[i] << "\" ]\n";
      }
    }
  } else
    WbNode::exportNodeFields(writer);
}
