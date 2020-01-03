// Copyright 1996-2020 Cyberbotics Ltd.
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
static QString gIrradianceUrlNames[6] = {"rightIrradianceUrl",  "leftIrradianceUrl",  "topIrradianceUrl",
                                         "bottomIrradianceUrl", "frontIrradianceUrl", "backIrradianceUrl"};
static QString gTextureSuffixes[6] = {"_right", "_left", "_top", "_bottom", "_front", "_back"};

void WbBackground::init() {
  mSkyColor = findMFColor("skyColor");
  mLuminosity = findSFDouble("luminosity");
  for (int i = 0; i < 6; ++i) {
    mUrlFields[i] = findMFString(gUrlNames[i]);
    mIrradianceUrlFields[i] = findMFString(gIrradianceUrlNames[i]);
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
  mIrradianceCubeTexture = NULL;
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
  for (int i = 0; i < 6; ++i) {
    connect(mUrlFields[i], &WbMFString::changed, this, &WbBackground::updateCubemap);
    connect(mIrradianceUrlFields[i], &WbMFString::changed, this, &WbBackground::updateCubemap);
  }

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

  if (mIrradianceCubeTexture) {
    wr_texture_delete(WR_TEXTURE(mIrradianceCubeTexture));
    mIrradianceCubeTexture = NULL;
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

  WbWrenOpenGlContext::makeWrenCurrent();

  int edgeLength = 0;
  QString lastFile;

  QString textureUrls[6];
  QVector<float *> hdrImageData;
  QVector<QImage *> regularImageData;

  // 1. Load the background.
  mCubeMapTexture = wr_texture_cubemap_new();
  try {
    bool allUrlDefined = true;
    bool atLeastOneUrlDefined = false;

    for (int i = 0; i < 6; ++i) {
      if (mUrlFields[i]->size() == 0) {
        allUrlDefined = false;
        textureUrls[i] = "";
        continue;
      } else
        atLeastOneUrlDefined = true;

      textureUrls[i] = WbUrl::computePath(this, "textureBaseName", mUrlFields[i]->item(0), false);
    }

    if (!allUrlDefined)
      throw QString(atLeastOneUrlDefined ? tr("Incomplete cubemap") : "");

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
  } catch (QString &error) {
    if (error.length() > 0)
      warn(error);
    destroySkyBox();
  }

  if (mCubeMapTexture) {
    wr_texture_set_size(WR_TEXTURE(mCubeMapTexture), edgeLength, edgeLength);
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
  }

  // 2. Load the irradiance map.
  WrTextureCubeMap *cm = wr_texture_cubemap_new();

  try {
    // Check first that every fields are present.
    bool allUrlDefined = true;
    bool atLeastOneUrlDefined = false;
    for (int i = 0; i < 6; ++i) {
      if (mIrradianceUrlFields[i]->size() == 0) {
        allUrlDefined = false;
        continue;
      } else
        atLeastOneUrlDefined = true;
    }
    if (!allUrlDefined)
      throw tr(atLeastOneUrlDefined ? "Incomplete irradiance cubemap" : "");

    // Actually load the irradiance map.
    int w, h, components;
    for (int i = 0; i < 6; ++i) {
      QString url = WbUrl::computePath(this, "textureBaseName", mIrradianceUrlFields[i]->item(0), false);
      if (url.isEmpty())
        throw QString();

      wr_texture_set_internal_format(WR_TEXTURE(cm), WR_TEXTURE_INTERNAL_FORMAT_RGB32F);
      float *data = stbi_loadf(url.toUtf8().constData(), &w, &h, &components, 0);
      wr_texture_cubemap_set_data(cm, reinterpret_cast<const char *>(data), static_cast<WrTextureOrientation>(i));
    }

    wr_texture_set_size(WR_TEXTURE(cm), w, h);
    wr_texture_set_texture_unit(WR_TEXTURE(cm), 13);
    wr_texture_setup(WR_TEXTURE(cm));

    mIrradianceCubeTexture =
      wr_texture_cubemap_bake_specular_irradiance(cm, WbWrenShaders::iblSpecularIrradianceBakingShader(), w);
    wr_texture_cubemap_disable_automatic_mip_map_generation(mIrradianceCubeTexture);

  } catch (QString &error) {
    if (error.length() > 0)
      warn(error);

    if (mIrradianceCubeTexture) {
      wr_texture_delete(WR_TEXTURE(mIrradianceCubeTexture));
      mIrradianceCubeTexture = NULL;
    }

    // Fallback: a cubemap is found but no irradiance map: bake a small irradiance map to have right colors.
    // Reflections won't be good in such case.
    if (mCubeMapTexture) {
      mIrradianceCubeTexture =
        wr_texture_cubemap_bake_specular_irradiance(mCubeMapTexture, WbWrenShaders::iblSpecularIrradianceBakingShader(), 64);
      wr_texture_cubemap_disable_automatic_mip_map_generation(mIrradianceCubeTexture);
    }
  }

  wr_texture_delete(WR_TEXTURE(cm));

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

  QString backgroundFileNames[6];
  for (int i = 0; i < 6; ++i) {
    if (mUrlFields[i]->size() == 0)
      continue;
    const QString &url = WbUrl::computePath(this, "textureBaseName", mUrlFields[i]->item(0), false);
    const QFileInfo &cubeInfo(url);
    if (writer.isWritingToFile())
      backgroundFileNames[i] =
        WbUrl::exportTexture(this, url, url, writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/", writer);
    else
      backgroundFileNames[i] = writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/" + cubeInfo.fileName();
    writer.addTextureToList(backgroundFileNames[i], url);
  }

  QString irradianceFileNames[6];
  for (int i = 0; i < 6; ++i) {
    if (mIrradianceUrlFields[i]->size() == 0)
      continue;
    const QString &url = WbUrl::computePath(this, "textureBaseName", mIrradianceUrlFields[i]->item(0), false);
    const QFileInfo &cubeInfo(url);
    if (writer.isWritingToFile())
      irradianceFileNames[i] =
        WbUrl::exportTexture(this, url, url, writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/", writer);
    else
      irradianceFileNames[i] = writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/" + cubeInfo.fileName();
    writer.addTextureToList(irradianceFileNames[i], url);
  }

  if (writer.isX3d()) {
    writer << " ";
    for (int i = 0; i < 6; ++i) {
      if (!backgroundFileNames[i].isEmpty())
        writer << gUrlNames[i] << "='\"" << backgroundFileNames[i] << "\"' ";
      if (!irradianceFileNames[i].isEmpty())
        writer << gIrradianceUrlNames[i] << "='\"" << irradianceFileNames[i] << "\"' ";
    }
  } else if (writer.isVrml()) {
    for (int i = 0; i < 6; ++i) {
      if (!irradianceFileNames[i].isEmpty()) {
        writer.indent();
        writer << gUrlNames[i] << " [ \"" << irradianceFileNames[i] << "\" ]\n";
      }
      if (!irradianceFileNames[i].isEmpty()) {
        writer.indent();
        writer << gIrradianceUrlNames[i] << " [ \"" << irradianceFileNames[i] << "\" ]\n";
      }
    }
  } else
    WbNode::exportNodeFields(writer);
}
