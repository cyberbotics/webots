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

#include "WbCubemap.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbMFColor.hpp"
#include "WbMFString.hpp"
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

QList<WbBackground *> WbBackground::cBackgroundList;
static QString gUrlNames[6] = {"rightUrl", "leftUrl", "topUrl", "bottomUrl", "frontUrl", "backUrl"};

void WbBackground::init() {
  mSkyColor = findMFColor("skyColor");
  mSkyColorMap = findSFNode("skyColorMap");
  mSpecularIrradianceMap = findSFNode("specularIrradianceMap");
  mDiffuseIrradianceMap = findSFNode("diffuseIrradianceMap");
  mLuminosity = findSFDouble("luminosity");

  if (!mSkyColorMap->value()) {
    // backward compatibility before R2018b and import from VRML97
    WbMFString *urlFields[6];
    for (int i = 0; i < 6; ++i)
      urlFields[i] = findMFString(gUrlNames[i]);

    bool isValid = true;
    QString directory;
    QString textureBaseName;
    for (int i = 0; i < 6; ++i) {
      if (!urlFields[i] || urlFields[i]->size() == 0) {
        // empty url
        isValid = false;
        if (i != 0)
          warn(tr("Impossible to create the skyColorMap cubemap because not all the url fields are defined."));
        break;
      }
      const QFileInfo fileInfo(urlFields[i]->item(0));
      const QString currentDirectory = fileInfo.dir().path();
      QString currentTextureBaseName = fileInfo.baseName();

      int index = currentTextureBaseName.lastIndexOf(WbCubemap::textureSuffixes()[i]);
      if (index < 0) {
        // suffix not found
        isValid = false;
        warn(tr("Impossible to create the skyColorMap cubemap because the texture file defined in the '%1' field doesn't end "
                "with the '%2' "
                "suffix.")
               .arg(gUrlNames[i])
               .arg(WbCubemap::textureSuffixes()[i]));
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
        isValid = false;
        break;
      }
    }

    if (isValid) {
      WbNode *previousParent = WbNode::globalParent();
      WbNodeOperations::instance()->importNode(
        this, findField("skyColorMap"), 0, "",
        QString("Cubemap { textureBaseName \"" + textureBaseName + "\" directory \"" + directory + "\" }"));
      WbNode::setGlobalParent(previousParent);
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

  if (mSkyboxMaterial) {
    wr_material_delete(mSkyboxMaterial);
    mSkyboxMaterial = NULL;
  }

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

  if (skyColorMap())
    skyColorMap()->preFinalize();

  if (specularIrradianceMap())
    specularIrradianceMap()->preFinalize();

  if (diffuseIrradianceMap())
    diffuseIrradianceMap()->preFinalize();

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

  if (skyColorMap())
    skyColorMap()->postFinalize();

  if (specularIrradianceMap())
    specularIrradianceMap()->postFinalize();

  if (diffuseIrradianceMap())
    diffuseIrradianceMap()->postFinalize();

  connect(mSkyColor, &WbMFColor::changed, this, &WbBackground::updateColor);
  connect(mSkyColorMap, &WbSFNode::changed, this, &WbBackground::updateSkyColorMap);
  connect(mSpecularIrradianceMap, &WbSFNode::changed, this, &WbBackground::updateSpecularIrradianceMap);
  connect(mDiffuseIrradianceMap, &WbSFNode::changed, this, &WbBackground::updateDiffuseIrradianceMap);

  updateColor();

  updateSkyColorMap();
  updateSpecularIrradianceMap();
  updateDiffuseIrradianceMap();
}

void WbBackground::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  if (skyColorMap())
    skyColorMap()->createWrenObjects();

  if (specularIrradianceMap())
    specularIrradianceMap()->createWrenObjects();

  if (diffuseIrradianceMap())
    diffuseIrradianceMap()->createWrenObjects();

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
  emit texturesDestroyed();
  if (mDiffuseIrradianceCubeTexture) {
    wr_texture_delete(WR_TEXTURE(mDiffuseIrradianceCubeTexture));
    mDiffuseIrradianceCubeTexture = NULL;
  }
  if (mSpecularIrradianceCubeTexture) {
    wr_texture_delete(WR_TEXTURE(mSpecularIrradianceCubeTexture));
    mSpecularIrradianceCubeTexture = NULL;
  }
  if (skyColorMap()) {
    skyColorMap()->clearWrenTexture();
    disconnect(skyColorMap(), &WbCubemap::destroyed, this, &WbBackground::updateSkyColorMap);
  }
  if (specularIrradianceMap()) {
    specularIrradianceMap()->clearWrenTexture();
    disconnect(specularIrradianceMap(), &WbCubemap::destroyed, this, &WbBackground::updateSpecularIrradianceMap);
  }
  if (diffuseIrradianceMap()) {
    diffuseIrradianceMap()->clearWrenTexture();
    disconnect(diffuseIrradianceMap(), &WbCubemap::destroyed, this, &WbBackground::updateDiffuseIrradianceMap);
  }
}

void WbBackground::updateColor() {
  if (WbFieldChecker::resetMultipleColorIfInvalid(this, mSkyColor))
    return;

  if (areWrenObjectsInitialized())
    applyColourToWren(skyColor());

  emit WbWrenRenderingContext::instance()->backgroundColorChanged();
}

void WbBackground::updateSkyColorMap() {
  if (skyColorMap()) {
    connect(skyColorMap(), &WbCubemap::changed, this, &WbBackground::updateSkyColorMap, Qt::UniqueConnection);
    emit skyColorMapChanged();
  }

  if (areWrenObjectsInitialized())
    applySkyBoxToWren();
}

void WbBackground::updateSpecularIrradianceMap() {
  if (specularIrradianceMap()) {
    connect(specularIrradianceMap(), &WbCubemap::changed, this, &WbBackground::updateSpecularIrradianceMap,
            Qt::UniqueConnection);
    emit specularIrradianceMapChanged();
  }

  if (areWrenObjectsInitialized())
    applySkyBoxToWren();
}

void WbBackground::updateDiffuseIrradianceMap() {
  if (diffuseIrradianceMap()) {
    connect(diffuseIrradianceMap(), &WbCubemap::changed, this, &WbBackground::updateDiffuseIrradianceMap, Qt::UniqueConnection);
    emit diffuseIrradianceMapChanged();
  }

  if (areWrenObjectsInitialized())
    applySkyBoxToWren();
}

void WbBackground::updateLuminosity() {
  // TODO
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

  if (skyColorMap()) {
    skyColorMap()->loadWrenTexture();
    if (skyColorMap()->isValid())
      connect(skyColorMap(), &WbCubemap::destroyed, this, &WbBackground::updateSkyColorMap, Qt::UniqueConnection);
  }
  if (specularIrradianceMap()) {
    specularIrradianceMap()->loadWrenTexture();
    if (specularIrradianceMap()->isValid())
      connect(specularIrradianceMap(), &WbCubemap::destroyed, this, &WbBackground::updateSpecularIrradianceMap,
              Qt::UniqueConnection);
  }
  if (diffuseIrradianceMap()) {
    diffuseIrradianceMap()->loadWrenTexture();
    if (diffuseIrradianceMap()->isValid())
      connect(diffuseIrradianceMap(), &WbCubemap::destroyed, this, &WbBackground::updateDiffuseIrradianceMap,
              Qt::UniqueConnection);
  }

  WbWrenOpenGlContext::makeWrenCurrent();

  if (!diffuseIrradianceMap() || !diffuseIrradianceMap()->isValid()) {
    mDiffuseIrradianceCubeTexture = wr_texture_cubemap_bake_diffuse_irradiance(
      skyColorMap()->texture(), WbWrenShaders::iblDiffuseIrradianceBakingShader(), 32);
  }

  if (!specularIrradianceMap() || !specularIrradianceMap()->isValid()) {
    const int quality = WbPreferences::instance()->value("OpenGL/textureQuality", 2).toInt();
    // maps the quality eihter to '0: 64, 1: 128, 2: 256' or in case of HDR to '0: 32, 1: 64, 2: 256'
    const int offset = (skyColorMap()->isEquirectangular() &&
                        (quality < 2 || WbPreferences::instance()->value("OpenGL/limitBakingResolution", false).toBool())) ?
                         5 :
                         6;
    const int resolution = 1 << (offset + quality);
    mSpecularIrradianceCubeTexture = wr_texture_cubemap_bake_specular_irradiance(
      skyColorMap()->texture(), WbWrenShaders::iblSpecularIrradianceBakingShader(), resolution);
    wr_texture_cubemap_disable_automatic_mip_map_generation(mSpecularIrradianceCubeTexture);
  }

  if (skyColorMap() && skyColorMap()->isValid()) {
    wr_material_set_texture_cubemap(mSkyboxMaterial, skyColorMap()->texture(), 0);
    wr_material_set_texture_cubemap_wrap_r(mSkyboxMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
    wr_material_set_texture_cubemap_wrap_s(mSkyboxMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
    wr_material_set_texture_cubemap_wrap_t(mSkyboxMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
    wr_scene_set_skybox(wr_scene_get_instance(), mSkyboxRenderable);
  }

  WbWrenOpenGlContext::doneWren();

  emit texturesLoaded();
}

WbRgb WbBackground::skyColor() const {
  return (mSkyColor->size() > 0 ? mSkyColor->item(0) : WbRgb());
}

WbCubemap *WbBackground::skyColorMap() const {
  return dynamic_cast<WbCubemap *>(mSkyColorMap->value());
}

WbCubemap *WbBackground::specularIrradianceMap() const {
  return dynamic_cast<WbCubemap *>(mSpecularIrradianceMap->value());
}

WbCubemap *WbBackground::diffuseIrradianceMap() const {
  return dynamic_cast<WbCubemap *>(mDiffuseIrradianceMap->value());
}

void WbBackground::modifyWrenMaterial(WrMaterial *material) {
  if (!material)
    return;

  // diffuse irradiance map
  if (diffuseIrradianceMap() && diffuseIrradianceMap()->isValid())
    wr_material_set_texture_cubemap(material, diffuseIrradianceMap()->texture(), 0);
  else
    wr_material_set_texture_cubemap(material, mDiffuseIrradianceCubeTexture, 0);
  wr_material_set_texture_cubemap_wrap_r(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_material_set_texture_cubemap_wrap_s(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_material_set_texture_cubemap_wrap_t(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
  wr_material_set_texture_cubemap_anisotropy(material, 8, 0);
  wr_material_set_texture_cubemap_enable_interpolation(material, true, 0);

  // specular irradiance map
  if (specularIrradianceMap() && specularIrradianceMap()->isValid())
    wr_material_set_texture_cubemap(material, specularIrradianceMap()->texture(), 1);
  else
    wr_material_set_texture_cubemap(material, mSpecularIrradianceCubeTexture, 1);
  wr_material_set_texture_cubemap_wrap_r(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
  wr_material_set_texture_cubemap_wrap_s(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
  wr_material_set_texture_cubemap_wrap_t(material, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
  wr_material_set_texture_cubemap_anisotropy(material, 8, 1);
  wr_material_set_texture_cubemap_enable_interpolation(material, true, 1);
  wr_material_set_texture_cubemap_enable_mip_maps(material, true, 1);
}

void WbBackground::exportNodeFields(WbVrmlWriter &writer) const {
  if (writer.isWebots()) {
    WbBaseNode::exportNodeFields(writer);
    return;
  }

  findField("skyColor", true)->write(writer);

  // TODO: export

  if (!skyColorMap() || !skyColorMap()->isValid())
    return;

  if (skyColorMap()->isEquirectangular()) {
    // supported only for x3d export
    if (!writer.isX3d())
      return;
    const QString &textureUrl(skyColorMap()->equirectangularTextureUrl());
    const QFileInfo &cubeInfo(textureUrl);
    QString outputFileName;
    if (writer.isWritingToFile())
      outputFileName = WbUrl::exportTexture(this, textureUrl, textureUrl,
                                            writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/", writer);
    else
      outputFileName = writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/" + cubeInfo.fileName();
    writer.addTextureToList(outputFileName, textureUrl);
    writer << " hdrUrl='" << outputFileName << "' ";
    return;
  }

  QString outputFileNames[6];
  for (int i = 0; i < 6; ++i) {
    const QFileInfo &cubeInfo(skyColorMap()->textureUrls(i));
    if (writer.isWritingToFile())
      outputFileNames[i] = WbUrl::exportTexture(this, skyColorMap()->textureUrls(i), skyColorMap()->textureUrls(i),
                                                writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/", writer);
    else
      outputFileNames[i] = writer.relativeTexturesPath() + cubeInfo.dir().dirName() + "/" + cubeInfo.fileName();
    writer.addTextureToList(outputFileNames[i], skyColorMap()->textureUrls(i));
  }

  if (writer.isX3d()) {
    writer << " ";
    for (int i = 0; i < 6; ++i)
      writer << gUrlNames[i] << "='\"" << outputFileNames[i] << "\"' ";
  } else if (writer.isVrml()) {
    for (int i = 0; i < 6; ++i) {
      writer.indent();
      writer << gUrlNames[i] << " [ \"" << outputFileNames[i] << "\" ]\n";
    }
  } else
    WbNode::exportNodeFields(writer);
}
