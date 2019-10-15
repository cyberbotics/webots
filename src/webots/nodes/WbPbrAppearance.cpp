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

#include "WbPbrAppearance.hpp"

#include "WbBackground.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbImageTexture.hpp"
#include "WbMaterial.hpp"
#include "WbPreferences.hpp"
#include "WbRgb.hpp"
#include "WbSFColor.hpp"
#include "WbSFNode.hpp"
#include "WbTextureTransform.hpp"
#include "WbWorld.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>
#include <wren/shader_program.h>
#include <wren/texture.h>
#include <wren/texture_cubemap_baker.h>
#include <wren/texture_rtt.h>

static WrTextureRtt *cBrdfTexture = NULL;
static int cInstanceCounter = 0;

void WbPbrAppearance::init() {
  mBaseColor = findSFColor("baseColor");
  mBaseColorMap = findSFNode("baseColorMap");
  mTransparency = findSFDouble("transparency");
  mRoughness = findSFDouble("roughness");
  mRoughnessMap = findSFNode("roughnessMap");
  mMetalness = findSFDouble("metalness");
  mMetalnessMap = findSFNode("metalnessMap");
  mIblStrength = findSFDouble("IBLStrength");
  mNormalMap = findSFNode("normalMap");
  mNormalMapFactor = findSFDouble("normalMapFactor");
  mOcclusionMap = findSFNode("occlusionMap");
  mOcclusionMapStrength = findSFDouble("occlusionMapStrength");
  mEmissiveColor = findSFColor("emissiveColor");
  mEmissiveColorMap = findSFNode("emissiveColorMap");
  mEmissiveIntensity = findSFDouble("emissiveIntensity");
}

WbPbrAppearance::WbPbrAppearance(WbTokenizer *tokenizer) : WbAbstractAppearance("PBRAppearance", tokenizer) {
  init();
}

WbPbrAppearance::WbPbrAppearance(const WbPbrAppearance &other) : WbAbstractAppearance(other) {
  init();
}

WbPbrAppearance::WbPbrAppearance(const WbNode &other) : WbAbstractAppearance(other) {
  init();
}

WbPbrAppearance::~WbPbrAppearance() {
  if (isPostFinalizedCalled())
    --cInstanceCounter;

  if (cInstanceCounter == 0) {
    wr_texture_delete(WR_TEXTURE(cBrdfTexture));
    cBrdfTexture = NULL;
  }
}

void WbPbrAppearance::preFinalize() {
  WbAbstractAppearance::preFinalize();

  if (baseColorMap())
    baseColorMap()->preFinalize();

  if (roughnessMap())
    roughnessMap()->preFinalize();

  if (metalnessMap())
    metalnessMap()->preFinalize();

  if (normalMap())
    normalMap()->preFinalize();

  if (occlusionMap())
    occlusionMap()->preFinalize();

  if (emissiveColorMap())
    emissiveColorMap()->preFinalize();

  updateBaseColorMap();
  updateRoughnessMap();
  updateMetalnessMap();
  updateNormalMap();
  updateOcclusionMap();
  updateEmissiveColorMap();

  if (cInstanceCounter == 0) {
    WbWrenOpenGlContext::makeWrenCurrent();
    const int quality = WbPreferences::instance()->value("OpenGL/textureQuality", 2).toInt();
    const int resolution = pow(2, 6 + quality);  // 0: 64, 1: 128, 2: 256
    cBrdfTexture = wr_texture_cubemap_bake_brdf(WbWrenShaders::iblBrdfBakingShader(), resolution);
    WbWrenOpenGlContext::doneWren();
  }
  ++cInstanceCounter;

  mInitialEmissiveColor = mEmissiveColor->value();
}

void WbPbrAppearance::postFinalize() {
  WbAbstractAppearance::postFinalize();

  if (baseColorMap())
    baseColorMap()->postFinalize();

  if (roughnessMap())
    roughnessMap()->postFinalize();

  if (metalnessMap())
    metalnessMap()->postFinalize();

  if (normalMap())
    normalMap()->postFinalize();

  if (occlusionMap())
    occlusionMap()->postFinalize();

  if (emissiveColorMap())
    emissiveColorMap()->postFinalize();

  connect(mBaseColor, &WbSFColor::changed, this, &WbPbrAppearance::updateBaseColor);
  connect(mBaseColorMap, &WbSFNode::changed, this, &WbPbrAppearance::updateBaseColorMap);
  connect(mTransparency, &WbSFDouble::changed, this, &WbPbrAppearance::updateTransparency);
  connect(mRoughness, &WbSFDouble::changed, this, &WbPbrAppearance::updateRoughness);
  connect(mRoughnessMap, &WbSFNode::changed, this, &WbPbrAppearance::updateRoughnessMap);
  connect(mMetalness, &WbSFDouble::changed, this, &WbPbrAppearance::updateMetalness);
  connect(mMetalnessMap, &WbSFNode::changed, this, &WbPbrAppearance::updateMetalnessMap);
  connect(mIblStrength, &WbSFDouble::changed, this, &WbPbrAppearance::updateIblStrength);
  connect(mNormalMap, &WbSFNode::changed, this, &WbPbrAppearance::updateNormalMap);
  connect(mNormalMapFactor, &WbSFDouble::changed, this, &WbPbrAppearance::updateNormalMapFactor);
  connect(mOcclusionMap, &WbSFNode::changed, this, &WbPbrAppearance::updateOcclusionMap);
  connect(mOcclusionMapStrength, &WbSFDouble::changed, this, &WbPbrAppearance::updateOcclusionMapStrength);
  connect(mEmissiveColor, &WbSFColor::changed, this, &WbPbrAppearance::updateEmissiveColor);
  connect(mEmissiveColorMap, &WbSFNode::changed, this, &WbPbrAppearance::updateEmissiveColorMap);
  connect(mEmissiveIntensity, &WbSFDouble::changed, this, &WbPbrAppearance::updateEmissiveIntensity);

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::backgroundColorChanged, this,
          &WbPbrAppearance::updateBackgroundColor);

  // we need to do one more background probe if there is no local cubemap
  updateCubeMap();

  if (!WbWorld::instance()->isLoading())
    emit changed();
}

void WbPbrAppearance::reset() {
  WbAbstractAppearance::reset();

  if (baseColorMap())
    baseColorMap()->reset();

  if (roughnessMap())
    roughnessMap()->reset();

  if (metalnessMap())
    metalnessMap()->reset();

  if (normalMap())
    normalMap()->reset();

  if (occlusionMap())
    occlusionMap()->reset();

  if (emissiveColorMap())
    emissiveColorMap()->reset();
}

void WbPbrAppearance::setEmissiveColor(const WbRgb &color) {
  mEmissiveColor->setValue(color);
}

void WbPbrAppearance::createWrenObjects() {
  WbAbstractAppearance::createWrenObjects();

  if (baseColorMap())
    baseColorMap()->createWrenObjects();

  if (roughnessMap())
    roughnessMap()->createWrenObjects();

  if (metalnessMap())
    metalnessMap()->createWrenObjects();

  if (normalMap())
    normalMap()->createWrenObjects();

  if (occlusionMap())
    occlusionMap()->createWrenObjects();

  if (emissiveColorMap())
    emissiveColorMap()->createWrenObjects();
}

WrMaterial *WbPbrAppearance::modifyWrenMaterial(WrMaterial *wrenMaterial) {
  if (!wrenMaterial || wrenMaterial->type != WR_MATERIAL_PBR) {
    wr_material_delete(wrenMaterial);
    wrenMaterial = wr_pbr_material_new();
  }

  // set up shaders
  wr_material_set_default_program(wrenMaterial, WbWrenShaders::pbrShader());
  wr_material_set_stencil_ambient_emissive_program(wrenMaterial, WbWrenShaders::pbrStencilAmbientEmissiveShader());
  wr_material_set_stencil_diffuse_specular_program(wrenMaterial, WbWrenShaders::pbrStencilDiffuseSpecularShader());

  // apply textures
  if (baseColorMap())
    baseColorMap()->modifyWrenMaterial(wrenMaterial, 0, 7);

  if (roughnessMap())
    roughnessMap()->modifyWrenMaterial(wrenMaterial, 1, 7);

  if (metalnessMap())
    metalnessMap()->modifyWrenMaterial(wrenMaterial, 2, 7);

  WbBackground *background = WbBackground::firstInstance();
  float backgroundLuminosity = 1.0;
  if (background) {
    backgroundLuminosity = background->luminosity();
    connect(background, &WbBackground::luminosityChanged, this, &WbPbrAppearance::updateCubeMap, Qt::UniqueConnection);

    // diffuse irradiance map
    WrTextureCubeMap *diffuseIrradianceCubeTexture = background->diffuseIrradianceCubeTexture();
    if (diffuseIrradianceCubeTexture) {
      wr_material_set_texture_cubemap(wrenMaterial, diffuseIrradianceCubeTexture, 0);
      wr_material_set_texture_cubemap_wrap_r(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      wr_material_set_texture_cubemap_wrap_s(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      wr_material_set_texture_cubemap_wrap_t(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      wr_material_set_texture_cubemap_anisotropy(wrenMaterial, 8, 0);
      wr_material_set_texture_cubemap_enable_interpolation(wrenMaterial, true, 0);
    } else
      wr_material_set_texture_cubemap(wrenMaterial, NULL, 0);

    // specular irradiance map
    WrTextureCubeMap *specularIrradianceCubeTexture = background->specularIrradianceCubeTexture();
    if (specularIrradianceCubeTexture) {
      wr_material_set_texture_cubemap(wrenMaterial, specularIrradianceCubeTexture, 1);
      wr_material_set_texture_cubemap_wrap_r(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
      wr_material_set_texture_cubemap_wrap_s(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
      wr_material_set_texture_cubemap_wrap_t(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 1);
      wr_material_set_texture_cubemap_anisotropy(wrenMaterial, 8, 1);
      wr_material_set_texture_cubemap_enable_interpolation(wrenMaterial, true, 1);
      wr_material_set_texture_cubemap_enable_mip_maps(wrenMaterial, true, 1);
    } else
      wr_material_set_texture_cubemap(wrenMaterial, NULL, 1);

    connect(background, &WbBackground::cubemapChanged, this, &WbPbrAppearance::updateCubeMap, Qt::UniqueConnection);
  } else {
    wr_material_set_texture_cubemap(wrenMaterial, NULL, 0);
    wr_material_set_texture_cubemap(wrenMaterial, NULL, 1);
  }

  if (normalMap())
    normalMap()->modifyWrenMaterial(wrenMaterial, 4, 7);

  if (occlusionMap())
    occlusionMap()->modifyWrenMaterial(wrenMaterial, 3, 7);

  if (emissiveColorMap())
    emissiveColorMap()->modifyWrenMaterial(wrenMaterial, 6, 7);

  if (textureTransform())
    textureTransform()->modifyWrenMaterial(wrenMaterial);
  else
    wr_material_set_texture_transform(wrenMaterial, NULL);

  wr_material_set_texture(wrenMaterial, WR_TEXTURE(cBrdfTexture), 5);
  wr_material_set_texture_enable_mip_maps(wrenMaterial, false, 5);
  wr_material_set_texture_enable_interpolation(wrenMaterial, false, 5);

  const float baseColor[] = {static_cast<float>(mBaseColor->red()), static_cast<float>(mBaseColor->green()),
                             static_cast<float>(mBaseColor->blue())};

  const float emissiveColor[] = {static_cast<float>(mEmissiveColor->red()), static_cast<float>(mEmissiveColor->green()),
                                 static_cast<float>(mEmissiveColor->blue())};

  float backgroundColor[] = {0.0, 0.0, 0.0};

  if (background) {
    WbRgb skyColor = background->skyColor();
    backgroundColor[0] = static_cast<float>(skyColor.red());
    backgroundColor[1] = static_cast<float>(skyColor.green());
    backgroundColor[2] = static_cast<float>(skyColor.blue());
  }

  // set material properties
  wr_pbr_material_set_all_parameters(wrenMaterial, backgroundColor, baseColor, mTransparency->value(), mRoughness->value(),
                                     mMetalness->value(), backgroundLuminosity * mIblStrength->value(),
                                     mNormalMapFactor->value(), mOcclusionMapStrength->value(), emissiveColor,
                                     mEmissiveIntensity->value());

  return wrenMaterial;
}

WbImageTexture *WbPbrAppearance::baseColorMap() const {
  return dynamic_cast<WbImageTexture *>(mBaseColorMap->value());
}

WbImageTexture *WbPbrAppearance::roughnessMap() const {
  return dynamic_cast<WbImageTexture *>(mRoughnessMap->value());
}

WbImageTexture *WbPbrAppearance::metalnessMap() const {
  return dynamic_cast<WbImageTexture *>(mMetalnessMap->value());
}

WbImageTexture *WbPbrAppearance::normalMap() const {
  return dynamic_cast<WbImageTexture *>(mNormalMap->value());
}

WbImageTexture *WbPbrAppearance::occlusionMap() const {
  return dynamic_cast<WbImageTexture *>(mOcclusionMap->value());
}

WbImageTexture *WbPbrAppearance::emissiveColorMap() const {
  return dynamic_cast<WbImageTexture *>(mEmissiveColorMap->value());
}

bool WbPbrAppearance::isBaseColorTextureLoaded() const {
  return (baseColorMap() && baseColorMap()->wrenTexture());
}

bool WbPbrAppearance::isRoughnessTextureLoaded() const {
  return (roughnessMap() && roughnessMap()->wrenTexture());
}

bool WbPbrAppearance::isOcclusionTextureLoaded() const {
  return (occlusionMap() && occlusionMap()->wrenTexture());
}

WbRgb WbPbrAppearance::baseColor() const {
  return mBaseColor->value();
}

double WbPbrAppearance::transparency() const {
  return mTransparency->value();
}

double WbPbrAppearance::roughness() const {
  return mRoughness->value();
}

void WbPbrAppearance::pickColorInBaseColorTexture(WbRgb &pickedColor, const WbVector2 &uv) const {
  WbImageTexture *tex = baseColorMap();
  if (tex) {
    WbVector2 uvTransformed = transformUVCoordinate(uv);
    tex->pickColor(pickedColor, uvTransformed);
  } else
    pickedColor.setValue(1.0, 1.0, 1.0);  // default value
}

void WbPbrAppearance::pickRoughnessInTexture(double *roughness, const WbVector2 &uv) const {
  *roughness = getRedValueInTexture(roughnessMap(), uv);
}

void WbPbrAppearance::pickOcclusionInTexture(double *occlusion, const WbVector2 &uv) const {
  *occlusion = getRedValueInTexture(occlusionMap(), uv);
}

double WbPbrAppearance::getRedValueInTexture(const WbImageTexture *texture, const WbVector2 &uv) const {
  if (texture) {
    WbRgb pickedColor;
    WbVector2 uvTransformed = transformUVCoordinate(uv);
    texture->pickColor(pickedColor, uvTransformed);
    return pickedColor.red();
  }
  return 0.0;  // default value
}

void WbPbrAppearance::updateCubeMap() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateBaseColor() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateBackgroundColor() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateBaseColorMap() {
  if (baseColorMap())
    connect(baseColorMap(), &WbImageTexture::changed, this, &WbPbrAppearance::updateBaseColorMap, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateTransparency() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransparency, 0.0, 1.0, 0.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateRoughness() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mRoughness, 0.0, 1.0, 0.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateRoughnessMap() {
  if (roughnessMap())
    connect(roughnessMap(), &WbImageTexture::changed, this, &WbPbrAppearance::updateRoughnessMap, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateMetalness() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mMetalness, 0.0, 1.0, 0.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateMetalnessMap() {
  if (metalnessMap())
    connect(metalnessMap(), &WbImageTexture::changed, this, &WbPbrAppearance::updateMetalnessMap, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateIblStrength() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mIblStrength, 1.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateNormalMap() {
  if (normalMap())
    connect(normalMap(), &WbImageTexture::changed, this, &WbPbrAppearance::updateNormalMap, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateNormalMapFactor() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mNormalMapFactor, 1.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateOcclusionMap() {
  if (occlusionMap())
    connect(occlusionMap(), &WbImageTexture::changed, this, &WbPbrAppearance::updateOcclusionMap, Qt::UniqueConnection);
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateOcclusionMapStrength() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mOcclusionMapStrength, 1.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateEmissiveColor() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateEmissiveColorMap() {
  if (emissiveColorMap())
    connect(emissiveColorMap(), &WbImageTexture::changed, this, &WbPbrAppearance::updateEmissiveColorMap, Qt::UniqueConnection);
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::updateEmissiveIntensity() {
  if (isPostFinalizedCalled())
    emit changed();
}

void WbPbrAppearance::exportNodeSubNodes(WbVrmlWriter &writer) const {
  if (writer.isWebots()) {
    WbAbstractAppearance::exportNodeSubNodes(writer);
    return;
  }

  if (writer.isX3d()) {
    writer << "<Material diffuseColor=\"";
    mBaseColor->write(writer);
    writer << "\" specularColor=\"" << 1.0 - mRoughness->value() << " " << 1.0 - mRoughness->value() << " "
           << 1.0 - mRoughness->value() << "\" ";
    writer << "shininess=\"" << 1.0 - mRoughness->value() << "\"";
    writer << "/>";
    mBaseColorMap->write(writer);
    if (textureTransform())
      textureTransform()->write(writer);
  } else if (writer.isVrml()) {
    // export as vrml
    writer.indent();
    writer << "material Material {\n";
    writer.increaseIndent();
    writer.indent();
    writer << "diffuseColor ";
    mBaseColor->write(writer);
    writer << "\n";
    writer.indent();
    writer << "specularColor " << 1.0 - mRoughness->value() << " " << 1.0 - mRoughness->value() << " "
           << 1.0 - mRoughness->value() << "\n";
    writer.indent();
    writer << "shininess " << 1.0 - mRoughness->value() << "\n";
    writer.decreaseIndent();
    writer.indent();
    writer << "}\n";
    if (mBaseColorMap->value()) {
      writer.indent();
      writer << "texture ";
      mBaseColorMap->write(writer);
      writer << "\n";
    }
    if (mTextureTransform->value()) {
      writer.indent();
      writer << "textureTransform ";
      mTextureTransform->write(writer);
      writer << "\n";
    }
  }
}

void WbPbrAppearance::exportNodeFooter(WbVrmlWriter &writer) const {
  WbAbstractAppearance::exportNodeFooter(writer);

  if (!writer.isX3d())
    return;

  writer << "<PBRAppearance id=\'n" << QString::number(uniqueId()) << "\'";

  if (isUseNode() && defNode()) {
    writer << " USE=\'" + QString::number(defNode()->uniqueId()) + "\'></PBRAppearance>";
    return;
  }

  foreach (WbField *field, fields())
    if (field->singleType() != WB_SF_NODE)
      field->write(writer);

  writer << ">";

  if (baseColorMap()) {
    baseColorMap()->setRole("baseColor");
    baseColorMap()->write(writer);
  }
  if (roughnessMap()) {
    roughnessMap()->setRole("roughness");
    roughnessMap()->write(writer);
  }
  if (metalnessMap()) {
    metalnessMap()->setRole("metalness");
    metalnessMap()->write(writer);
  }
  if (normalMap()) {
    normalMap()->setRole("normal");
    normalMap()->write(writer);
  }
  if (occlusionMap()) {
    occlusionMap()->setRole("occlusion");
    occlusionMap()->write(writer);
  }
  if (emissiveColorMap()) {
    emissiveColorMap()->setRole("emissiveColor");
    emissiveColorMap()->write(writer);
  }

  if (textureTransform())
    textureTransform()->write(writer);

  writer << "</PBRAppearance>";
}

QStringList WbPbrAppearance::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "baseColor"
         << "emissiveColor";
  return fields;
}
