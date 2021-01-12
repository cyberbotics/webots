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
import {World} from "./World.js";
import {WbAbstractAppearance} from "./WbAbstractAppearance.js"
import {WbWrenShaders} from "./WbWrenShaders.js";
import {WbBackground} from "./WbBackground.js";
import {array3Pointer} from "./WbUtils.js";
import {textureQuality} from "./WbPreferences.js";

class WbPBRAppearance extends WbAbstractAppearance {
  constructor(id, baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness, metalnessMap,
    IBLStrength, normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor, emissiveColorMap, emissiveIntensity, textureTransform) {
    super(id, textureTransform);

    this.baseColor = baseColor;
    this.baseColorMap = baseColorMap;
    this.transparency = transparency;
    this.roughness = roughness;
    this.roughnessMap = roughnessMap;
    this.metalness = metalness;
    this.metalnessMap = metalnessMap;
    this.IBLStrength = IBLStrength;
    this.normalMap = normalMap;
    this.normalMapFactor = normalMapFactor;
    this.occlusionMap = occlusionMap;
    this.occlusionMapStrength = occlusionMapStrength;
    this.emissiveColor = emissiveColor;
    this.emissiveColorMap = emissiveColorMap;
    this.emissiveIntensity = emissiveIntensity;
  }

  delete(){
    if (this.isPostFinalizeCalled)
      WbPBRAppearance.cInstanceCounter--;

    if (WbPBRAppearance.cInstanceCounter == 0) {
      _wr_texture_delete(WbPBRAppearance.cBrdfTexture);
      WbPBRAppearance.cBrdfTexture = undefined;
    }

    if (typeof this.baseColorMap !== 'undefined')
      this.baseColorMap.delete();

    if (typeof this.roughnessMap !== 'undefined')
      this.roughnessMap.delete();

    if (typeof this.metalnessMap !== 'undefined')
      this.metalnessMap.delete();

    if (typeof this.normalMap !== 'undefined')
      this.normalMap.delete();

    if (typeof this.occlusionMap !== 'undefined')
      this.occlusionMap.delete();

    if (typeof this.emissiveColorMap !== 'undefined')
      this.emissiveColorMap.delete();

    super.delete();
  }

  createWrenObjects(){
    super.createWrenObjects();
    if (typeof this.baseColorMap !== 'undefined')
      this.baseColorMap.createWrenObjects();

    if (typeof this.roughnessMap !== 'undefined')
      this.roughnessMap.createWrenObjects();

    if (typeof this.metalnessMap !== 'undefined')
      this.metalnessMap.createWrenObjects();

    if (typeof this.normalMap !== 'undefined')
      this.normalMap.createWrenObjects();

    if (typeof this.occlusionMap !== 'undefined')
      this.occlusionMap.createWrenObjects();

    if (typeof this.emissiveColorMap !== 'undefined')
      this.emissiveColorMap.createWrenObjects();
  }

  modifyWrenMaterial(wrenMaterial) {
    if (!wrenMaterial) { //TODO check if not pbr material
      _wr_material_delete(wrenMaterial);
      wrenMaterial = _wr_pbr_material_new();
    }

    // set up shaders
    _wr_material_set_default_program(wrenMaterial, WbWrenShaders.pbrShader());
    _wr_material_set_stencil_ambient_emissive_program(wrenMaterial, WbWrenShaders.pbrStencilAmbientEmissiveShader());
    _wr_material_set_stencil_diffuse_specular_program(wrenMaterial, WbWrenShaders.pbrStencilDiffuseSpecularShader());

    // apply textures
    if (typeof this.baseColorMap !== 'undefined')
      this.baseColorMap.modifyWrenMaterial(wrenMaterial, 0, 7);

    if (typeof this.roughnessMap !== 'undefined')
      this.roughnessMap.modifyWrenMaterial(wrenMaterial, 1, 7);

    if (typeof this.metalnessMap !== 'undefined')
      this.metalnessMap.modifyWrenMaterial(wrenMaterial, 2, 7);

    let background = WbBackground.instance;
    let backgroundLuminosity = 1.0;
    if (typeof background !== 'undefined') {
      backgroundLuminosity = background.luminosity;

      // irradiance map
      let irradianceCubeTexture = background.irradianceCubeTexture;
      if (typeof irradianceCubeTexture !== 'undefined') {
        _wr_material_set_texture_cubemap(wrenMaterial, irradianceCubeTexture, 0);
        _wr_material_set_texture_cubemap_wrap_r(wrenMaterial, ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        _wr_material_set_texture_cubemap_wrap_s(wrenMaterial, ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        _wr_material_set_texture_cubemap_wrap_t(wrenMaterial, ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        _wr_material_set_texture_cubemap_anisotropy(wrenMaterial, 8, 0);
        _wr_material_set_texture_cubemap_enable_interpolation(wrenMaterial, false, 0);
        _wr_material_set_texture_cubemap_enable_mip_maps(wrenMaterial, true, 0);
      } else
        _wr_material_set_texture_cubemap(wrenMaterial, null, 0);
    } else
      _wr_material_set_texture_cubemap(wrenMaterial, null, 0);

    if (typeof this.normalMap !== 'undefined')
      this.normalMap.modifyWrenMaterial(wrenMaterial, 4, 7);

    if (typeof this.occlusionMap !== 'undefined')
      this.occlusionMap.modifyWrenMaterial(wrenMaterial, 3, 7);

    if (typeof this.emissiveColorMap !== 'undefined'){
      this.emissiveColorMap.modifyWrenMaterial(wrenMaterial, 6, 7);
    }
    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.modifyWrenMaterial(wrenMaterial);
    else
      _wr_material_set_texture_transform(wrenMaterial, null);

    _wr_material_set_texture(wrenMaterial, WbPBRAppearance.cBrdfTexture, 5);
    _wr_material_set_texture_enable_mip_maps(wrenMaterial, false, 5);
    _wr_material_set_texture_enable_interpolation(wrenMaterial, false, 5);

    let baseColorPointer = array3Pointer(this.baseColor.x, this.baseColor.y, this.baseColor.z);
    let emissiveColorPointer = array3Pointer(this.emissiveColor.x, this.emissiveColor.y, this.emissiveColor.z);


    let backgroundColor = glm.vec3(0.0, 0.0, 0.0);

    if (typeof background !== 'undefined') {
      backgroundColor.x = background.skyColor.x;
      backgroundColor.y = background.skyColor.y;
      backgroundColor.z = background.skyColor.z;
    }

    let backgroundColorPointer = array3Pointer(backgroundColor.x, backgroundColor.y, backgroundColor.z);
    // set material properties
    _wr_pbr_material_set_all_parameters(wrenMaterial, backgroundColorPointer, baseColorPointer,
      this.transparency, this.roughness, this.metalness, backgroundLuminosity * this.IBLStrength,this.normalMapFactor,
      this.occlusionMapStrength, emissiveColorPointer,this.emissiveIntensity);

    return wrenMaterial;
  }

  preFinalize() {
    super.preFinalize();

    if (typeof this.baseColorMap !== 'undefined')
      this.baseColorMap.preFinalize();

    if (typeof this.roughnessMap !== 'undefined')
      this.roughnessMap.preFinalize();

    if (typeof this.metalnessMap !== 'undefined')
      this.metalnessMap.preFinalize();

    if (typeof this.normalMap !== 'undefined')
      this.normalMap.preFinalize();

    if (typeof this.occlusionMap !== 'undefined')
      this.occlusionMap.preFinalize();

    if (typeof this.emissiveColorMap !== 'undefined')
      this.emissiveColorMap.preFinalize();


    if (WbPBRAppearance.cInstanceCounter == 0) {
      let quality = textureQuality;
      let resolution = Math.pow(2, 6 + quality);  // 0: 64, 1: 128, 2: 256
      WbPBRAppearance.cBrdfTexture = _wr_texture_cubemap_bake_brdf(WbWrenShaders.iblBrdfBakingShader(), resolution);
    }
    ++WbPBRAppearance.cInstanceCounter;
  }

  postFinalize() {
    super.postFinalize();

    if (typeof this.baseColorMap !== 'undefined')
      this.baseColorMap.postFinalize();

    if (typeof this.roughnessMap !== 'undefined')
      this.roughnessMap.postFinalize();

    if (typeof this.metalnessMap !== 'undefined')
      this.metalnessMap.postFinalize();

    if (typeof this.normalMap !== 'undefined')
      this.normalMap.postFinalize();

    if (typeof this.occlusionMap !== 'undefined')
      this.occlusionMap.postFinalize();

    if (typeof this.emissiveColorMap !== 'undefined')
      this.emissiveColorMap.postFinalize();
  }

}

WbPBRAppearance.cBrdfTexture = undefined;
WbPBRAppearance.cInstanceCounter = 0;

export {WbPBRAppearance}
