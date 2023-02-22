import {array3Pointer} from './utils/utils.js';
import {textureQuality} from './wb_preferences.js';
import WbAbstractAppearance from './WbAbstractAppearance.js';
import WbBackground from './WbBackground.js';
import WbVector3 from './utils/WbVector3.js';
import WbWorld from './WbWorld.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import {getAnId} from './utils/id_provider.js';
import {resetIfNegative, resetIfNotInRangeWithIncludedBounds} from './utils/WbFieldChecker.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbPbrAppearance extends WbAbstractAppearance {
  #baseColor;
  #baseColorMap;
  #transparency;
  #roughness;
  #roughnessMap;
  #metalness;
  #metalnessMap;
  #IBLStrength;
  #normalMap;
  #normalMapFactor;
  #occlusionMap;
  #occlusionMapStrength;
  #emissiveColor;
  #emissiveColorMap;
  #emissiveIntensity;
  constructor(id, baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness, metalnessMap,
    IBLStrength, normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor, emissiveColorMap,
    emissiveIntensity, textureTransform) {
    super(id, textureTransform);

    this.#baseColor = baseColor;
    this.baseColorMap = baseColorMap;
    this.#transparency = transparency;
    this.#roughness = roughness;
    this.roughnessMap = roughnessMap;
    this.#metalness = metalness;
    this.metalnessMap = metalnessMap;
    this.#IBLStrength = IBLStrength;
    this.normalMap = normalMap;
    this.#normalMapFactor = normalMapFactor;
    this.occlusionMap = occlusionMap;
    this.#occlusionMapStrength = occlusionMapStrength;
    this.#emissiveColor = emissiveColor;
    this.emissiveColorMap = emissiveColorMap;
    this.#emissiveIntensity = emissiveIntensity;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_PBR_APPEARANCE;
  }

  get baseColor() {
    return this.#baseColor;
  }

  set baseColor(newBaseColor) {
    this.#baseColor = newBaseColor;

    this.#update();
  }

  get baseColorMap() {
    return this.#baseColorMap;
  }

  set baseColorMap(newBaseColorMap) {
    this.#baseColorMap = newBaseColorMap;

    if (typeof this.#baseColorMap !== 'undefined')
      this.#baseColorMap.onChange = () => this.#update();

    this.#update();
  }

  get transparency() {
    return this.#transparency;
  }

  set transparency(newTransparency) {
    this.#transparency = newTransparency;

    this.#updateTransparency();
  }

  get roughness() {
    return this.#roughness;
  }

  set roughness(newRoughness) {
    this.#roughness = newRoughness;

    this.#updateRoughness();
  }

  get roughnessMap() {
    return this.#roughnessMap;
  }

  set roughnessMap(newRoughnessMap) {
    this.#roughnessMap = newRoughnessMap;

    if (typeof this.#roughnessMap !== 'undefined')
      this.#roughnessMap.onChange = () => this.#update();

    this.#update();
  }

  get metalness() {
    return this.#metalness;
  }

  set metalness(newMetalness) {
    this.#metalness = newMetalness;

    this.#updateMetalness();
  }

  get metalnessMap() {
    return this.#metalnessMap;
  }

  set metalnessMap(newMetalnessMap) {
    this.#metalnessMap = newMetalnessMap;

    if (typeof this.#metalnessMap !== 'undefined')
      this.#metalnessMap.onChange = () => this.#update();

    this.#update();
  }

  get IBLStrength() {
    return this.#IBLStrength;
  }

  set IBLStrength(newIBLStrength) {
    this.#IBLStrength = newIBLStrength;

    this.#updateIblStrength();
  }

  get normalMap() {
    return this.#normalMap;
  }

  set normalMap(newNormalMap) {
    this.#normalMap = newNormalMap;

    if (typeof this.#normalMap !== 'undefined')
      this.#normalMap.onChange = () => this.#update();

    this.#update();
  }

  get normalMapFactor() {
    return this.#normalMapFactor;
  }

  set normalMapFactor(newNormalFactor) {
    this.#normalMapFactor = newNormalFactor;

    this.#updateNormalMapFactor();
  }

  get occlusionMap() {
    return this.#occlusionMap;
  }

  set occlusionMap(newOcclustionMap) {
    this.#occlusionMap = newOcclustionMap;

    if (typeof this.#occlusionMap !== 'undefined')
      this.#occlusionMap.onChange = () => this.#update();

    this.#update();
  }

  get occlusionMapStrength() {
    return this.#occlusionMapStrength;
  }

  set occlusionMapStrength(newOcclustionMapStrength) {
    this.#occlusionMapStrength = newOcclustionMapStrength;

    this.#updateOcclusionMapStrength();
  }

  get emissiveColor() {
    return this.#emissiveColor;
  }

  set emissiveColor(newEmissiveColor) {
    this.#emissiveColor = newEmissiveColor;

    this.#update();
  }

  get emissiveColorMap() {
    return this.#emissiveColorMap;
  }

  set emissiveColorMap(newEmissiveColorMap) {
    this.#emissiveColorMap = newEmissiveColorMap;

    if (typeof this.#emissiveColorMap !== 'undefined')
      this.#emissiveColorMap.onChange = () => this.#update();

    this.#update();
  }

  get emissiveIntensity() {
    return this.#emissiveColorMap;
  }

  set emissiveIntensity(newEmissiveIntensity) {
    this.#emissiveIntensity = newEmissiveIntensity;

    this.#update();
  }

  clone(customID) {
    if (typeof customID === 'undefined')
      customID = getAnId();

    let baseColorMap, roughnessMap, metalnessMap, normalMap, occlusionMap, emissiveColorMap, textureTransform;
    if (typeof this.#baseColorMap !== 'undefined') {
      baseColorMap = this.#baseColorMap.clone(getAnId());
      baseColorMap.parent = customID;
      baseColorMap.role = 'baseColorMap';
      WbWorld.instance.nodes.set(baseColorMap.id, baseColorMap);
    }

    if (typeof this.#roughnessMap !== 'undefined') {
      roughnessMap = this.#roughnessMap.clone(getAnId());
      roughnessMap.role = 'roughnessMap';
      roughnessMap.parent = customID;
      WbWorld.instance.nodes.set(roughnessMap.id, roughnessMap);
    }

    if (typeof this.#metalnessMap !== 'undefined') {
      metalnessMap = this.#metalnessMap.clone(getAnId());
      metalnessMap.role = 'metalnessMap';
      metalnessMap.parent = customID;
      WbWorld.instance.nodes.set(metalnessMap.id, metalnessMap);
    }

    if (typeof this.#normalMap !== 'undefined') {
      normalMap = this.#normalMap.clone(getAnId());
      normalMap.role = 'normalMap';
      normalMap.parent = customID;
      WbWorld.instance.nodes.set(normalMap.id, normalMap);
    }

    if (typeof this.#occlusionMap !== 'undefined') {
      occlusionMap = this.#occlusionMap.clone(getAnId());
      occlusionMap.role = 'occlusionMap';
      occlusionMap.parent = customID;
      WbWorld.instance.nodes.set(occlusionMap.id, occlusionMap);
    }

    if (typeof this.#emissiveColorMap !== 'undefined') {
      emissiveColorMap = this.#emissiveColorMap.clone(getAnId());
      emissiveColorMap.role = 'emissiveColorMap';
      emissiveColorMap.parent = customID;
      WbWorld.instance.nodes.set(emissiveColorMap.id, emissiveColorMap);
    }

    if (typeof this.textureTransform !== 'undefined') {
      textureTransform = this.textureTransform.clone(getAnId());
      textureTransform.parent = customID;
      WbWorld.instance.nodes.set(textureTransform.id, textureTransform);
    }

    this.useList.push(customID);
    return new WbPbrAppearance(customID, this.#baseColor, baseColorMap, this.#transparency, this.#roughness, roughnessMap,
      this.#metalness, metalnessMap, this.#IBLStrength, normalMap, this.#normalMapFactor, occlusionMap,
      this.#occlusionMapStrength, this.#emissiveColor, emissiveColorMap, this.#emissiveIntensity, textureTransform);
  }

  createWrenObjects() {
    super.createWrenObjects();
    this.#baseColorMap?.createWrenObjects();
    this.#roughnessMap?.createWrenObjects();
    this.#metalnessMap?.createWrenObjects();
    this.#normalMap?.createWrenObjects();
    this.#occlusionMap?.createWrenObjects();
    this.#emissiveColorMap?.createWrenObjects();
  }

  delete() {
    if (this.isPostFinalizedCalled)
      WbPbrAppearance.cInstanceCounter--;

    if (WbPbrAppearance.cInstanceCounter === 0) {
      _wr_texture_delete(WbPbrAppearance.cBrdfTexture);
      WbPbrAppearance.cBrdfTexture = undefined;
    }

    this.#baseColorMap?.delete();
    this.#roughnessMap?.delete();
    this.#metalnessMap?.delete();
    this.#normalMap?.delete();
    this.#occlusionMap?.delete();
    this.#emissiveColorMap?.delete();

    super.delete();
  }

  modifyWrenMaterial(wrenMaterial) {
    if (!wrenMaterial) {
      _wr_material_delete(wrenMaterial);
      wrenMaterial = _wr_pbr_material_new();
    }

    // set up shaders
    _wr_material_set_default_program(wrenMaterial, WbWrenShaders.pbrShader());
    _wr_material_set_stencil_ambient_emissive_program(wrenMaterial, WbWrenShaders.pbrStencilAmbientEmissiveShader());
    _wr_material_set_stencil_diffuse_specular_program(wrenMaterial, WbWrenShaders.pbrStencilDiffuseSpecularShader());

    // apply textures
    this.#baseColorMap?.modifyWrenMaterial(wrenMaterial, 0, 7);
    this.#roughnessMap?.modifyWrenMaterial(wrenMaterial, 1, 7);
    this.#metalnessMap?.modifyWrenMaterial(wrenMaterial, 2, 7);

    const background = WbBackground.instance;
    let backgroundLuminosity = 1.0;
    if (typeof background !== 'undefined') {
      backgroundLuminosity = background.luminosity;

      // irradiance map
      const irradianceCubeTexture = background.irradianceCubeTexture;
      if (typeof irradianceCubeTexture !== 'undefined') {
        _wr_material_set_texture_cubemap(wrenMaterial, irradianceCubeTexture, 0);
        _wr_material_set_texture_cubemap_wrap_r(wrenMaterial, Enum.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        _wr_material_set_texture_cubemap_wrap_s(wrenMaterial, Enum.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        _wr_material_set_texture_cubemap_wrap_t(wrenMaterial, Enum.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        _wr_material_set_texture_cubemap_anisotropy(wrenMaterial, 8, 0);
        _wr_material_set_texture_cubemap_enable_interpolation(wrenMaterial, true, 0);
        _wr_material_set_texture_cubemap_enable_mip_maps(wrenMaterial, true, 0);
      } else
        _wr_material_set_texture_cubemap(wrenMaterial, null, 0);
    } else
      _wr_material_set_texture_cubemap(wrenMaterial, null, 0);

    this.#normalMap?.modifyWrenMaterial(wrenMaterial, 4, 7);
    this.#occlusionMap?.modifyWrenMaterial(wrenMaterial, 3, 7);
    this.#emissiveColorMap?.modifyWrenMaterial(wrenMaterial, 6, 7);

    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.modifyWrenMaterial(wrenMaterial);
    else
      _wr_material_set_texture_transform(wrenMaterial, null);

    _wr_material_set_texture(wrenMaterial, WbPbrAppearance.cBrdfTexture, 5);
    _wr_material_set_texture_enable_mip_maps(wrenMaterial, false, 5);
    _wr_material_set_texture_enable_interpolation(wrenMaterial, false, 5);

    const baseColorPointer = array3Pointer(this.#baseColor.x, this.#baseColor.y, this.#baseColor.z);
    const emissiveColorPointer = array3Pointer(this.#emissiveColor.x, this.#emissiveColor.y, this.#emissiveColor.z);

    const backgroundColor = new WbVector3(0.0, 0.0, 0.0);

    if (typeof background !== 'undefined') {
      backgroundColor.x = background.skyColor.x;
      backgroundColor.y = background.skyColor.y;
      backgroundColor.z = background.skyColor.z;
    }

    const backgroundColorPointer = array3Pointer(backgroundColor.x, backgroundColor.y, backgroundColor.z);
    // set material properties
    _wr_pbr_material_set_all_parameters(wrenMaterial, backgroundColorPointer, baseColorPointer,
      this.#transparency, this.#roughness, this.#metalness, backgroundLuminosity * this.#IBLStrength, this.#normalMapFactor,
      this.#occlusionMapStrength, emissiveColorPointer, this.#emissiveIntensity);

    _free(baseColorPointer);
    _free(emissiveColorPointer);
    _free(backgroundColorPointer);

    return wrenMaterial;
  }

  preFinalize() {
    super.preFinalize();

    this.#baseColorMap?.preFinalize();
    this.#roughnessMap?.preFinalize();
    this.#metalnessMap?.preFinalize();
    this.#normalMap?.preFinalize();
    this.#occlusionMap?.preFinalize();
    this.#emissiveColorMap?.preFinalize();

    if (WbPbrAppearance.cInstanceCounter === 0) {
      const quality = textureQuality;
      const resolution = Math.pow(2, 6 + quality); // 0: 64, 1: 128, 2: 256
      WbPbrAppearance.cBrdfTexture = _wr_texture_cubemap_bake_brdf(WbWrenShaders.iblBrdfBakingShader(), resolution);
    }
    ++WbPbrAppearance.cInstanceCounter;
  }

  postFinalize() {
    super.postFinalize();

    this.#baseColorMap?.postFinalize();
    this.#roughnessMap?.postFinalize();
    this.#metalnessMap?.postFinalize();
    this.#normalMap?.postFinalize();
    this.#occlusionMap?.postFinalize();
    this.#emissiveColorMap?.postFinalize();
  }

  #update() {
    if (this.isPostFinalizedCalled && typeof this.onChange === 'function')
      this.onChange();
  }

  #updateTransparency() {
    const newTransparency = resetIfNotInRangeWithIncludedBounds(this.#transparency, 0, 1, 0);
    if (newTransparency !== false) {
      this.transparency = newTransparency;
      return;
    }

    this.#update();
  }

  #updateRoughness() {
    const newRoughness = resetIfNotInRangeWithIncludedBounds(this.#roughness, 0, 1, 0);
    if (newRoughness !== false) {
      this.roughness = newRoughness;
      return;
    }

    this.#update();
  }

  #updateMetalness() {
    const newMetalness = resetIfNotInRangeWithIncludedBounds(this.#metalness, 0, 1, 0);
    if (newMetalness !== false) {
      this.metalness = newMetalness;
      return;
    }

    this.#update();
  }

  #updateIblStrength() {
    const newIBLStrength = resetIfNegative(this.#IBLStrength, 1);
    if (newIBLStrength !== false) {
      this.IBLStrength = newIBLStrength;
      return;
    }

    this.#update();
  }

  #updateNormalMapFactor() {
    const newNormalFactor = resetIfNegative(this.#normalMapFactor, 1);
    if (newNormalFactor !== false) {
      this.normalMapFactor = newNormalFactor;
      return;
    }

    this.#update();
  }

  #updateOcclusionMapStrength() {
    const newOcclusionMapStrength = resetIfNegative(this.#occlusionMapStrength, 1);
    if (newOcclusionMapStrength !== false) {
      this.occlusionMapStrength = newOcclusionMapStrength;
      return;
    }

    this.#update();
  }
}

WbPbrAppearance.cInstanceCounter = 0;
