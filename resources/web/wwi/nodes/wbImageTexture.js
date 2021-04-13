import {arrayXPointer} from './utils/wbUtils.js';
import {textureFiltering} from './wbPreferences.js';
import {WbAppearance} from './wbAppearance.js';
import {WbBaseNode} from './wbBaseNode.js';
import {WbWorld} from './wbWorld.js';

import {Parser} from './../parser.js';

class WbImageTexture extends WbBaseNode {
  constructor(id, url, isTransparent, s, t, filtering) {
    super(id);
    this.url = url;

    this.isTransparent = isTransparent;
    this.repeatS = s;
    this.repeatT = t;
    this.filtering = filtering;

    this.wrenTextureIndex = 0;
    this.usedFiltering = 0;
  }

  delete() {
    this.destroyWrenTexture();

    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        if (parent instanceof WbAppearance)
          parent.texture = undefined;
        else {
          switch (this.type) {
            case 'baseColorMap':
              parent.baseColorMap = undefined;
              break;
            case 'roughnessMap':
              parent.roughnessMap = undefined;
              break;
            case 'metalnessMap':
              parent.metalnessMap = undefined;
              break;
            case 'normalMap':
              parent.normalMap = undefined;
              break;
            case 'occlusionMap':
              parent.occlusionMap = undefined;
              break;
            case 'emissiveColorMap':
              parent.emissiveColorMap = undefined;
              break;
            default:
              console.error('unknow imageTexture: ' + this.id);
          }
        }
      }
    }
    super.delete();
  }

  modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex) {
    if (!wrenMaterial)
      return;
    this.wrenTextureIndex = mainTextureIndex;
    _wr_material_set_texture(wrenMaterial, this.wrenTexture, this.wrenTextureIndex);
    if (this.wrenTexture) {
      _wr_texture_set_translucent(this.wrenTexture, this.isTransparent);
      _wr_material_set_texture_wrap_s(wrenMaterial, this.repeatS ? ENUM.WR_TEXTURE_WRAP_MODE_REPEAT : ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, this.wrenTextureIndex);
      _wr_material_set_texture_wrap_t(wrenMaterial, this.repeatT ? ENUM.WR_TEXTURE_WRAP_MODE_REPEAT : ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, this.wrenTextureIndex);
      _wr_material_set_texture_anisotropy(wrenMaterial, 1 << (this.usedFiltering - 1), this.wrenTextureIndex);
      _wr_material_set_texture_enable_interpolation(wrenMaterial, this.usedFiltering, this.wrenTextureIndex);
      _wr_material_set_texture_enable_mip_maps(wrenMaterial, this.usedFiltering, this.wrenTextureIndex);
    }

    _wr_material_set_texture(wrenMaterial, null, backgroundTextureIndex);
  }

  async updateWrenTexture() {
    this.destroyWrenTexture();
    // Only load the image from disk if the texture isn't already in the cache
    let texture = Module.ccall('wr_texture_2d_copy_from_cache', 'number', ['string'], [this.url]);
    if (texture === 0) {
      const image = await Parser.loadTextureData(this.url);
      texture = _wr_texture_2d_new();
      _wr_texture_set_size(texture, image.width, image.height);
      _wr_texture_set_translucent(texture, this.isTransparent);
      const bitsPointer = arrayXPointer(image.bits);
      _wr_texture_2d_set_data(texture, bitsPointer);
      Module.ccall('wr_texture_2d_set_file_path', null, ['number', 'string'], [texture, this.url]);
      _wr_texture_setup(texture);
      _free(bitsPointer);
    } else
      this.isTransparent = _wr_texture_is_translucent(texture);

    this.wrenTexture = texture;
  }

  destroyWrenTexture() {
    _wr_texture_delete(this.wrenTexture);

    _wr_texture_transform_delete(this.wrenTextureTransform);

    this.wrenTexture = undefined;
    this.wrenTextureTransform = undefined;
  }

  preFinalize() {
    super.preFinalize();
    this.updateFiltering();
  }

  async updateUrl() {
    // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
    this.url = this.url.replaceAll('\\', '/');

    await this.updateWrenTexture();
  }

  updateFiltering() {
    // The filtering level has an upper bound defined by the maximum supported anisotropy level.
    // A warning is not produced here because the maximum anisotropy level is not up to the user
    // and may be repeatedly shown even though a minimum requirement warning was already given.
    this.usedFiltering = Math.min(this.filtering, textureFiltering);
  }

  clone(customID) {
    const imageTexture = new WbImageTexture(customID, this.url, this.isTransparent, this.repeatS, this.repeatT, this.filtering);
    imageTexture.updateUrl();
    this.useList.push(customID);
    return imageTexture;
  }
}

export {WbImageTexture};
