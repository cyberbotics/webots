import {textureFiltering} from './wb_preferences.js';
import {resetIfNotInRangeWithIncludedBounds} from './utils/WbFieldChecker.js';
import ImageLoader from '../ImageLoader.js';
import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbImageTexture extends WbBaseNode {
  #filtering;
  #isTransparent;
  #repeatS;
  #repeatT;
  #url;
  #usedFiltering;
  #wrenTextureIndex;
  constructor(id, url, s, t, filtering) {
    super(id);
    this.#url = url;

    this.#isTransparent = false; // this field is updated whenever loadTextureData is called
    this.#repeatS = s;
    this.#repeatT = t;
    this.#filtering = filtering;

    this.#wrenTextureIndex = 0;
    this.#usedFiltering = 0;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_IMAGE_TEXTURE;
  }

  get isTransparent() {
    return this.#isTransparent;
  }

  set isTransparent(newValue) {
    this.#isTransparent = newValue;
  }

  get filtering() {
    return this.#filtering;
  }

  set filtering(newFiltering) {
    this.#filtering = newFiltering;

    this.#updateFiltering();
  }

  get url() {
    return this.#url;
  }

  set url(newUrl) {
    this.#url = newUrl;

    this.#updateUrl();
  }

  get repeatS() {
    return this.#repeatS;
  }

  set repeatS(newRepeatS) {
    this.#repeatS = newRepeatS;

    this.#update();
  }

  get repeatT() {
    return this.#repeatT;
  }

  set repeatT(newRepeatT) {
    this.#repeatT = newRepeatT;

    this.#update();
  }

  clone(customID) {
    const imageTexture = new WbImageTexture(customID, this.#url, this.#isTransparent, this.#repeatS, this.#repeatT,
      this.#filtering);
    this.useList.push(customID);
    return imageTexture;
  }

  delete() {
    this.#destroyWrenTexture();

    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        if (parent.nodeType === WbNodeType.WB_NODE_APPEARANCE)
          parent.texture = undefined;
        else {
          switch (this.role) {
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
              console.error('Unknown imageTexture: ' + this.id);
          }
        }
      }
    }
    super.delete();
  }

  modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex) {
    if (!wrenMaterial)
      return;

    this.#wrenTextureIndex = mainTextureIndex;
    _wr_material_set_texture(wrenMaterial, this.wrenTexture, this.#wrenTextureIndex);
    if (this.wrenTexture) {
      _wr_texture_set_translucent(this.wrenTexture, this.#isTransparent);
      _wr_material_set_texture_wrap_s(wrenMaterial, this.#repeatS ? Enum.WR_TEXTURE_WRAP_MODE_REPEAT
        : Enum.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, this.#wrenTextureIndex);
      _wr_material_set_texture_wrap_t(wrenMaterial, this.#repeatT ? Enum.WR_TEXTURE_WRAP_MODE_REPEAT
        : Enum.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, this.#wrenTextureIndex);
      _wr_material_set_texture_anisotropy(wrenMaterial, 1 << (this.#usedFiltering - 1), this.#wrenTextureIndex);
      _wr_material_set_texture_enable_interpolation(wrenMaterial, this.#usedFiltering, this.#wrenTextureIndex);
      _wr_material_set_texture_enable_mip_maps(wrenMaterial, this.#usedFiltering, this.#wrenTextureIndex);
    }

    _wr_material_set_texture(wrenMaterial, null, backgroundTextureIndex);
  }

  preFinalize() {
    if (this.isPreFinalizedCalled)
      return;

    super.preFinalize();
    this.updateUrl();
    this.#updateFiltering();
  }

  updateUrl() {
    if (typeof this.#url === 'undefined')
      return;

    // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
    this.#url = this.#url.replaceAll('\\', '/');

    this.#updateWrenTexture();
  }

  // Private functions

  #destroyWrenTexture() {
    _wr_texture_delete(this.wrenTexture);

    this.wrenTexture = undefined;
  }

  #updateFiltering() {
    const newFiltering = resetIfNotInRangeWithIncludedBounds(this.#filtering, 0, 5, 4);
    if (newFiltering !== false) {
      this.filtering = newFiltering;
      return;
    }

    // The filtering level has an upper bound defined by the maximum supported anisotropy level.
    // A warning is not produced here because the maximum anisotropy level is not up to the user
    // and may be repeatedly shown even though a minimum requirement warning was already given.
    this.#usedFiltering = Math.min(this.#filtering, textureFiltering);
    this.#update();
  }

  #updateWrenTexture() {
    this.#destroyWrenTexture();
    // Only load the image from disk if the texture isn't already in the cache
    const texture = Module.ccall('wr_texture_2d_copy_from_cache', 'number', ['string'], [this.#url]);
    if (texture === 0)
      console.warn('Image not found in wren');
    else
      this.#isTransparent = _wr_texture_is_translucent(texture);

    this.wrenTexture = texture;
  }

  #update() {
    if (this.isPostFinalizedCalled && typeof this.onChange === 'function')
      this.onChange();
  }

  #updateUrl() {
    if (typeof this.#url === 'undefined') {
      this.#destroyWrenTexture();
      return;
    }

    ImageLoader.loadImageTextureInWren(this, WbWorld.instance.prefix, this.#url)
      .then(() => {
        this.#updateWrenTexture();
        this.#update();
      });
  }
}
