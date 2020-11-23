import {WbBaseNode} from "./WbBaseNode.js"
import {arrayXPointer} from "./WbUtils.js";


class WbImageTexture extends WbBaseNode {
  constructor(id, url, isTransparent, s, t, anisotropy, image){
    super(id);
    this.url = url;

    this.isTransparent = isTransparent;
    this.repeatS = s;
    this.repeatT = t;

    this.anisotropy = anisotropy;
    this.wrenTextureIndex = 0;
    this.usedFiltering = 0
    this.image = image;
    this.wrenTexture = undefined;
    this.wrenTextureTransform = undefined;
    this.wrenBackgroundTexture = undefined;
    this.externalTexture = false;
    this.externalTextureRatio = glm.vec2(1.0,1.0);
  }

  modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex) {
    if (!wrenMaterial)
      return;
    this.wrenTextureIndex = mainTextureIndex;
    _wr_material_set_texture(wrenMaterial, this.wrenTexture, this.wrenTextureIndex);
    if (this.wrenTexture) {
      _wr_texture_set_translucent(this.wrenTexture, this.isTransparent);
      _wr_material_set_texture_wrap_s(wrenMaterial, this.repeatS ? ENUM.WR_TEXTURE_WRAP_MODE_REPEAT  : ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, this.wrenTextureIndex);
      _wr_material_set_texture_wrap_t(wrenMaterial, this.repeatT ? ENUM.WR_TEXTURE_WRAP_MODE_REPEAT  : ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, this.wrenTextureIndex);
      _wr_material_set_texture_anisotropy(wrenMaterial, 1 << (this.usedFiltering  - 1), this.wrenTextureIndex);
      _wr_material_set_texture_enable_interpolation(wrenMaterial, this.usedFiltering , this.wrenTextureIndex);
      _wr_material_set_texture_enable_mip_maps(wrenMaterial, this.usedFiltering, this.wrenTextureIndex);

      if (this.externalTexture && ! this.parentNode().textureTransform) {
        _wr_texture_transform_delete(this.wrenTextureTransform);
        this.wrenTextureTransform = _wr_texture_transform_new();
        _wr_texture_transform_set_scale(this.wrenTextureTransform, this.externalTextureRatio.x, this.externalTextureRatio.y);
        _wr_material_set_texture_transform(wrenMaterial, this.wrenTextureTransform);
      }
    }

    _wr_material_set_texture(wrenMaterial, this.wrenBackgroundTexture, backgroundTextureIndex);
    if (typeof this.wrenBackgroundTexture !== 'undefined') {
      // background texture can't be transparent
      _wr_texture_set_translucent(this.wrenBackgroundTexture, false);

      // if there's an opaque background texture, we can't treat the foreground texture as opaque, as we're going to alpha blend
      // them in the shader anyway
      if (typeof this.wrenTexture !== 'undefined')
        _wr_texture_set_translucent(this.wrenTexture, false);

      _wr_material_set_texture_wrap_s(wrenMaterial, this.repeatS ? ENUM.WR_TEXTURE_WRAP_MODE_REPEAT : ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, backgroundTextureIndex);
      _wr_material_set_texture_wrap_t(wrenMaterial, this.repeatT ? ENUM.WR_TEXTURE_WRAP_MODE_REPEAT : ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, backgroundTextureIndex);
      _wr_material_set_texture_enable_interpolation(wrenMaterial, false, backgroundTextureIndex);
      _wr_material_set_texture_enable_mip_maps(wrenMaterial, false, backgroundTextureIndex);
    }
  }

  updateWrenTexture() {
    this.destroyWrenTexture();
    // Only load the image from disk if the texture isn't already in the cache
    let texture = Module.ccall('wr_texture_2d_copy_from_cache', 'number', ['string'], [this.url]);
    if (texture === 0) {
      texture = _wr_texture_2d_new();
      _wr_texture_set_size(texture, this.image.width, this.image.height);
      _wr_texture_set_translucent(texture, this.isTransparent);
      let bitsPointer = arrayXPointer(this.image.bits);
      _wr_texture_2d_set_data(texture, bitsPointer);
      _free(bitsPointer);
      Module.ccall('wr_texture_2d_set_file_path', null, ['number', 'string'], [texture, this.url]);
      _wr_texture_setup(texture);

    } else
      this.isTransparent = _wr_texture_is_translucent(texture);

      this.wrenTexture = texture;
  }

  destroyWrenTexture() {
    if (typeof this.externalTexture != 'undefined')
      _wr_texture_delete(this.wrenTexture);

    _wr_texture_transform_delete(this.wrenTextureTransform);

    this.wrenTexture = undefined;
    this.wrenTextureTransform = undefined;

    //TODO see how to delete js image
    //delete mImage;
    //this.image = undefined;
  }

  preFinalize() {
    super.preFinalize();
    this.updateUrl();
  }

  postFinalize() {
    super.postFinalize();
  }

  updateUrl() {
    // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
    this.url = this.url.replaceAll("\\", "/");

    this.updateWrenTexture();
  }

}

export {WbImageTexture}
