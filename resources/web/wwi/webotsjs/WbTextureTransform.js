import {WbBaseNode} from "./WbBaseNode.js"

class WbTextureTransform extends WbBaseNode{
  constructor(id, center, rotation, scale, translation){
    super(id);
    this.center = center;
    this.rotation = rotation;
    this.scale = scale;
    this.translation = translation;
    this.wrenTextureTransform;
  }

  modifyWrenMaterial(wrenMaterial) {
    this.destroyWrenObjects();

    // apply translation before rotation
    this.wrenTextureTransform = _wr_texture_transform_new();
    _wr_texture_transform_set_scale(this.wrenTextureTransform, this.scale.x, this.scale.y);
    _wr_texture_transform_set_position(this.wrenTextureTransform, this.translation.x, this.translation.y);
    _wr_texture_transform_set_center(this.wrenTextureTransform, this.center.x, this.center.y);
    _wr_texture_transform_set_rotation(this.wrenTextureTransform, this.rotation);

    _wr_material_set_texture_transform(wrenMaterial, this.wrenTextureTransform);
  }

  destroyWrenObjects() {
    if (this.wrenTextureTransform)
      _wr_texture_transform_delete(this.wrenTextureTransform);
  }

  preFinalize() {
    super.preFinalize();
  }

  postFinalize() {
    super.postFinalize();
  }
}

export {WbTextureTransform}
