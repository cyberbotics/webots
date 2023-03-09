import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';

export default class WbTextureTransform extends WbBaseNode {
  #wrenTextureTransform;
  constructor(id, center, rotation, scale, translation) {
    super(id);
    this.center = center;
    this.rotation = rotation;
    this.scale = scale;
    this.translation = translation;
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbTextureTransform(customID, this.center, this.rotation, this.scale, this.translation);
  }

  delete() {
    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined')
        parent.textureTransform = undefined;
    }

    this.#destroyWrenObjects();

    super.delete();
  }

  modifyWrenMaterial(wrenMaterial) {
    this.#destroyWrenObjects();

    // apply translation before rotation
    this.#wrenTextureTransform = _wr_texture_transform_new();
    _wr_texture_transform_set_scale(this.#wrenTextureTransform, this.scale.x, this.scale.y);
    _wr_texture_transform_set_position(this.#wrenTextureTransform, this.translation.x, this.translation.y);
    _wr_texture_transform_set_center(this.#wrenTextureTransform, this.center.x, this.center.y);
    _wr_texture_transform_set_rotation(this.#wrenTextureTransform, this.rotation);

    _wr_material_set_texture_transform(wrenMaterial, this.#wrenTextureTransform);
  }

  // Private functions

  #destroyWrenObjects() {
    if (typeof this.#wrenTextureTransform !== 'undefined')
      _wr_texture_transform_delete(this.#wrenTextureTransform);
  }
}
