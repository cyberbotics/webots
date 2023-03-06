import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbTextureTransform extends WbBaseNode {
  #center;
  #rotation;
  #scale;
  #translation;
  #wrenTextureTransform;
  constructor(id, center, rotation, scale, translation) {
    super(id);
    this.#center = center;
    this.#rotation = rotation;
    this.#scale = scale;
    this.#translation = translation;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_TEXTURE_TRANSFORM;
  }

  get center() {
    return this.#center;
  }

  set center(newCenter) {
    this.#center = newCenter;

    this.#update();
  }

  get rotation() {
    return this.#rotation;
  }

  set rotation(newRotation) {
    this.#rotation = newRotation;

    this.#update();
  }

  get scale() {
    return this.#scale;
  }

  set scale(newScale) {
    this.#scale = newScale;

    this.#update();
  }

  get translation() {
    return this.#translation;
  }

  set translation(newTranslation) {
    this.#translation = newTranslation;

    this.#update();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbTextureTransform(customID, this.#center, this.#rotation, this.#scale, this.#translation);
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
    _wr_texture_transform_set_scale(this.#wrenTextureTransform, this.#scale.x, this.#scale.y);
    _wr_texture_transform_set_position(this.#wrenTextureTransform, this.#translation.x, this.#translation.y);
    _wr_texture_transform_set_center(this.#wrenTextureTransform, this.#center.x, this.#center.y);
    _wr_texture_transform_set_rotation(this.#wrenTextureTransform, this.#rotation);

    _wr_material_set_texture_transform(wrenMaterial, this.#wrenTextureTransform);
  }

  // Private functions

  #destroyWrenObjects() {
    if (typeof this.#wrenTextureTransform !== 'undefined')
      _wr_texture_transform_delete(this.#wrenTextureTransform);
  }

  #update() {
    if (this.isPostFinalizedCalled && typeof this.onChange === 'function')
      this.onChange();
  }
}
