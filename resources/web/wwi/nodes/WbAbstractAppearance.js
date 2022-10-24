import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';

export default class WbAbstractAppearance extends WbBaseNode {
  #textureTransform;
  constructor(id, transform) {
    super(id);
    this.textureTransform = transform;
  }

  get textureTransform() {
    return this.#textureTransform;
  }
  set textureTransform(newTextureTransform) {
    this.#textureTransform = newTextureTransform;

    if (typeof this.#textureTransform !== 'undefined')
      this.#textureTransform.onChange = () => this.#update();

    this.#update();
  }

  createWrenObjects() {
    super.createWrenObjects();
    if (typeof this.#textureTransform !== 'undefined')
      this.#textureTransform.createWrenObjects();
  }

  delete() {
    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        parent.appearance = undefined;
        parent.wrenMaterial = undefined;
        parent.updateAppearance();
      }
    }

    if (typeof this.#textureTransform !== 'undefined')
      this.#textureTransform.delete();

    super.delete();
  }

  preFinalize() {
    super.preFinalize();
    if (typeof this.#textureTransform !== 'undefined')
      this.#textureTransform.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    if (typeof this.#textureTransform !== 'undefined')
      this.#textureTransform.postFinalize();
  }

  #update() {
    if (this.isPostFinalizedCalled && typeof this.onChange === 'function')
      this.onChange();
  }
}
