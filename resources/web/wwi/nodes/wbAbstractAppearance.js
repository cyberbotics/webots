import {WbBaseNode} from './wbBaseNode.js';
import {WbWorld} from './wbWorld.js';

class WbAbstractAppearance extends WbBaseNode {
  constructor(id, transform) {
    super(id);
    this.textureTransform = transform;
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

    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.delete();

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();
    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.createWrenObjects();
  }

  preFinalize() {
    super.preFinalize();
    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.postFinalize();
  }
}

export {WbAbstractAppearance};
