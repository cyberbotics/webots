import {WbBaseNode} from "./WbBaseNode.js"

class WbAbstractAppearance extends WbBaseNode {
  constructor(id, transform){
    super(id);
    this.textureTransform = transform;
  }

  createWrenObjects(){
    super.createWrenObjects();
    if(typeof this.textureTransform !== 'undefined') {
      this.textureTransform.createWrenObjects();
    }
  }

  preFinalize() {
    super.preFinalize();
    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.preFinalize();

    this.updateTextureTransform();
  }

  postFinalize() {
    super.postFinalize();

    if (typeof this.textureTransform !== undefined)
      this.textureTransform.postFinalize();
  }

  updateTextureTransform() {
  }

}

export {WbAbstractAppearance}
