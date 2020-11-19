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

}

export {WbAbstractAppearance}
