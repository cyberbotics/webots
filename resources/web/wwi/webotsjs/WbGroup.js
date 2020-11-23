import {WbBaseNode} from "./WbBaseNode.js"

class WbGroup extends WbBaseNode{
  constructor(id){
    super(id);
    this.children = [];
  }

  createWrenObjects(isTransform){
    super.createWrenObjects();

    if(!isTransform) {
      this.children.forEach(child => {
        child.createWrenObjects()
      });
    }
  }

  preFinalize() {
    super.preFinalize();

    this.children.forEach( child => child.preFinalize());
  }

  postFinalize() {
    super.postFinalize();

    this.children.forEach( child => child.postFinalize());
  }
}

export{WbGroup}
