import {WbBaseNode} from "./WbBaseNode.js"

class WbGroup extends WbBaseNode{
  constructor(id){
    super(id);
    this.children = [];
  }

  createWrenObjects(){
    super.createWrenObjects();
  }
}

export{WbGroup}
