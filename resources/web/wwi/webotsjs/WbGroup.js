// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import {WbBaseNode} from "./WbBaseNode.js"
import {World} from "./World.js";

class WbGroup extends WbBaseNode{
  constructor(id,isPropeller){
    super(id);
    this.children = [];

    this.isPropeller = isPropeller;
    this.currentHelix = -1; //to switch between fast and slow helix
  }

  delete() {
    if (typeof this.parent === 'undefined'){
      let index = World.instance.sceneTree.indexOf(this)
      World.instance.sceneTree.splice(index, 1);
    } else {
      let parent = World.instance.nodes.get(this.parent);
      if(typeof parent !== 'undefined') {
        let index = parent.children.indexOf(this)
        parent.children.splice(index, 1);
      }
    }

    let index = this.children.length - 1;
    while(index >= 0) {
      this.children[index].delete();
      --index;
    }

    super.delete();
  }

  createWrenObjects(isTransform){
    super.createWrenObjects();

    if(!isTransform) {
      this.children.forEach(child => {
        child.createWrenObjects()
      });
    }
  }

  updateBoundingObjectVisibility() {
    this.children.forEach(child => {
      child.updateBoundingObjectVisibility();
    });
  }

  preFinalize() {
    super.preFinalize();

    this.children.forEach( child => child.preFinalize());
  }

  postFinalize() {
    super.postFinalize();

    this.children.forEach( child => child.postFinalize());

    if (this.isPropeller === true) {
      if (typeof this.children[1] !== 'undefined')
        this.currentHelix = this.children[1].id;
      else if (typeof this.children[0] !== 'undefined')
        this.currentHelix = this.children[0].id;
      this.switchHelix(this.currentHelix, true);
    }
  }

  switchHelix(id, force) {
    if (id !== this.currentHelix || force) {
      this.currentHelix = id;
      this.children.forEach(child => {
        if(child.id === this.currentHelix)
          _wr_node_set_visible(child.wrenNode, true);
        else
          _wr_node_set_visible(child.wrenNode, false);
      });

    }
  }
}

export{WbGroup}
