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
import {World} from "./World.js"

class WbAbstractAppearance extends WbBaseNode {
  constructor(id, transform){
    super(id);
    this.textureTransform = transform;
  }

  delete() {
    if(typeof this.parent !== 'undefined'){
      let parent = World.instance.nodes.get(this.parent);
      if(typeof parent !== 'undefined'){
        parent.appearance = undefined;
        parent.updateAppearance();
      }
    }

    if(typeof this.textureTransform !== 'undefined')
      this.textureTransform.delete;

    super.delete();
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

    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.postFinalize();
  }

  updateTextureTransform() {
  }

}

export {WbAbstractAppearance}
