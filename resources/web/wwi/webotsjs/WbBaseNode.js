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

import {World} from "./World.js"
import {findUpperTransform, nodeIsInBoundingObject} from "./WbUtils.js"

class WbBaseNode {
  constructor(id){
    this.id = id;
    this.parent = undefined;
    this.wrenNode = undefined;

    this.wrenObjectsCreatedCalled = false;
    this.isPreFinalizeCalled = false;
    this.isPostFinalizeCalled = false;

    this.upperTransformFirstTimeSearch = true;
    this.upperTransform = false;

    this.boundingObjectFirstTimeSearch = true;
    this.isInBoundingObject = false;
  }


  delete(){
    //check if it is a DEF node
    let uses = World.instance.defUse[this.id];
    if (typeof uses !== 'undefined') {
      if (uses.size !== 0) {

      }
      delete World.instance.defUse[this.id];
    }

    World.instance.nodes.delete(this.id);
  }

  createWrenObjects() {
    this.wrenObjectsCreatedCalled = true;

    if (typeof this.parent !== 'undefined') {
      this.wrenNode = World.instance.nodes.get(this.parent).wrenNode;
    } else{
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
    }
  }

  upperTransform() {
    if (this.upperTransformFirstTimeSearch) {
      this.upperTransform = findUpperTransform(this);
      if (this.wrenObjectsCreatedCalled)
        this.upperTransformFirstTimeSearch = false;
    }

    return this.upperTransform;
  }

  isInBoundingObject() {
    if (this.boundingObjectFirstTimeSearch) {
      this.isInBoundingObject = nodeIsInBoundingObject(this);
      if (this.wrenObjectsCreatedCalled)
        this.boundingObjectFirstTimeSearch = false;
    }

    return this.isInBoundingObject;
  }


  finalize() {
    if (!this.isPreFinalizeCalled)
      this.preFinalize();

    if (!this.wrenObjectsCreatedCalled)
      this.createWrenObjects();

    if (!this.isPostFinalizeCalled)
      this.postFinalize();

    //emit finalizationCompleted(this);
  }

  preFinalize() {
    this.isPreFinalizeCalled = true;
  }

  postFinalize() {
    this.isPostFinalizeCalled = true;
  }
}

export{WbBaseNode}
