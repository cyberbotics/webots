
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

import {WbBaseNode} from "./WbBaseNode.js";
import {WbShape} from "./WbShape.js";
import {WbWrenShaders} from "./WbWrenShaders.js"
import {WbAbstractAppearance} from "./WbAbstractAppearance.js"
import {WbTransform} from "./WbTransform.js"


class Use extends WbBaseNode {
  constructor(id, def, parent){
    super(id);
    this.def = def;

    this.parent = parent;
    this.wrenObjectsCreatedCalled = false

    this.wrenRenderable;
    this.wrenTextureTransform;
    this.wrenMaterial = [];
    this.wrenMesh;
  }

  //TODO Cleanup, test the type of the def node to know what is necessary to replace
  createWrenObjects() {
    super.createWrenObjects();
    let temp = this.def.parent;
    this.def.parent = this.parent;

    let temp2 = this.def.wrenRenderable;
    this.def.wrenRenderable = undefined;

    let temp3 = this.replaceWrenMaterial(this.def);

    let temp4 = this.def.wrenMesh;
    this.def.wrenMesh = undefined;

    this.def.createWrenObjects();

    this.def.parent = temp;

    this.wrenRenderable = this.def.wrenRenderable;
    this.def.wrenRenderable = temp2;

    //TODO replace back wrenmaterial also in recursion. But check that it is usefull to replace back.
    if (this.def instanceof WbTransform && typeof this.def.children !== 'undefined') {
      for (let i = 0; i < this.def.children.length; i++){
        this.wrenMaterial[i] = this.def.children[i].wrenMaterial;
        this.def.children[i].wrenMaterial = temp3.push(this.def.children[i].wrenMaterial);
      }
    } else {
      temp3 = this.def.wrenMaterial;
      this.def.wrenMaterial = undefined;
    }

    this.wrenMesh = this.def.wrenMesh;
    this.def.wrenMesh = temp4;

    this.wrenObjectsCreatedCalled = true;
  }


  modifyWrenMaterial1(wrenMaterial) {
    wrenMaterial = this.def.modifyWrenMaterial(wrenMaterial);
    return wrenMaterial;
  }

  modifyWrenMaterial2(wrenMaterial, textured) {
    return this.def.modifyWrenMaterial(wrenMaterial, textured);
  }

  modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex) {
    let temp
    if(typeof this.def.textureTransform !== 'undefined') {
      temp = this.def.textureTransform.wrenTextureTransform;
      this.def.textureTransform.wrenTextureTransform = undefined;
    } else {
      temp = this.def.wrenTextureTransform;
      this.def.wrenTextureTransform = undefined;
    }

    if (typeof backgroundTextureIndex === 'undefined') {
      if (typeof mainTextureIndex === 'undefined')
        wrenMaterial = this.modifyWrenMaterial1(wrenMaterial);
      else
        wrenMaterial = this.modifyWrenMaterial2(wrenMaterial, mainTextureIndex);
    }else
      wrenMaterial = this.def.modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex);

    if(typeof this.def.textureTransform !== 'undefined') {
      this.wrenTextureTransform = this.def.textureTransform.wrenTextureTransform;
      this.def.textureTransform.wrenTextureTransform = temp;
    } else {
      this.wrenTextureTransform = this.def.wrenTextureTransform;
      this.def.wrenTextureTransform = temp;
    }


    return wrenMaterial;
  }

  setWrenMaterial(wrenMaterial, castShadow) {
    let temp2 = this.def.wrenRenderable;
    this.def.wrenRenderable = this.wrenRenderable;

    this.def.setWrenMaterial(wrenMaterial, castShadow);

    this.wrenRenderable = this.def.wrenRenderable;
    this.def.wrenRenderable = temp2;
  }

  preFinalize(){
    this.isPreFinalizeCalled = true;
  }

  postFinalize(){
    this.isPreFinalizeCalled = false;
  }

  computeCastShadows(castShadows){
    this.def.computeCastShadows(castShadows);
  }

  replaceWrenMaterial(node){
    let temp = [];
    if (node instanceof WbTransform && typeof node.children !== 'undefined') {
      for (let i = 0; i < node.children.length; i++){
        if (node.children[i] instanceof WbTransform && typeof node.children !== 'undefined')
          this.replaceWrenMaterial(node.children[i]);
        temp.push(node.children[i].wrenMaterial);
        node.children[i].wrenMaterial = undefined;
      }
    } else {
      temp = node.wrenMaterial;
      node.wrenMaterial = undefined;
    }

    return temp;
  }

  setPickable(pickable){
    let temp = this.def.id;
    let temp2 = this.def.wrenRenderable;
    this.def.id = this.id;
    this.def.wrenRenderable = this.wrenRenderable;

    this.def.setPickable(pickable);

    this.def.id = temp;
    this.wrenRenderable = this.def.wrenRenderable;
    this.def.wrenRenderable = temp2;
  }

  updateBoundingObjectVisibility() {
    this.def.updateBoundingObjectVisibility();
  }
}

export{Use}
