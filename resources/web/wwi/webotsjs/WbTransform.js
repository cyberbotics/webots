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

import {WbGroup} from "./WbGroup.js"
import {World} from "./World.js";

//Is also used to represent a solid
class WbTransform extends WbGroup {
  constructor(id, isSolid, translation, scale, rotation) {
    super(id);
    this.isSolid = isSolid;
    this.translation = translation;
    this.scale = scale;
    this.rotation = rotation;

    this.children = [];
    this.boundingObject = undefined;
  }

  delete(){
    if (this.wrenObjectsCreatedCalled){
      _wr_node_delete(this.wrenNode);
    }

    if(typeof this.boundingObject !== 'undefined'){
      this.boundingObject.delete()
    }

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects(true);
    let transform = _wr_transform_new();

    _wr_transform_attach_child(this.wrenNode, transform);
    this.wrenNode = transform;

    this.children.forEach(child => {
      child.createWrenObjects()
    });

    if(typeof this.boundingObject !== 'undefined')
      this.boundingObject.createWrenObjects()
    
    this.applyTranslationToWren();
    this.applyRotationToWren();
    this.applyScaleToWren();
  }

  applyTranslationToWren() {
    let translation = _wrjs_color_array(this.translation.x, this.translation.y, this.translation.z);
    _wr_transform_set_position(this.wrenNode, translation);
  }

  applyRotationToWren() {
    let rotation = _wrjs_array4(this.rotation.w, this.rotation.x, this.rotation.y, this.rotation.z);
    _wr_transform_set_orientation(this.wrenNode, rotation);
  }

  applyScaleToWren() {
    let scale = _wrjs_color_array(this.scale.x, this.scale.y, this.scale.z);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  updateBoundingObjectVisibility() {
    super.updateBoundingObjectVisibility();

    if(typeof this.boundingObject !== 'undefined')
      this.boundingObject.updateBoundingObjectVisibility();
  }

  preFinalize() {
    super.preFinalize();

    if(typeof this.boundingObject !== 'undefined')
      this.boundingObject.preFinalize()
  }

  postFinalize() {
    super.postFinalize();

    if(typeof this.boundingObject !== 'undefined')
      this.boundingObject.postFinalize()
  }
}

export {WbTransform}
