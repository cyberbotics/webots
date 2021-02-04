// Copyright 1996-2021 Cyberbotics Ltd.
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

class WbTextureTransform extends WbBaseNode{
  constructor(id, center, rotation, scale, translation){
    super(id);
    this.center = center;
    this.rotation = rotation;
    this.scale = scale;
    this.translation = translation;
    this.wrenTextureTransform;
  }

  delete(){
    if(typeof this.parent !== 'undefined'){
      let parent = World.instance.nodes.get(this.parent);
      if(typeof parent !== 'undefined')
        parent.textureTransform = undefined;
    }

    this.destroyWrenObjects();

    super.delete();
  }

  modifyWrenMaterial(wrenMaterial) {
    this.destroyWrenObjects();

    // apply translation before rotation
    this.wrenTextureTransform = _wr_texture_transform_new();
    _wr_texture_transform_set_scale(this.wrenTextureTransform, this.scale.x, this.scale.y);
    _wr_texture_transform_set_position(this.wrenTextureTransform, this.translation.x, this.translation.y);
    _wr_texture_transform_set_center(this.wrenTextureTransform, this.center.x, this.center.y);
    _wr_texture_transform_set_rotation(this.wrenTextureTransform, this.rotation);

    _wr_material_set_texture_transform(wrenMaterial, this.wrenTextureTransform);
  }

  destroyWrenObjects() {
    if (this.wrenTextureTransform)
      _wr_texture_transform_delete(this.wrenTextureTransform);
  }

  preFinalize() {
    super.preFinalize();
  }

  postFinalize() {
    super.postFinalize();
  }
}

export {WbTextureTransform}
