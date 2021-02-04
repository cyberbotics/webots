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
import {World} from "./World.js";

class WbFog extends WbBaseNode {
  constructor(id, color, visibilityRange, fogType) {
    super(id);
    this.color = color;
    this.visibilityRange = visibilityRange;
    this.fogType = fogType;

    this.wrenFogType = undefined;
  }

  delete() {
    if (typeof this.parent === 'undefined'){
      let index = World.instance.sceneTree.indexOf(this)
      World.instance.sceneTree.splice(index, 1);
    }

    if (this.wrenObjectsCreatedCalled)
      _wr_scene_set_fog(_wr_scene_get_instance(), ENUM.WR_SCENE_FOG_TYPE_NONE, ENUM.WR_SCENE_FOG_DEPTH_TYPE_PLANE, null, 1.0, 0.0, 1.0);

    World.instance.hasFog = false;
    
    super.delete();
  }

  createWrenObjects(){
    super.createWrenObjects();

    this.applyChangesToWren();
  }

  applyChangesToWren() {
    let density = 0.0;
    if (this.visibilityRange > 0.0)
      density = 1.0 / this.visibilityRange;
    else
      this.wrenFogType = ENUM.WR_SCENE_FOG_TYPE_NONE;

    let colorPointer = _wrjs_color_array(this.color.x, this.color.y, this.color.z);
    _wr_scene_set_fog(_wr_scene_get_instance(), this.wrenFogType, ENUM.WR_SCENE_FOG_DEPTH_TYPE_POINT, colorPointer, density, 0.0, this.visibilityRange);
  }

  preFinalize() {
    super.preFinalize();
    this.updateFogType();
  }

  updateFogType() {
    if (this.fogType === "EXPONENTIAL")
      this.wrenFogType = ENUM.WR_SCENE_FOG_TYPE_EXPONENTIAL;
    else if (this.fogType == "EXPONENTIAL2")
      this.wrenFogType = ENUM.WR_SCENE_FOG_TYPE_EXPONENTIAL2;
    else
      this.wrenFogType = ENUM.WR_SCENE_FOG_TYPE_LINEAR;

    if (this.wrenFogType === ENUM.WR_SCENE_FOG_TYPE_LINEAR && this.fogType !== "LINEAR")
      console.warn("Unknown 'fogType': " + this.fogType + " Set to \"LINEAR\"");

    if (this.wrenObjectsCreatedCalled)
      this.applyChangesToWren();

  }
}

export {WbFog}
