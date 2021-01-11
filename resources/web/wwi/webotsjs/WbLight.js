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

class WbLight extends WbBaseNode {
  constructor(id, on, color, intensity, castShadows, ambientIntensity) {
    super(id);
    this.color = color;

    this.ambientIntensity = ambientIntensity;
    this.intensity = intensity;

    this.on = on;
    this.castShadows = castShadows;
  }

  delete(){
    if (typeof this.parent === 'undefined') {
      World.instance.sceneTree.splice(this, 1);
    } else {
      let parent = World.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        parent.children.splice(this, 1);
      }
    }

    if (this.wrenObjectsCreatedCalled) {
      WbLight.lights.splice(this, 1);
      this.applySceneAmbientColorToWren();
    }

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.applyLightColorToWren();
    this.applyLightIntensityToWren();
    this.applyLightVisibilityToWren();
    this.applyLightShadowsToWren();
    this.applySceneAmbientColorToWren();
  }

  applyLightColorToWren(){}
  applyLightIntensityToWren(){}
  applyLightVisibilityToWren(){}
  applyLightShadowsToWren(){}

  applySceneAmbientColorToWren() {
    this.computeAmbientLight();
  }

  computeAmbientLight() {
    let rgb = glm.vec3(0.0, 0.0, 0.0);

    WbLight.lights.forEach (light => {
      if (light.on) {
          rgb.x += light.ambientIntensity * light.color.x;
          rgb.y += light.ambientIntensity * light.color.y;
          rgb.z += light.ambientIntensity * light.color.z;
      }
    });

    _wr_scene_set_ambient_light(_wrjs_color_array(rgb.x, rgb.y, rgb.z));
  }

  preFinalize() {
    WbLight.lights.push(this);
  }
}

WbLight.lights = [];
export {WbLight}
