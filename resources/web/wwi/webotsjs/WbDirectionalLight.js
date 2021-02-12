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

import {WbLight} from "./WbLight.js"

class WbDirectionalLight extends WbLight {
  constructor(id, on, color, direction, intensity, castShadows, ambientIntensity){
    super(id, on, color, intensity, castShadows, ambientIntensity);
    this.wrenLight = undefined;
    this.direction = direction;
  }

  delete(){
    if (this.wrenObjectsCreatedCalled)
      _wr_node_delete(this.wrenLight);

    super.delete();
  }

  createWrenObjects() {
    this.wrenLight = _wr_directional_light_new();
    super.createWrenObjects();

    this.applyLightDirectionToWren();
  }

  applyLightDirectionToWren() {
    let pointer = _wrjs_color_array(this.direction.x, this.direction.y, this.direction.z);
    _wr_directional_light_set_direction(this.wrenLight, pointer);
  }

  applyLightColorToWren() {
    let pointer = _wrjs_color_array(this.color.x, this.color.y, this.color.z);

    _wr_directional_light_set_color(this.wrenLight, pointer);
  }

  applyLightIntensityToWren() {
    _wr_directional_light_set_intensity(this.wrenLight, this.intensity);
  }

  applyLightVisibilityToWren() {
    _wr_directional_light_set_on(this.wrenLight, this.on);

    let maxCount = _wr_config_get_max_active_directional_light_count();
    let activeCount = _wr_scene_get_active_directional_light_count(_wr_scene_get_instance());
    if (activeCount === maxCount)
      console.log("Maximum number of directional lights " + maxCount +" has been reached, newly added lights won't be rendered.");
  }

  applyLightShadowsToWren() {
    _wr_directional_light_set_cast_shadows(this.wrenLight, this.castShadows);
  }

  clone(customID) {
    return new WbDirectionalLight(customID, this.on, this.color, this.direction, this.intensity, this.castShadows, this.ambientIntensity);
  }
}
export {WbDirectionalLight}
