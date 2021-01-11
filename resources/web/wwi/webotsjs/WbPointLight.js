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

import {WbLight} from "./WbLight.js"
import {findUpperTransform} from "./WbUtils.js"

class WbPointLight extends WbLight {
  constructor(id, on, attenuation, color, intensity, location, radius, ambientIntensity, castShadows, parent) {
    super(id, on, color, intensity, castShadows, ambientIntensity);
    this.attenuation = attenuation;
    this.location = location;
    this.radius = radius;

    this.wrenLight;
    if(typeof parent !== 'undefined')
      this.parent = parent.id;
  }

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      this.detachFromUpperTransform();
      _wr_node_delete(this.wrenLight);
    }

    super.delete();
  }

  createWrenObjects() {
    this.wrenLight = _wr_point_light_new();
    this.attachToUpperTransform();
    super.createWrenObjects();

    this.applyLightAttenuationToWren();
    this.applyNodeLocationToWren();
  }

  attachToUpperTransform() {
    let upperTransform = findUpperTransform(this);

    if (typeof upperTransform !== 'undefined'){
      _wr_transform_attach_child(upperTransform.wrenNode, this.wrenLight);
    }
  }

  applyLightAttenuationToWren() {
    _wr_point_light_set_radius(this.wrenLight, this.radius);
    _wr_point_light_set_attenuation(this.wrenLight, this.attenuation.x, this.attenuation.y, this.attenuation.z);
  }

  applyNodeLocationToWren() {
    let position = _wrjs_color_array(this.location.x, this.location.y, this.location.z);
    _wr_point_light_set_position_relative(this.wrenLight, position);
  }

  applyLightColorToWren() {
    let pointer = _wrjs_color_array(this.color.x, this.color.y, this.color.z);

    _wr_point_light_set_color(this.wrenLight, pointer);
  }

  applyLightIntensityToWren() {
    _wr_point_light_set_intensity(this.wrenLight, this.intensity);
  }

  applyLightVisibilityToWren() {
    _wr_point_light_set_on(this.wrenLight, this.on);

    //FIXME : Verify if this check is still needed or if we can get rid of it
    const maxCount = undefined;//wr_config_get_max_active_point_light_count();
    const activeCount = _wr_scene_get_active_point_light_count(_wr_scene_get_instance());
    if (activeCount == maxCount)
      console.log("Maximum number of active point lights has been reached, newly added lights won't be rendered.");
  }

  applyLightShadowsToWren() {
    _wr_point_light_set_cast_shadows(this.wrenLight, this.castShadows);
  }

  detachFromUpperTransform() {
    let node = this.wrenLight;
    let parent = _wr_node_get_parent(node);
    if (typeof parent !== 'undefined')
      _wr_transform_detach_child(parent, node);
  }
}

export {WbPointLight}
