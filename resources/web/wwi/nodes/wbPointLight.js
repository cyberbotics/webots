import {findUpperTransform} from './utils/wbUtils.js';
import {WbLight} from './wbLight.js';

class WbPointLight extends WbLight {
  constructor(id, on, attenuation, color, intensity, location, radius, ambientIntensity, castShadows, parent) {
    super(id, on, color, intensity, castShadows, ambientIntensity);
    this.attenuation = attenuation;
    this.location = location;
    this.radius = radius;

    if (typeof parent !== 'undefined')
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
    const upperTransform = findUpperTransform(this);

    if (typeof upperTransform !== 'undefined')
      _wr_transform_attach_child(upperTransform.wrenNode, this.wrenLight);
  }

  applyLightAttenuationToWren() {
    _wr_point_light_set_radius(this.wrenLight, this.radius);
    _wr_point_light_set_attenuation(this.wrenLight, this.attenuation.x, this.attenuation.y, this.attenuation.z);
  }

  applyNodeLocationToWren() {
    const position = _wrjs_array3(this.location.x, this.location.y, this.location.z);
    _wr_point_light_set_position_relative(this.wrenLight, position);
  }

  applyLightColorToWren() {
    const pointer = _wrjs_array3(this.color.x, this.color.y, this.color.z);

    _wr_point_light_set_color(this.wrenLight, pointer);
  }

  applyLightIntensityToWren() {
    _wr_point_light_set_intensity(this.wrenLight, this.intensity);
  }

  applyLightVisibilityToWren() {
    _wr_point_light_set_on(this.wrenLight, this.on);

    const maxCount = _wr_config_get_max_active_point_light_count();
    const activeCount = _wr_scene_get_active_point_light_count(_wr_scene_get_instance());
    if (activeCount === maxCount)
      console.log("Maximum number of active point lights has been reached, newly added lights won't be rendered.");
  }

  applyLightShadowsToWren() {
    _wr_point_light_set_cast_shadows(this.wrenLight, this.castShadows);
  }

  detachFromUpperTransform() {
    const node = this.wrenLight;
    const parent = _wr_node_get_parent(node);
    if (typeof parent !== 'undefined')
      _wr_transform_detach_child(parent, node);
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbPointLight(customID, this.on, this.attenuation, this.color, this.intensity, this.location, this.radius, this.ambientIntensity, this.castShadows);
  }
}

export {WbPointLight};
