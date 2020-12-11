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
}

export {WbPointLight}
