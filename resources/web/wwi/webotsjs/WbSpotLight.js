import {WbLight} from "./WbLight.js"
import {findUpperTransform} from "./WbUtils.js"

class WbSpotLight extends WbLight {
  constructor(id, on, attenuation, beamWidth, color, cutOffAngle, direction, intensity, location, radius, ambientIntensity, castShadows, parent){
    super(id, on, color, intensity, castShadows, ambientIntensity);
    this.attenuation = attenuation;
    this.beamWidth = beamWidth;
    this.cutOffAngle = cutOffAngle;
    this.direction = direction;
    this.location = location;
    this.radius = radius;

    this.wrenlight
    if(typeof parent !== 'undefined')
      this.parentNode = parent.id;
  }

  createWrenObjects() {
    this.wrenLight = _wr_spot_light_new();
    super.createWrenObjects();
    this.attachToUpperTransform();

    this.applyLightDirectionToWren();
    this.applyLightBeamWidthAndCutOffAngleToWren();
    this.applyLightAttenuationToWren();
    this.applyNodeLocationToWren();
  }

  attachToUpperTransform() {
    let upperTransform = findUpperTransform(this);

    if (typeof upperTransform !== 'undefined')
      _wr_transform_attach_child(upperTransform.wrenNode, this.wrenLight);
  }

  applyLightDirectionToWren() {
    let pointer = _wrjs_color_array(this.direction.x, this.direction.y, this.direction.z);
    _wr_spot_light_set_direction(this.wrenLight, pointer);
  }

  applyLightBeamWidthAndCutOffAngleToWren() {
    _wr_spot_light_set_beam_width(this.wrenLight, this.beamWidth);
    _wr_spot_light_set_cutoff_angle(this.wrenLight, this.cutOffAngle);
  }

  applyLightAttenuationToWren() {
    _wr_spot_light_set_radius(this.wrenLight, this.radius);
    _wr_spot_light_set_attenuation(this.wrenLight, this.attenuation.x, this.attenuation.y, this.attenuation.z);
  }

  applyNodeLocationToWren() {
    let pointer = _wrjs_color_array(this.location.x, this.location.y, this.location.z);
    _wr_spot_light_set_position_relative(this.wrenLight, pointer);
  }

  applyLightColorToWren() {
    let pointer = _wrjs_color_array(this.color.x, this.color.y, this.color.z);
    _wr_spot_light_set_color(this.wrenLight, pointer);
  }

  applyLightIntensityToWren() {
    _wr_spot_light_set_intensity(this.wrenLight, this.intensity);
  }

  applyLightVisibilityToWren() {
    _wr_spot_light_set_on(this.wrenLight, this.on);

    const maxCount = undefined;//wr_config_get_max_active_spot_light_count();
    const activeCount = _wr_scene_get_active_spot_light_count(_wr_scene_get_instance());
    if (activeCount == maxCount)
      console.log("Maximum number of active spotlights has been reached, newly added lights won't be rendered.");
  }

  applyLightShadowsToWren() {
    _wr_spot_light_set_cast_shadows(this.wrenLight, this.castShadows);
  }
}

export {WbSpotLight}
