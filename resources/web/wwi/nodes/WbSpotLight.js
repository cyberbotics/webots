import {findUpperTransform} from './utils/utils.js';
import WbLight from './WbLight.js';

export default class WbSpotLight extends WbLight {
  #wrenLight;
  constructor(id, on, attenuation, beamWidth, color, cutOffAngle, direction, intensity, location, radius, ambientIntensity,
    castShadows, parent) {
    super(id, on, color, intensity, castShadows, ambientIntensity);
    this.attenuation = attenuation;
    this.beamWidth = beamWidth;
    this.cutOffAngle = cutOffAngle;
    this.direction = direction;
    this.location = location;
    this.radius = radius;

    if (typeof parent !== 'undefined')
      this.parent = parent.id;
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbSpotLight(customID, this.on, this.attenuation, this.beamWidth, this.color, this.cutOffAngle, this.direction,
      this.intensity, this.location, this.radius, this.ambientIntensity, this.castShadows);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    this.#wrenLight = _wr_spot_light_new();
    super.createWrenObjects();
    this.#attachToUpperTransform();

    this.#applyLightDirectionToWren();
    this.#applyLightBeamWidthAndCutOffAngleToWren();
    this.#applyLightAttenuationToWren();
    this.#applyNodeLocationToWren();
  }

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      this.#detachFromUpperTransform();
      _wr_node_delete(this.#wrenLight);
    }

    super.delete();
  }

  // Private functions
  #applyLightAttenuationToWren() {
    _wr_spot_light_set_radius(this.#wrenLight, this.radius);
    _wr_spot_light_set_attenuation(this.#wrenLight, this.attenuation.x, this.attenuation.y, this.attenuation.z);
  }

  #applyLightBeamWidthAndCutOffAngleToWren() {
    _wr_spot_light_set_beam_width(this.#wrenLight, this.beamWidth);
    _wr_spot_light_set_cutoff_angle(this.#wrenLight, this.cutOffAngle);
  }

  _applyLightColorToWren() {
    const pointer = _wrjs_array3(this.color.x, this.color.y, this.color.z);
    _wr_spot_light_set_color(this.#wrenLight, pointer);
  }

  #applyLightDirectionToWren() {
    const pointer = _wrjs_array3(this.direction.x, this.direction.y, this.direction.z);
    _wr_spot_light_set_direction(this.#wrenLight, pointer);
  }

  _applyLightIntensityToWren() {
    _wr_spot_light_set_intensity(this.#wrenLight, this.intensity);
  }

  _applyLightShadowsToWren() {
    _wr_spot_light_set_cast_shadows(this.#wrenLight, this.castShadows);
  }

  _applyLightVisibilityToWren() {
    _wr_spot_light_set_on(this.#wrenLight, this.on);

    const maxCount = _wr_config_get_max_active_spot_light_count();
    const activeCount = _wr_scene_get_active_spot_light_count(_wr_scene_get_instance());
    if (activeCount === maxCount)
      console.log("Maximum number of active spotlights has been reached, newly added lights won't be rendered.");
  }

  #applyNodeLocationToWren() {
    const pointer = _wrjs_array3(this.location.x, this.location.y, this.location.z);
    _wr_spot_light_set_position_relative(this.#wrenLight, pointer);
  }

  #attachToUpperTransform() {
    const upperTransform = findUpperTransform(this);

    if (typeof upperTransform !== 'undefined')
      _wr_transform_attach_child(upperTransform.wrenNode, this.#wrenLight);
  }

  #detachFromUpperTransform() {
    const node = this.#wrenLight;
    const parent = _wr_node_get_parent(node);
    if (typeof parent !== 'undefined')
      _wr_transform_detach_child(parent, node);
  }
}
