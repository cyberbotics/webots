import {findUpperTransform} from './utils/wbUtils.js';
import {WbLight} from './wbLight.js';

class WbSpotLight extends WbLight {
  constructor(id, on, attenuation, beamWidth, color, cutOffAngle, direction, intensity, location, radius, ambientIntensity, castShadows, parent) {
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

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      this._detachFromUpperTransform();
      _wr_node_delete(this.wrenLight);
    }

    super.delete();
  }

  createWrenObjects() {
    this.wrenLight = _wr_spot_light_new();
    super.createWrenObjects();
    this._attachToUpperTransform();

    this._applyLightDirectionToWren();
    this.applyLightBeamWidthAndCutOffAngleToWren();
    this._applyLightAttenuationToWren();
    this._applyNodeLocationToWren();
  }

  _attachToUpperTransform() {
    const upperTransform = findUpperTransform(this);

    if (typeof upperTransform !== 'undefined')
      _wr_transform_attach_child(upperTransform.wrenNode, this.wrenLight);
  }

  _applyLightDirectionToWren() {
    const pointer = _wrjs_array3(this.direction.x, this.direction.y, this.direction.z);
    _wr_spot_light_set_direction(this.wrenLight, pointer);
  }

  applyLightBeamWidthAndCutOffAngleToWren() {
    _wr_spot_light_set_beam_width(this.wrenLight, this.beamWidth);
    _wr_spot_light_set_cutoff_angle(this.wrenLight, this.cutOffAngle);
  }

  _applyLightAttenuationToWren() {
    _wr_spot_light_set_radius(this.wrenLight, this.radius);
    _wr_spot_light_set_attenuation(this.wrenLight, this.attenuation.x, this.attenuation.y, this.attenuation.z);
  }

  _applyNodeLocationToWren() {
    const pointer = _wrjs_array3(this.location.x, this.location.y, this.location.z);
    _wr_spot_light_set_position_relative(this.wrenLight, pointer);
  }

  _applyLightColorToWren() {
    const pointer = _wrjs_array3(this.color.x, this.color.y, this.color.z);
    _wr_spot_light_set_color(this.wrenLight, pointer);
  }

  _applyLightIntensityToWren() {
    _wr_spot_light_set_intensity(this.wrenLight, this.intensity);
  }

  _applyLightVisibilityToWren() {
    _wr_spot_light_set_on(this.wrenLight, this.on);

    const maxCount = _wr_config_get_max_active_spot_light_count();
    const activeCount = _wr_scene_get_active_spot_light_count(_wr_scene_get_instance());
    if (activeCount === maxCount)
      console.log("Maximum number of active spotlights has been reached, newly added lights won't be rendered.");
  }

  _applyLightShadowsToWren() {
    _wr_spot_light_set_cast_shadows(this.wrenLight, this.castShadows);
  }

  _detachFromUpperTransform() {
    const node = this.wrenLight;
    const parent = _wr_node_get_parent(node);
    if (typeof parent !== 'undefined')
      _wr_transform_detach_child(parent, node);
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbSpotLight(customID, this.on, this.attenuation, this.beamWidth, this.color, this.cutOffAngle, this.direction, this.intensity, this.location, this.radius, this.ambientIntensity, this.castShadows);
  }
}

export {WbSpotLight};
