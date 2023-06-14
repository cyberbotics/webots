import { resetIfNegative, resetVector3IfNegative } from './utils/WbFieldChecker.js';
import { findUpperPose } from './utils/node_utilities.js';
import WbVector3 from './utils/WbVector3.js';
import { WbNodeType } from './wb_node_type.js';

import WbLight from './WbLight.js';

export default class WbPointLight extends WbLight {
  #attenuation;
  #location;
  #radius;
  #wrenLight;
  constructor(id, on, attenuation, color, intensity, location, radius, ambientIntensity, castShadows, parent) {
    super(id, on, color, intensity, castShadows, ambientIntensity);
    this.#attenuation = attenuation;
    this.#location = location;
    this.#radius = radius;

    if (typeof parent !== 'undefined')
      this.parent = parent.id;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_POINT_LIGHT;
  }

  get attenuation() {
    return this.#attenuation;
  }

  set attenuation(newAttenuation) {
    this.#attenuation = newAttenuation;

    this.#updateAttenuation();
  }

  get location() {
    return this.#location;
  }

  set location(newLocation) {
    this.#location = newLocation;

    this.#updateLocation();
  }

  get radius() {
    return this.#radius;
  }

  set radius(newRadius) {
    this.#radius = newRadius;

    this.#updateRadius();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbPointLight(customID, this.on, this.#attenuation, this.color, this.intensity, this.#location, this.#radius,
      this.ambientIntensity, this.castShadows);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    this.#wrenLight = _wr_point_light_new();
    this.#attachToUpperPose();
    super.createWrenObjects();

    this.#applyLightAttenuationToWren();
    this.#applyNodeLocationToWren();
  }

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      this.#detachFromUpperPose();
      _wr_node_delete(this.#wrenLight);
    }

    super.delete();
  }

  // Private functions

  #attachToUpperPose() {
    const upperPose = findUpperPose(this);

    if (typeof upperPose !== 'undefined')
      _wr_transform_attach_child(upperPose.wrenNode, this.#wrenLight);
  }

  #applyLightAttenuationToWren() {
    _wr_point_light_set_radius(this.#wrenLight, this.#radius);
    _wr_point_light_set_attenuation(this.#wrenLight, this.#attenuation.x, this.#attenuation.y, this.#attenuation.z);
  }

  _applyLightColorToWren() {
    const pointer = _wrjs_array3(this.color.x, this.color.y, this.color.z);

    _wr_point_light_set_color(this.#wrenLight, pointer);
  }

  _applyLightIntensityToWren() {
    _wr_point_light_set_intensity(this.#wrenLight, this.intensity);
  }

  _applyLightShadowsToWren() {
    _wr_point_light_set_cast_shadows(this.#wrenLight, this.castShadows);
  }

  _applyLightVisibilityToWren() {
    _wr_point_light_set_on(this.#wrenLight, this.on);

    const maxCount = _wr_config_get_max_active_point_light_count();
    const activeCount = _wr_scene_get_active_point_light_count(_wr_scene_get_instance());
    if (activeCount === maxCount)
      console.log("Maximum number of active point lights has been reached, newly added lights won't be rendered.");
  }

  #applyNodeLocationToWren() {
    const position = _wrjs_array3(this.#location.x, this.#location.y, this.#location.z);
    _wr_point_light_set_position_relative(this.#wrenLight, position);
  }

  #detachFromUpperPose() {
    const node = this.#wrenLight;
    const parent = _wr_node_get_parent(node);
    if (typeof parent !== 'undefined')
      _wr_transform_detach_child(parent, node);
  }

  #checkAmbientAndAttenuationExclusivity() {
    if (!this.#attenuation.equal(new WbVector3(1.0, 0.0, 0.0)) && this.ambientIntensity !== 0) {
      console.warn("'ambientIntensity' and 'attenuation' cannot differ from their default values at the same time. 'ambientIntensity' was changed to 0.");
      this.ambientIntensity = 0;
    }
  }

  #updateAttenuation() {
    const newAttenuation = resetVector3IfNegative(this.#attenuation, new WbVector3());
    if (newAttenuation !== false) {
      this.attenuation = newAttenuation;
      return;
    }

    if (this.#attenuation.x > 0.0 || this.attenuation.y > 0.0)
      console.warn("A quadratic 'attenuation' should be preferred to have a realistic simulation of light. Only the third component of the 'attenuation' field should be greater than 0.");

    this.#checkAmbientAndAttenuationExclusivity();

    if (this.wrenObjectsCreatedCalled)
      this.#applyLightAttenuationToWren();
  }

  #updateLocation() {
    if (this.wrenObjectsCreatedCalled)
      this.#applyNodeLocationToWren();
  }

  #updateRadius() {
    const newRadius = resetIfNegative(this.#radius, 0);
    if (newRadius !== false) {
      this.radius = newRadius;
      return;
    }

    if (this.wrenObjectsCreatedCalled)
      this.#applyLightAttenuationToWren();
  }
}
