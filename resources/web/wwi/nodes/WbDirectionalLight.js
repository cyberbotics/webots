import WbLight from './WbLight.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbDirectionalLight extends WbLight {
  #direction;
  #wrenLight;
  constructor(id, on, color, direction, intensity, castShadows, ambientIntensity) {
    super(id, on, color, intensity, castShadows, ambientIntensity);

    this.#direction = direction;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_DIRECTIONAL_LIGHT;
  }

  get direction() {
    return this.#direction;
  }

  set direction(newDirection) {
    this.#direction = newDirection;

    this.#updateDirection();
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    this.#wrenLight = _wr_directional_light_new();
    super.createWrenObjects();

    this.#applyLightDirectionToWren();
  }

  delete() {
    if (this.wrenObjectsCreatedCalled)
      _wr_node_delete(this.#wrenLight);

    super.delete();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbDirectionalLight(customID, this.on, this.color, this.#direction, this.intensity, this.castShadows,
      this.ambientIntensity);
  }

  // Private functions

  _applyLightColorToWren() {
    const pointer = _wrjs_array3(this.color.x, this.color.y, this.color.z);

    _wr_directional_light_set_color(this.#wrenLight, pointer);
  }

  #applyLightDirectionToWren() {
    const pointer = _wrjs_array3(this.#direction.x, this.#direction.y, this.#direction.z);
    _wr_directional_light_set_direction(this.#wrenLight, pointer);
  }

  _applyLightIntensityToWren() {
    _wr_directional_light_set_intensity(this.#wrenLight, this.intensity);
  }

  _applyLightShadowsToWren() {
    _wr_directional_light_set_cast_shadows(this.#wrenLight, this.castShadows);
  }

  _applyLightVisibilityToWren() {
    _wr_directional_light_set_on(this.#wrenLight, this.on);

    const maxCount = _wr_config_get_max_active_directional_light_count();
    const activeCount = _wr_scene_get_active_directional_light_count(_wr_scene_get_instance());
    if (activeCount === maxCount) {
      console.log('Maximum number of directional lights ' + maxCount +
        ' has been reached, newly added lights won\'t be rendered.');
    }
  }

  #updateDirection() {
    if (this.wrenObjectsCreatedCalled)
      this.#applyLightDirectionToWren();
  }
}
