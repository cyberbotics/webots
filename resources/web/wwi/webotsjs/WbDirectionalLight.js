import {WbLight} from "./WbLight.js"

class WbDirectionalLight extends WbLight {
  constructor(id, on, color, direction, intensity, castShadows, ambientIntensity){
    super(id, on, color, intensity, castShadows, ambientIntensity);
    this.wrenLight = undefined;
    this.direction = direction;
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

    //FIXME : Verify if this check is still needed or if we can get rid of it
    let maxCount = undefined;//_wr_config_get_max_active_directional_light_count();
    let activeCount = _wr_scene_get_active_directional_light_count(_wr_scene_get_instance());
    if (activeCount == maxCount)
      console.log("Maximum number of directional lights " + maxCount +" has been reached, newly added lights won't be rendered.");
  }

  applyLightShadowsToWren() {
    _wr_directional_light_set_cast_shadows(this.wrenLight, this.castShadows);
  }
}
export {WbDirectionalLight}
