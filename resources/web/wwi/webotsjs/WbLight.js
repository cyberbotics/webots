import {WbBaseNode} from "./WbBaseNode.js"

class WbLight extends WbBaseNode {
  constructor(id, on, color, intensity, castShadows, ambientIntensity) {
    super(id);
    this.color = color;

    this.ambientIntensity = ambientIntensity;
    this.intensity = intensity;

    this.on = on;
    this.castShadows = castShadows;
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.applyLightColorToWren();
    this.applyLightIntensityToWren();
    this.applyLightVisibilityToWren();
    this.applyLightShadowsToWren();
    this.applySceneAmbientColorToWren();
  }

  applyLightColorToWren(){}
  applyLightIntensityToWren(){}
  applyLightVisibilityToWren(){}
  applyLightShadowsToWren(){}

  applySceneAmbientColorToWren() {
    this.computeAmbientLight();
  }

  computeAmbientLight() {
    let rgb = glm.vec3(0,0,0);

    //foreach (const WbLight *light, cLights) {
      if (this.on) {
        rgb[0] += this.ambientIntensity * this.color.x;
        rgb[1] += this.ambientIntensity * this.color.y;
        rgb[2] += this.ambientIntensity * this.color.z;
      }
    //}
    let pointer = _wrjs_color_array(rgb.x, rgb.y, rgb.z);
    _wr_scene_set_ambient_light(rgb);
  }
}
export {WbLight}
