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
    let rgb = glm.vec3(0.0, 0.0, 0.0);

    WbLight.lights.forEach (light => {
      if (light.on) {
          rgb.x += light.ambientIntensity * light.color.x;
          rgb.y += light.ambientIntensity * light.color.y;
          rgb.z += light.ambientIntensity * light.color.z;
      }
    });

    _wr_scene_set_ambient_light(_wrjs_color_array(rgb.x, rgb.y, rgb.z));
  }

  preFinalize() {
    WbLight.lights.push(this);
  }
}

WbLight.lights = [];
export {WbLight}
