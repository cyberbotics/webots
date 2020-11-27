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
    //TODO: not needed until we export ambientIntensity
    //this.computeAmbientLight();
  }
}
export {WbLight}
