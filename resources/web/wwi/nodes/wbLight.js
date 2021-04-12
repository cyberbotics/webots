import {WbBaseNode} from './wbBaseNode.js';
import {WbVector3} from './utils/wbVector3.js';
import {WbWorld} from './wbWorld.js';

class WbLight extends WbBaseNode {
  constructor(id, on, color, intensity, castShadows, ambientIntensity) {
    super(id);
    this.color = color;

    this.ambientIntensity = ambientIntensity;
    this.intensity = intensity;

    this.on = on;
    this.castShadows = castShadows;
  }

  delete() {
    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.sceneTree.indexOf(this);
      WbWorld.instance.sceneTree.splice(index, 1);
    } else {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        const index = parent.children.indexOf(this);
        parent.children.splice(index, 1);
      }
    }

    if (this.wrenObjectsCreatedCalled) {
      WbLight.lights.splice(this, 1);
      this.applySceneAmbientColorToWren();
    }

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.applyLightColorToWren();
    this.applyLightIntensityToWren();
    this.applyLightVisibilityToWren();
    this.applyLightShadowsToWren();
    this.applySceneAmbientColorToWren();
  }

  applyLightColorToWren() {}
  applyLightIntensityToWren() {}
  applyLightVisibilityToWren() {}
  applyLightShadowsToWren() {}

  applySceneAmbientColorToWren() {
    this.computeAmbientLight();
  }

  computeAmbientLight() {
    const rgb = new WbVector3(0.0, 0.0, 0.0);

    WbLight.lights.forEach(light => {
      if (light.on) {
        rgb.x += light.ambientIntensity * light.color.x;
        rgb.y += light.ambientIntensity * light.color.y;
        rgb.z += light.ambientIntensity * light.color.z;
      }
    });

    _wr_scene_set_ambient_light(_wrjs_array3(rgb.x, rgb.y, rgb.z));
  }

  preFinalize() {
    super.preFinalize();
    WbLight.lights.push(this);
  }
}

WbLight.lights = [];
export {WbLight};
