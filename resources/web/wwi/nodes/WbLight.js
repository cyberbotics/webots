import WbBaseNode from './WbBaseNode.js';
import WbVector3 from './utils/WbVector3.js';
import WbWorld from './WbWorld.js';
import {resetColorIfInvalid} from './utils/WbFieldChecker.js';

export default class WbLight extends WbBaseNode {
  #color;
  #ambientIntensity;
  #intensity;
  #on;
  #castShadows;
  constructor(id, on, color, intensity, castShadows, ambientIntensity) {
    super(id);
    this.#color = color;

    this.#ambientIntensity = ambientIntensity;
    this.#intensity = intensity;

    this.#on = on;
    this.#castShadows = castShadows;
  }

  get color() {
    return this.#color;
  }

  set color(newColor) {
    this.#color = newColor;

    if (this.wrenObjectsCreatedCalled)
      this.#updateColor();
  }

  get ambientIntensity() {
    return this.#ambientIntensity;
  }

  get intensity() {
    return this.#intensity;
  }

  get on() {
    return this.#on;
  }

  set on(newOn) {
    this.#on = newOn;

    if (this.wrenObjectsCreatedCalled)
      this.#updateOn();
  }

  get castShadows() {
    return this.#castShadows;
  }

  createWrenObjects() {
    super.createWrenObjects();

    this._applyLightColorToWren();
    this._applyLightIntensityToWren();
    this._applyLightVisibilityToWren();
    this._applyLightShadowsToWren();
    this.#applySceneAmbientColorToWren();
  }

  delete() {
    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.sceneTree.indexOf(this);
      WbWorld.instance.sceneTree.splice(index, 1);
    } else {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        if (typeof parent.endPoint !== 'undefined')
          parent.endPoint = undefined;
        else {
          const index = parent.children.indexOf(this);
          parent.children.splice(index, 1);
        }
      }
    }

    if (this.wrenObjectsCreatedCalled) {
      WbLight.lights.splice(this, 1);
      this.#applySceneAmbientColorToWren();
    }

    super.delete();
  }

  preFinalize() {
    super.preFinalize();
    WbLight.lights.push(this);
  }

  #updateColor() {
    const newColor = resetColorIfInvalid(this.#color);
    if (newColor !== false) {
      this.#color = newColor;
      return;
    }

    this._applyLightColorToWren();
  }

  #updateOn() {
    this._applyLightVisibilityToWren();
  }

  // Private functions

  _applyLightColorToWren() {}
  _applyLightIntensityToWren() {}
  _applyLightShadowsToWren() {}
  _applyLightVisibilityToWren() {}

  #applySceneAmbientColorToWren() {
    this.#computeAmbientLight();
  }

  #computeAmbientLight() {
    const rgb = new WbVector3(0.0, 0.0, 0.0);

    WbLight.lights.forEach(light => {
      if (light.#on) {
        rgb.x += light.#ambientIntensity * light.#color.x;
        rgb.y += light.#ambientIntensity * light.#color.y;
        rgb.z += light.#ambientIntensity * light.#color.z;
      }
    });

    _wr_scene_set_ambient_light(_wrjs_array3(rgb.x, rgb.y, rgb.z));
  }
}

WbLight.lights = [];
