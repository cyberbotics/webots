import WbAppearance from './WbAppearance.js';
import WbDevice from './WbDevice.js';
import WbGroup from './WbGroup.js';
import WbLight from './WbLight.js';
import WbShape from './WbShape.js';
import WbVector3 from './utils/WbVector3.js';
import {clampValuesIfNeeded} from './utils/WbFieldChecker.js';
import {WbNodeType} from './wb_node_type.js';

// This class is used to retrieve the type of device
export default class WbLed extends WbDevice {
  #color;
  #lights;
  #materials;
  #pbrAppearances;
  #value;
  constructor(id, translation, rotation, name, color) {
    super(id, translation, rotation, name);
    this.#lights = [];
    this.#materials = [];
    this.#pbrAppearances = [];
    this.#color = color;
    this.#value = 0;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_LED;
  }

  get value() {
    return this.#value;
  }

  set value(newValue) {
    if (this.#value === newValue)
      return;
    this.#value = newValue;

    this.#setMaterialsAndLightsColor();
  }

  postFinalize() {
    super.postFinalize();

    this._updateChildren();

    // disable WREN lights if not on
    const on = this.#value !== 0;
    for (const light of this.#lights)
      light.on = on;
  }

  applyOptionalRendering(enable) {
    this.value = enable ? 1 : 0;
  }

  _updateChildren() {
    if (!this.isPostFinalizedCalled)
      return;

    this.#findMaterialsAndLights(this);

    // update color of lights and materials
    this.#setMaterialsAndLightsColor();
  }

  #clearMaterialsAndLights() {
    this.#materials = [];
    this.#lights = [];
    this.#pbrAppearances = [];
  }

  #findMaterialsAndLights(group) {
    let size = group.children.length;
    if (size < 1)
      return;

    if (group === this) {
      this.#clearMaterialsAndLights();
      size = 1; // we look only into the first child of the WbLed node
    }

    for (const child of group.children) {
      if (child instanceof WbShape && typeof child.appearance !== 'undefined') {
        const appearance = child.appearance;
        if (appearance instanceof WbAppearance) {
          const material = appearance.material;
          if (typeof material !== 'undefined')
            this.#materials.push(material);

          appearance.notifyLed = () => this._updateChildren();
          child.notifyLed = () => this._updateChildren();
        } else if (typeof appearance !== 'undefined') {
          this.#pbrAppearances.push(appearance);
          child.notifyLed = () => this._updateChildren();
        }
      } else if (child instanceof WbLight)
        this.#lights.push(child);
      else if (child instanceof WbGroup) {
        this.#findMaterialsAndLights(child);
        const proxy = new Proxy(child.children, {
          deleteProperty: (target, property) => {
            delete target[property];
            this._updateChildren();
            return true;
          },
          set: (target, property, value, receiver) => {
            target[property] = value;
            this._updateChildren();
            return true;
          }
        });
        child.children = proxy;
      }
    }

    if (group === this && !this.#isAnyMaterialOrLightFound())
      console.warn(`No PBRAppearance, Material and no Light found.
          The first child of a LED should be either a Shape, a Light or a Group containing Shape and Light nodes.`);
  }

  #isAnyMaterialOrLightFound() {
    return (this.#materials.length > 0 || this.#lights.length > 0 || this.#pbrAppearances.length > 0);
  }

  #setMaterialsAndLightsColor() {
    // compute the new color
    let r, g, b;
    if (this.#value > 0) {
      if (this.#value - 1 < this.#color.length) {
        const c = this.#color[this.value - 1];
        r = c.x;
        g = c.y;
        b = c.z;
      } else {
        r = 1;
        g = 1;
        b = 1;
      }
    } else {
      r = 0;
      g = 0;
      b = 0;
    }

    let color = new WbVector3(r, g, b);
    color = clampValuesIfNeeded(color);

    // update every material
    for (const material of this.#materials)
      material.emissiveColor = color;

    // same for PbrAppearances
    for (const pbrAppearance of this.#pbrAppearances)
      pbrAppearance.emissiveColor = color;

    // update every lights
    const on = this.#value !== 0;
    for (const light of this.#lights) {
      light.color = color;
      // disable WREN lights if not on
      light.on = on;
    }
  }
}
