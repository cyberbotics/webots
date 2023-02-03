import WbAppearance from './WbAppearance.js';
import WbDevice from './WbDevice.js';
import WbGroup from './WbGroup.js';
import WbLight from './WbLight.js';
import WbShape from './WbShape.js';

// This class is used to retrieve the type of device
export default class WbLed extends WbDevice {
  #lights;
  #materials;
  #pbrAppearances;
  constructor(id, translation, scale, rotation, name) {
    super(id, translation, scale, rotation, name);
    this.#lights = [];
    this.#materials = [];
    this.#pbrAppearances = [];
  }

  postFinalize() {
    super.postFinalize();

    this._updateChildren();

    // disable WREN lights if not on
    // bool on = mValue != 0;
    // foreach (WbLight *light, mLights)
    //   light->toggleOn(on);
  }

  _updateChildren() {
    if (!this.isPostFinalizedCalled)
      return;

    this.#findMaterialsAndLights(this);

    // update color of lights and materials
    // this.setMaterialsAndLightsColor();
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
          connect(appearance, &WbAppearance::fieldChanged, this, &WbLed::updateIfNeeded, Qt::UniqueConnection);
          connect(appearance->parentNode(), &WbShape::fieldChanged, this, &WbLed::updateIfNeeded, Qt::UniqueConnection);
        } else {
          mPbrAppearances.append(appearance);
          connect(appearance, &WbPbrAppearance::fieldChanged, this, &WbLed::updateIfNeeded, Qt::UniqueConnection);
          connect(appearance->parentNode(), &WbShape::fieldChanged, this, &WbLed::updateIfNeeded, Qt::UniqueConnection);
        }
      } else if (child instanceof WbLight)
        this.#lights.push(child);
      else if (child instanceof WbGroup) {
        this.#findMaterialsAndLights(child);
        const proxy = new Proxy(child.children, {
          deleteProperty: (target, property) => {
            delete target[property];
            this.updateChildren();
            return true;
          },
          set: (target, property, value, receiver) => {
            target[property] = value;
            this.updateChildren();
            return true;
          }
        });
        child.children = proxy;
      }
    }
    //
    //   if (group == this && !isAnyMaterialOrLightFound())
    //     parsingWarn(tr("No PBRAppearance, Material and no Light found. "
    //                    "The first child of a LED should be either a Shape, a Light "
    //                    "or a Group containing Shape and Light nodes."));
  }
}
