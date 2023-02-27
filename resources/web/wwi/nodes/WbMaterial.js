import {resetIfNotInRangeWithIncludedBounds, resetColorIfInvalid} from './utils/WbFieldChecker.js';
import {array3Pointer} from './utils/utils.js';
import WbBaseNode from './WbBaseNode.js';
import WbVector3 from './utils/WbVector3.js';
import WbWorld from './WbWorld.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbMaterial extends WbBaseNode {
  #ambientIntensity;
  #diffuseColor;
  #specularColor;
  #emissiveColor;
  #shininess;
  #transparency;
  constructor(id, ambientIntensity, diffuseColor, specularColor, emissiveColor, shininess, transparency) {
    super(id);
    this.#ambientIntensity = ambientIntensity;
    this.#diffuseColor = diffuseColor;
    this.#specularColor = specularColor;
    this.#emissiveColor = emissiveColor;
    this.#shininess = shininess;
    this.#transparency = transparency;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_MATERIAL;
  }

  get ambientIntensity() {
    return this.#ambientIntensity;
  }

  set ambientIntensity(newAmbientIntensity) {
    this.#ambientIntensity = newAmbientIntensity;

    this.#updateAmbientIntensity();
  }

  get diffuseColor() {
    return this.#diffuseColor;
  }

  set diffuseColor(newDiffuseColor) {
    this.#diffuseColor = newDiffuseColor;

    this.#updateDiffuseColor();
  }

  get specularColor() {
    return this.#specularColor;
  }

  set specularColor(newSpecularColor) {
    this.#specularColor = newSpecularColor;

    this.#updateSpecularColor();
  }

  get emissiveColor() {
    return this.#emissiveColor;
  }

  set emissiveColor(newEmissiveColor) {
    this.#emissiveColor = newEmissiveColor;

    this.#updateEmissiveColor();
  }

  get shininess() {
    return this.#shininess;
  }

  set shininess(newShininess) {
    this.#shininess = newShininess;

    this.#updateShininess();
  }

  get transparency() {
    return this.#transparency;
  }

  set transparency(newTransparency) {
    this.#transparency = newTransparency;

    this.#updateTransparency();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbMaterial(customID, this.#ambientIntensity, this.#diffuseColor, this.#specularColor, this.#emissiveColor,
      this.#shininess, this.#transparency);
  }

  delete() {
    const parent = WbWorld.instance.nodes.get(this.parent);

    if (typeof parent !== 'undefined') {
      const shape = WbWorld.instance.nodes.get(parent.parent);
      if (typeof shape !== 'undefined') {
        parent.material = undefined;
        shape.updateAppearance();
      }
    }
    super.delete();
  }

  modifyWrenMaterial(wrenMaterial, textured) {
    let ambient, diffuse, specular, shininess;

    if (textured) {
      ambient = new WbVector3(this.#ambientIntensity, this.#ambientIntensity, this.#ambientIntensity);
      diffuse = new WbVector3(1.0, 1.0, 1.0);
      specular = new WbVector3(1.0, 1.0, 1.0);
      shininess = 0.0;
    } else {
      ambient = new WbVector3(this.#ambientIntensity * this.#diffuseColor.x, this.#ambientIntensity * this.#diffuseColor.y,
        this.#ambientIntensity * this.#diffuseColor.z);
      diffuse = new WbVector3(this.#diffuseColor.x, this.#diffuseColor.y, this.#diffuseColor.z);
      specular = new WbVector3(this.#specularColor.x, this.#specularColor.y, this.#specularColor.z);
      shininess = this.#shininess;
    }

    const ambientColorPointer = array3Pointer(ambient.x, ambient.y, ambient.z);
    const diffuseColorPointer = array3Pointer(diffuse.x, diffuse.y, diffuse.z);
    const specularColorPointer = array3Pointer(specular.x, specular.y, specular.z);
    const emissiveColorPointer = array3Pointer(this.#emissiveColor.x, this.#emissiveColor.y, this.#emissiveColor.z);

    _wr_phong_material_set_all_parameters(wrenMaterial, ambientColorPointer, diffuseColorPointer, specularColorPointer,
      emissiveColorPointer, shininess, this.#transparency);

    _free(ambientColorPointer);
    _free(diffuseColorPointer);
    _free(specularColorPointer);
    _free(emissiveColorPointer);
  }

  #update() {
    if (this.isPostFinalizedCalled && typeof this.onChange === 'function')
      this.onChange();
  }

  #updateAmbientIntensity() {
    const newAmbientIntensity = resetIfNotInRangeWithIncludedBounds(this.#ambientIntensity, 0, 1, 0.5);
    if (newAmbientIntensity !== false) {
      this.ambientIntensity = newAmbientIntensity;
      return;
    }

    this.#update();
  }

  #updateDiffuseColor() {
    const newDiffuseColor = resetColorIfInvalid(this.#diffuseColor);
    if (newDiffuseColor !== false) {
      this.diffuseColor = newDiffuseColor;
      return;
    }

    this.#update();
  }

  #updateSpecularColor() {
    const newSpecularColor = resetColorIfInvalid(this.#specularColor);
    if (newSpecularColor !== false) {
      this.specularColor = newSpecularColor;
      return;
    }

    this.#update();
  }

  #updateEmissiveColor() {
    const newEmissiveColor = resetColorIfInvalid(this.#emissiveColor);
    if (newEmissiveColor !== false) {
      this.emissiveColor = newEmissiveColor;
      return;
    }

    this.#update();
  }

  #updateShininess() {
    const newShininess = resetIfNotInRangeWithIncludedBounds(this.#shininess, 0, 1, 0.5);
    if (newShininess !== false) {
      this.shininess = newShininess;
      return;
    }

    this.#update();
  }

  #updateTransparency() {
    const newTransparency = resetIfNotInRangeWithIncludedBounds(this.#transparency, 0, 1, 0.5);
    if (newTransparency !== false) {
      this.transparency = newTransparency;
      return;
    }

    this.#update();
  }
}
