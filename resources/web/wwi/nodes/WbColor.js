import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {resetMultipleColorIfInvalid} from './utils/WbFieldChecker.js';

// This class is used to retrieve the type of device
export default class WbColor extends WbBaseNode {
  #color;
  constructor(id, color) {
    super(id);
    this.#color = color;
  }

  get color() {
    return this.#color;
  }

  set color(newColor) {
    this.#color = newColor;

    if (this.wrenObjectsCreatedCalled)
      this.#updateColor();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbColor(customID, this.#color);
  }

  delete() {
    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined')
        parent.color = undefined;
    }

    super.delete();
  }

  preFinalize() {
    super.preFinalize();
    this.#sanitizeFields();
  }

  #sanitizeFields() {
    const newColor = resetMultipleColorIfInvalid(this.#color);
    if (newColor !== false)
      this.color = newColor;

    return newColor === false;
  }

  #updateColor() {
    if (!this.#sanitizeFields())
      return;
    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
