import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import {resetMultipleColorIfInvalid} from './utils/WbFieldChecker.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbColor extends WbBaseNode {
  #color;
  constructor(id, color) {
    super(id);
    this.#color = color;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_COLOR;
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
      this.#color = newColor;
  }

  #updateColor() {
    this.#sanitizeFields();

    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
