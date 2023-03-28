import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';

import {resetIfNegative, resetColorIfInvalid} from './utils/WbFieldChecker.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbFog extends WbBaseNode {
  #color;
  #fogType;
  #visibilityRange;
  #wrenFogType;
  constructor(id, color, visibilityRange, fogType) {
    super(id);
    this.#color = color;
    this.#visibilityRange = visibilityRange;
    this.#fogType = fogType;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_FOG;
  }

  get color() {
    return this.#color;
  }

  set color(newColor) {
    this.#color = newColor;

    this.#updateColor();
  }

  get fogType() {
    return this.#fogType;
  }

  set fogType(newFogType) {
    this.#fogType = newFogType;

    this.#updateFogType();
  }

  get visibilityRange() {
    return this.#visibilityRange;
  }

  set visibilityRange(newVisibilityRange) {
    this.#visibilityRange = newVisibilityRange;

    this.#updateVisibilityRange();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.#applyChangesToWren();
  }

  delete() {
    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.root.children.indexOf(this);
      WbWorld.instance.root.children.splice(index, 1);
    }

    if (this.wrenObjectsCreatedCalled) {
      _wr_scene_set_fog(_wr_scene_get_instance(), Enum.WR_SCENE_FOG_TYPE_NONE, Enum.WR_SCENE_FOG_DEPTH_TYPE_PLANE, null, 1.0,
        0.0, 1.0);
    }

    WbWorld.instance.hasFog = false;

    super.delete();
  }

  preFinalize() {
    super.preFinalize();

    this.#updateFogType();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbFog(customID, this.#color, this.#visibilityRange, this.#fogType);
  }

  // Private functions

  #applyChangesToWren() {
    let density = 0.0;
    let fogType = this.#wrenFogType;
    if (this.#visibilityRange > 0.0)
      density = 1.0 / this.#visibilityRange;
    else
      fogType = Enum.WR_SCENE_FOG_TYPE_NONE;

    const colorPointer = _wrjs_array3(this.#color.x, this.#color.y, this.#color.z);
    _wr_scene_set_fog(_wr_scene_get_instance(), fogType, Enum.WR_SCENE_FOG_DEPTH_TYPE_POINT, colorPointer, density,
      0.0, this.#visibilityRange);
  }

  #updateColor() {
    const newColor = resetColorIfInvalid(this.#color);
    if (newColor !== false) {
      this.color = newColor;
      return;
    }
    if (this.wrenObjectsCreatedCalled)
      this.#applyChangesToWren();
  }

  #updateFogType() {
    if (this.#fogType === 'EXPONENTIAL')
      this.#wrenFogType = Enum.WR_SCENE_FOG_TYPE_EXPONENTIAL;
    else if (this.#fogType === 'EXPONENTIAL2')
      this.#wrenFogType = Enum.WR_SCENE_FOG_TYPE_EXPONENTIAL2;
    else
      this.#wrenFogType = Enum.WR_SCENE_FOG_TYPE_LINEAR;

    if (this.#wrenFogType === Enum.WR_SCENE_FOG_TYPE_LINEAR && this.#fogType !== 'LINEAR')
      console.warn("Unknown 'fogType': " + this.#fogType + ', set to "LINEAR" instead.');

    if (this.wrenObjectsCreatedCalled)
      this.#applyChangesToWren();
  }

  #updateVisibilityRange() {
    const newVisibilityRange = resetIfNegative(this.#visibilityRange, 0);
    if (newVisibilityRange !== false) {
      this.visibilityRange = newVisibilityRange;
      return;
    }

    if (this.wrenObjectsCreatedCalled)
      this.#applyChangesToWren();
  }
}
