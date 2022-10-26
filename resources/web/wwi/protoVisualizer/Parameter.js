'use strict';

import {stringifyType} from './Vrml.js';

export default class Parameter {
  #type;
  #name;
  #value;
  #defaultValue;
  #isTemplateRegenerator;
  constructor(node, name, type, defaultValue, value, isTemplateRegenerator) {
    this.node = node; // node this parameter belongs to
    this.type = type;
    this.name = name;
    this.defaultValue = defaultValue;
    this.value = value;
    this.isTemplateRegenerator = isTemplateRegenerator;
    this.parentNode = undefined;
  }

  get value() {
    return this.#value;
  }

  set value(v) {
    if (v.type() !== this.type)
      throw new Error('Type mismatch, setting ' + stringifyType(v.type()) + ' to ' + stringifyType(this.type) + ' parameter.');

    this.#value = v;
  }

  get defaultValue() {
    return this.#defaultValue;
  }

  set defaultValue(v) {
    if (v.type() !== this.type)
      throw new Error('Type mismatch, setting ' + stringifyType(v.type()) + ' to ' + stringifyType(this.type) + ' parameter.');

    this.#defaultValue = v;
  }

  get type() {
    return this.#type;
  }

  set type(type) {
    this.#type = type;
  }

  get name() {
    return this.#name;
  }

  set name(value) {
    this.#name = value;
  }

  get isTemplateRegenerator() {
    return this.#isTemplateRegenerator;
  }

  set isTemplateRegenerator(value) {
    this.#isTemplateRegenerator = value;
  }

  setValueFromJavaScript(v) {
    this.#value.setValueFromJavaScript(v);
    if (this.isTemplateRegenerator) {
      const proto = this.node;

      // note: only base-nodes write to x3d, so to know the ID of the node we need to delete, we need to navigate through the
      // value of the proto (or multiple times if it's a derived PROTO)
      let baseNode = proto;
      console.log(proto)
      while (baseNode.isProto)
        baseNode = baseNode.baseType;

      // id to delete
      const id = baseNode.id;

      // delete js side
      this._view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);

      proto.parseBody(true);
      const x3d = new XMLSerializer().serializeToString(proto.toX3d());
      this._view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>');
    } else if (this.parentNode?.aliasLinks) {
      for (const linkedParameter of this.parentNode?.aliasLinks) {
        const id = linkedParameter.parentNode.id;
        const action = {'id': id.replace('n', ''), baseColor: v.r + ' ' + v.g + ' ' + v.b};
        this._view.x3dScene.applyPose(action);
        this._view.x3dScene.render();
      }
    }
  }

  isDefault() {
    if (typeof this.defaultValue === 'undefined' || typeof this.value === 'undefined')
      throw new Error('Cannot check default-ness, either "value" or "defaultValue" is undefined.');

    return this.value.equals(this.defaultValue);
  }

  clone() {
    const copy = new Parameter(this.node, this.name, this.type, this.defaultValue.clone(), this.value.clone(),
      this.isTemplateRegenerator);
    copy.parentNode = this.parentNode;
    return copy;
  }
}

export {Parameter};
