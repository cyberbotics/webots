'use strict';

import {SFNode, stringifyType} from './Vrml.js';

export default class Parameter {
  #type;
  #name;
  #value;
  #defaultValue;
  #isTemplateRegenerator;
  #parameterLinks;
  constructor(node, name, type, defaultValue, value, isTemplateRegenerator) {
    this.node = node; // node this parameter belongs to
    this.type = type;
    this.name = name;
    this.defaultValue = defaultValue;
    this.value = value;
    this.isTemplateRegenerator = isTemplateRegenerator;
    this.parentNode = undefined;
    this.#parameterLinks = []; // list of other parameters to notify whenever this instance changes
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

  get parameterLinks() {
    return this.#parameterLinks;
  }

  insertLink(parameter) {
    this.#parameterLinks.push(parameter);
  }

  // TODO: find better approach rather than propagating the view to subsequent parameters
  setValueFromJavaScript(view, v) {
    // notify linked parameters of the change
    for (const link of this.parameterLinks) {
      console.log(this.name + ' change notifies ' + link.name);
      link.setValueFromJavaScript(view, v);
    }

    if (this.isTemplateRegenerator) {
      // update value on the structure side
      this.#value.setValueFromJavaScript(v); // TODO: should be move after delete?

      console.log('  > ' + this.name + ' is a template regenerator!');

      if (!this.node.isProto)
        throw new Error('Attempting to regenerate a base node.'); // TODO: can we reach this point anyway? if so, just return

      // note: only base-nodes write to x3d, so to know the ID of the node we need to delete, we need to navigate through the
      // value of the proto (or multiple times if it's a derived PROTO)
      let baseNode = this.node;
      while (baseNode.isProto)
        baseNode = baseNode.baseType;

      // delete existing node (must be done prior to regeneration or the information is lost)
      const id = baseNode.id;
      view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);

      // regenerate and parse the body of the associated node
      this.node.parseBody(true);

      const x3d = new XMLSerializer().serializeToString(this.node.toX3d());
      view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>');
    } else {
      if (this.node.isProto)
        return; // webotsJS needs to be notified of parameter changes only if the parameter belongs to a base-node, not PROTO

      if (this.#value instanceof SFNode) {
        let baseNode = this.node;
        while (baseNode.isProto)
          baseNode = baseNode.baseType;

        const p = baseNode.getParameterByName(this.name);
        const id = p.value.value.getBaseNodeId();

        // get parent node
        const parentId = baseNode.getBaseNodeId().replace('n', '');
        console.log('parent node:' + parentId);

        // delete existing node
        view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);
        console.log('delete: ', id);

        // update value on the structure side
        this.#value.setValueFromJavaScript(v);

        const x3d = new XMLSerializer().serializeToString(v.toX3d());
        console.log('insert:' + x3d);
        view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parentId);
      } else {
        // update value on the structure side
        this.#value.setValueFromJavaScript(v);

        const action = {};
        action['id'] = this.node.id;
        action[this.name] = this.value.toJson();
        view.x3dScene.applyPose(action);
      }
      view.x3dScene.render();
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
