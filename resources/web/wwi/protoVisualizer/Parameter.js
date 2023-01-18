'use strict';

import {MFNode, SFNode, stringifyType} from './Vrml.js';
import Node from './Node.js';
import {VRML} from './vrml_type.js';
import WbWorld from '../nodes/WbWorld.js';

export default class Parameter {
  #type;
  #name;
  #value;
  #defaultValue;
  #isTemplateRegenerator;
  #restrictions;
  #parameterLinks;
  constructor(node, name, type, restrictions, defaultValue, value, isTemplateRegenerator) {
    this.node = node; // node this parameter belongs to
    this.#type = type;
    this.#name = name;
    this.#restrictions = restrictions;
    this.defaultValue = defaultValue;
    this.value = value;
    this.#isTemplateRegenerator = isTemplateRegenerator;
    this.#parameterLinks = []; // list of other parameters to notify whenever this instance changes
  }

  get restrictions() {
    return this.#restrictions;
  }

  set restrictions(newValue) {
    this.#restrictions = newValue;
  }

  get value() {
    return this.#value;
  }

  set value(v) {
    if (v.type() !== this.type)
      throw new Error('Type mismatch, setting ' + stringifyType(v.type()) + ' to ' + stringifyType(this.type) + ' parameter.');

    if (this.restrictions.length > 0) {
      for (const item of this.restrictions) {
        if ((v instanceof SFNode && v.value === null) || item.equals(v)) {
          this.#value = v;
          return;
        }
      }

      throw new Error('Parameter ' + this.name + ' is restricted and the value being set is not permitted.');
    }

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

  resetParameterLinks() {
    this.#parameterLinks = [];
  }

  insertLink(parameter) {
    this.#parameterLinks.push(parameter);
  }

  insertNode(view, v, index) {
    if (this.type !== VRML.MFNode)
      throw new Error('Item insertion is possible only for MFNodes.')

    for (const link of this.parameterLinks)
      link.insertNode(view, v.clone(), index);

    if (this.node.isProto) { // add value on the structure side
      this.#value.insertNode(v, index);
      return; // webotsJS needs to be notified of parameter changes only if the parameter belongs to a base-node, not PROTO
    }

    // get the parent id to insert the new node
    const baseNode = this.node.getBaseNode();
    const parentId = baseNode.id.replace('n', '');
    const x3d = new XMLSerializer().serializeToString(v.toX3d());
    view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parentId);
    this.#value.insertNode(v, index); // add value on the structure side

    view.x3dScene.render();
  }

  removeNode(view, index) {
    if (this.type !== VRML.MFNode)
      throw new Error('Item insertion is possible only for MFNodes.');

    for (const link of this.parameterLinks)
      link.removeNode(view, index);

    if (this.node.isProto) { // update value on the structure side
      this.#value.removeNode(index);
      return; // webotsJS needs to be notified of parameter changes only if the parameter belongs to a base-node, not PROTO
    }

    if (this.#value.value.length > 0) {
      // delete existing node
      const baseNode = this.node.getBaseNode();
      const p = baseNode.getParameterByName(this.name);
      const id = p.value.value[index].value.getBaseNode().id;

      view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);
      this.#value.removeNode(index); // update value on the structure side
    }

    view.x3dScene.render();
  }

  setValueFromJavaScript(view, v, index) {
    console.log(this.name, ' change to ', v, 'node:', this.node)

    if (this.isTemplateRegenerator) {
      console.log(this.name, 'is a template regenerator!')

      const jsNode = WbWorld.instance.nodes.get(this.node.getBaseNode().id);
      const parentNodeId = jsNode.parent;

      console.log(`delete: ${this.node.id.replace('n', '')}`);
      view.x3dScene.processServerMessage(`delete: ${this.node.getBaseNode().id.replace('n', '')}`);

      // update the value of the parameter
      this.value.setValueFromJavaScript(v);
      this.node.regenerate();

      const x3d = new XMLSerializer().serializeToString(this.node.toX3d());
      console.log('Insert regenerated x3d into node', parentNodeId)
      view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parentNodeId.replace('n', ''));
      return;
    }

    if (v instanceof Node) {
      for (const link of this.parameterLinks) {
        console.log('notifying link', link)
        const parentId = link.node.getBaseNode().id;
        console.log('myId:', this.value.value.getBaseNode().id, 'parentId:', parentId)
        view.x3dScene.processServerMessage(`delete: ${this.value.value.getBaseNode().id.replace('n', '')}`);
        console.log(`delete: ${this.value.value.id}`);

        // update the parameter (must happen after the existing node is deleted or the information is lost)
        this.value.value = v;

        const x3d = new XMLSerializer().serializeToString(v.toX3d());
        console.log(x3d)
        view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parentId.replace('n', ''));
      }
    } else {
      console.log('pose change required')
      // update the parameter
      this.value.setValueFromJavaScript(v);

      for (const link of this.parameterLinks) {
        const action = {}
        action['id'] = link.node.getBaseNode().id;
        action[link.name] = this.value.toJson();
        console.log('setPose', action);
        view.x3dScene.applyPose(action);
        view.x3dScene.render();
      }
    }
  }

  // TODO: find better approach rather than propagating the view to subsequent parameters
  /*
  setValueFromJavaScript(view, v, index) {
    // notify linked parameters of the change
    for (const link of this.parameterLinks)
      link.setValueFromJavaScript(view, (v !== null && v instanceof Node) ? v.clone() : v, index);

    if (this.isTemplateRegenerator) {
      // regenerate this node, and all its siblings
      this.#value.setValueFromJavaScript(v, index);
      this.node.regenerateNode(view);

      if (typeof this.onChange === 'function')
        this.onChange();
    } else {
      if (this.node.isProto) {
        // update value on the structure side
        this.#value.setValueFromJavaScript(v, index);
        return; // webotsJS needs to be notified of parameter changes only if the parameter belongs to a base-node, not PROTO
      }

      if (this.#value instanceof SFNode) {
        const baseNode = this.node.getBaseNode();
        if (this.#value.value !== null) {
          // delete existing node
          const p = baseNode.getParameterByName(this.name);
          const id = p.value.value.getBaseNode().id;
          view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);
        }

        // update value on the structure side
        this.#value.setValueFromJavaScript(v);

        if (v !== null) {
          // get the parent id to insert the new node
          const parentId = baseNode.id.replace('n', '');

          const x3d = new XMLSerializer().serializeToString(v.toX3d());
          view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parentId);
        }
      } else if (this.#value instanceof MFNode) {
        const baseNode = this.node.getBaseNode();
        if (this.#value.value.length > 0) { // delete existing node
          const p = baseNode.getParameterByName(this.name);
          const id = p.value.value[index].value.getBaseNode().id;
          view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);
        }

        // update value on the structure side
        this.#value.setValueFromJavaScript(v, index);

        // get the parent id to insert the new node and notify webotsjs
        const parentId = baseNode.id.replace('n', '');
        v.forEach((item) => {
          const x3d = new XMLSerializer().serializeToString(item.toX3d());
          view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parentId);
        });
      } else {
        // update value on the structure side
        this.#value.setValueFromJavaScript(v);

        // update value on the webotsJS side
        const action = {};
        action['id'] = this.node.id;
        action[this.name] = this.value.toJson();
        console.log('setPose', action);
        view.x3dScene.applyPose(action);
      }
      view.x3dScene.render();
    }
  }
  */

  isDefault() {
    if (typeof this.defaultValue === 'undefined' || typeof this.value === 'undefined')
      throw new Error('Cannot check default-ness, either "value" or "defaultValue" is undefined.');

    return this.value.equals(this.defaultValue);
  }

  clone(deep = false) {
    const restrictions = [];
    for (const item of this.restrictions)
      restrictions.push(item.clone(deep));

    const copy = new Parameter(this.node, this.name, this.type, restrictions,
      deep ? this.defaultValue.clone(deep) : this.defaultValue,
      deep ? this.value.clone(deep) : this.value, this.isTemplateRegenerator);

    copy.#parameterLinks = [];
    for (const item of this.parameterLinks)
      copy.insertLink(item.clone(deep));

    return copy;
  }
}

export { Parameter };
