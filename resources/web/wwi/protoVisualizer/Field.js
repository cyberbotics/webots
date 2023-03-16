'use strict';

import {stringifyType} from './Vrml.js';
import Node from './Node.js';
import {VRML} from './vrml_type.js';
import WbWorld from '../nodes/WbWorld.js';

export default class Field {
  #node;
  #type;
  #name;
  #value;
  #defaultValue;
  #restrictions;
  constructor(node, name, type, defaultValue, value) {
    this.#node = node;
    this.#name = name;
    this.#type = type;
    this.#value = value;
    this.#defaultValue = defaultValue;
    this.#restrictions = [];
  }

  get node() {
    return this.#node;
  }

  get value() {
    return this.#value;
  }

  set value(newValue) {
    this.#value = newValue;
  }

  get defaultValue() {
    return this.#defaultValue;
  }

  set defaultValue(newValue) {
    if (newValue.type() !== this.type)
      throw new Error(`Type mismatch, setting ${stringifyType(newValue.type())} to ${stringifyType(this.type)} parameter.`);

    this.#defaultValue = newValue;
  }

  get type() {
    return this.#type;
  }

  get name() {
    return this.#name;
  }

  get restrictions() {
    return this.#restrictions;
  }

  set restrictions(newValue) {
    this.#restrictions = newValue;
  }

  isDefault() {
    if (typeof this.defaultValue === 'undefined' || typeof this.value === 'undefined')
      throw new Error('Cannot check default-ness, either "value" or "defaultValue" is undefined.');

    return this.value.equals(this.defaultValue);
  }

  insertNode(view, v, index) {
    if (this.type !== VRML.MFNode)
      throw new Error('Item insertion is possible only for MFNodes.');

    if (this.isTemplateRegenerator) {
      this.regenerate(view, v, index);
      return;
    }

    this.value.insertNode(v, index, this);

    for (const link of this.linksToNotify()) {
      // insert the new node on the webotsjs side
      for (const id of link.node.getBaseNodeIds()) {
        v.assignId();
        const x3d = new XMLSerializer().serializeToString(v.toX3d(this));
        view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', id.replace('n', ''));
      }
    }

    view.x3dScene.render();
  }

  removeNode(view, index) {
    if (this.type !== VRML.MFNode)
      throw new Error('Item insertion is possible only for MFNodes.');

    if (this.isTemplateRegenerator) {
      this.regenerate(view, undefined, index);
      return;
    }

    const links = this.linksToNotify();
    const idsToDelete = new Set();

    for (const link of links) {
      // determine ids of nodes that need to be deleted on the webotsjs side
      for (const id of link.value.value[index].value.getBaseNodeIds())
        idsToDelete.add(id);
    }

    // delete node on the structure side
    this.value.removeNode(index);

    // delete existing nodes on the webotsjs side
    for (const id of idsToDelete)
      view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);

    view.x3dScene.render();
  }

  setValueFromJavaScript(view, v) {
    if (this.isTemplateRegenerator) {
      this.regenerate(view, v);
      return;
    }

    if (v instanceof Node || v === null) {
      const links = this.linksToNotify();
      const parentIds = new Set();
      const idsToDelete = new Set();

      for (const link of links) {
        // determine parent node ids and ids of nodes that need to be deleted on the webotsjs side
        for (const id of link.node.getBaseNodeIds()) {
          parentIds.add(id);

          if (link.value.value !== null) {
            for (const id of link.value.value.getBaseNodeIds())
              idsToDelete.add(id);
          }
        }
      }

      // delete existing nodes
      for (const id of idsToDelete)
        view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);

      // update the parameter value (note: all IS instances refer to the parameter itself, so they don't need to change)
      this.value.setValueFromJavaScript(v);

      if (v !== null) {
        this.value.value.parentField = this;
        // insert the new node on the webotsjs side
        for (const parent of parentIds) {
          v.assignId();
          const x3d = new XMLSerializer().serializeToString(v.toX3d(this));
          view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parent.replace('n', ''));
        }
      }
    } else {
      // update the parameter
      this.value.setValueFromJavaScript(v);

      const links = this.linksToNotify();
      for (const link of links) {
        for (const id of link.node.ids) {
          const action = {};
          action['id'] = id;
          action[link.name] = this.value.toJson();
          view.x3dScene.applyUpdate(action);
        }
      }
    }

    view.x3dScene.render();
  }

  regenerate(view, v, index) {
    const parentIds = new Set();
    for (const id of this.node.getBaseNodeIds()) {
      const jsNode = WbWorld.instance.nodes.get(id);
      parentIds.add(jsNode.parent);
      view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);
    }

    // now that the nodes have been deleted on the webotsjs side, the corresponding ids need to be cleared in the structure
    this.node.resetIds();

    // update the parameter value (note: all IS instances refer to the parameter itself, so they don't need to change)
    if (typeof v === 'undefined' && typeof index !== 'undefined')
      this.value.removeNode(index);
    else if (typeof index !== 'undefined')
      this.value.insertNode(v, index, this);
    else if (typeof v !== 'undefined')
      this.value.setValueFromJavaScript(v);

    this.node.createBaseType(); // regenerate the base type

    this.node.resetRefs(); // reset the instance counters
    const promises = [];
    // insert the new node on the webotsjs side
    for (const id of parentIds) {
      // note: there must be as many id assignments as there are parents, this is because on the webotsjs side the instances
      // need to be distinguishable, so each "IS" needs to notify webotsjs and provide a unique variant of the x3d
      // (with unique ids) for each node
      this.node.assignId();
      const x3d = new XMLSerializer().serializeToString(this.node.toX3d(this));
      promises.push(view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', id.replace('n', '')));
    }

    Promise.all(promises).then(() => view.x3dScene.render());
  }

  linksToNotify() {
    return [this];
  }
}

export { Field };
