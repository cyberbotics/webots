'use strict';

import {SFNode, stringifyType} from './Vrml.js';
import Node from './Node.js';
import {VRML} from './vrml_type.js';
import WbWorld from '../nodes/WbWorld.js';
import Field from './Field.js';

export default class Parameter extends Field {
  #restrictions;
  #aliasLinks;
  #isTemplateRegenerator;
  constructor(node, name, type, defaultValue, value, restrictions, isTemplateRegenerator) {
    super(node, name, type, value, defaultValue);

    this.#restrictions = restrictions;
    this.#isTemplateRegenerator = isTemplateRegenerator;
    this.#aliasLinks = []; // list of other parameters to notify whenever this instance changes
  }

  get restrictions() {
    return this.#restrictions;
  }

  set restrictions(newValue) {
    this.#restrictions = newValue;
  }

  get value() {
    return super.value;
  }

  set value(newValue) {
    if (newValue.type() !== this.type)
      throw new Error(`Type mismatch, setting ${stringifyType(newValue.type())} to ${stringifyType(this.type)} parameter.`);

    if (this.restrictions.length > 0) {
      for (const item of this.restrictions) {
        if ((newValue instanceof SFNode && newValue.value === null) || item.equals(newValue)) {
          super.value = newValue;
          return;
        }
      }

      throw new Error(`Parameter ${this.name} is restricted and the value being set is not permitted.`);
    }

    super.value = newValue;
  }

  get isTemplateRegenerator() {
    return this.#isTemplateRegenerator;
  }

  set isTemplateRegenerator(value) {
    this.#isTemplateRegenerator = value;
  }

  get aliasLinks() {
    return this.#aliasLinks;
  }

  set aliasLinks(newValue) {
    this.#aliasLinks = newValue;
  }

  addAliasLink(parameter) {
    this.#aliasLinks.push(parameter);
  }

  resetAliasLinks() {
    this.#aliasLinks = [];
  }

  insertNode(view, v, index) {
    if (this.type !== VRML.MFNode)
      throw new Error('Item insertion is possible only for MFNodes.');

    for (const link of this.linksToNotify()) {
      link.value.insertNode(v, index);

      // insert the new node on the webotsjs side
      for (const id of link.node.getBaseNodeIds()) {
        v.assignId();
        const x3d = new XMLSerializer().serializeToString(v.toX3d());
        view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', id.replace('n', ''));
      }

      view.x3dScene.render();
    }
  }

  removeNode(view, index) {
    if (this.type !== VRML.MFNode)
      throw new Error('Item insertion is possible only for MFNodes.');

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
      const parentIds = new Set();
      for (const id of this.node.getBaseNodeIds()) {
        const jsNode = WbWorld.instance.nodes.get(id);
        parentIds.add(jsNode.parent);
        view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);
      }

      // now that the nodes have been deleted on the webotsjs side, the corresponding ids need to be cleared in the structure
      this.node.resetIds();

      // update the parameter value (note: all IS instances refer to the parameter itself, so they don't need to change)
      this.value.setValueFromJavaScript(v);
      this.node.createBaseType(); // regenerate the base type

      this.node.resetRefs(); // reset the instance counters
      // insert the new node on the webotsjs side
      for (const id of parentIds) {
        // note: there must be as many id assignments as there are parents, this is because on the webotsjs side the instances
        // need to be distinguishable, so each "IS" needs to notify webotsjs and provide a unique variant of the x3d
        // (with unique ids) for each node
        this.node.assignId();
        const x3d = new XMLSerializer().serializeToString(this.node.toX3d());
        view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', id.replace('n', ''));
      }

      view.x3dScene.render();
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
        // insert the new node on the webotsjs side
        for (const parent of parentIds) {
          v.assignId();
          const x3d = new XMLSerializer().serializeToString(v.toX3d());
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

  linksToNotify() {
    let links = [];
    for (const link of this.aliasLinks) {
      if (link instanceof Parameter && link.aliasLinks.length > 0)
        links = links.concat(link.linksToNotify());

      if (!link.node.isProto)
        links.push(link);
    }

    return links;
  }
}

export { Parameter };
