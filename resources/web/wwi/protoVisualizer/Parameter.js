'use strict';

import { SFNode, stringifyType } from './Vrml.js';
import { VRML } from './vrml_type.js';
import Node from './Node.js';
import WbWorld from '../nodes/WbWorld.js';

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

    if (type === VRML.SFNode) {
      if (this.#defaultValue.value !== null)
        this.#defaultValue.value.parentParameter = this;
      if (this.#value.value !== null)
        this.#value.value.parentParameter = this;
    }

    this.isTemplateRegenerator = isTemplateRegenerator;
    this.#parameterLinks = []; // list of other parameters to notify whenever this instance changes
    //this.clones = [];
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

  resetParameterLinks() {
    this.#parameterLinks = [];
  }

  insertLink(parameter) {
    this.#parameterLinks.push(parameter);
  }

  // TODO: find better approach rather than propagating the view to subsequent parameters
  setValueFromJavaScript(view, v) {
    // notify linked parameters of the change
    console.log(this.name, ' has links: ', this.parameterLinks);
    for (const link of this.parameterLinks) {
      console.log('NOTIFYING LINK', link.name, 'TO SET VALUE', v)
      const newValue = (v !== null && v instanceof Node) ? v.clone() : v;
      // console.log(this.name + ' change notifies ' + link.name);
      link.setValueFromJavaScript(view, newValue);
    }

    if (this.isTemplateRegenerator) {
      // regenerate this node, and all its siblings
      console.log('doing parameter ' + this.name + ' of node ' + this.node.name + ' (which belongs to parameter: ', this.node.parentParameter?.name, 'node id:', this.node.parentParameter?.node.id, ')')
      console.log('siblings of ' + this.node.id + ': ', Node.cNodeFamily.get(this.node.id))
      console.log('setting value of parameter', this)
      this.#value.setValueFromJavaScript(v);
      this.node.regenerate(v, view);

      /*
      // update value on the structure side
      this.#value.setValueFromJavaScript(v);

      console.log('  > ' + this.name + ' is a template regenerator!');

      if (!this.node.isProto)
        throw new Error('Attempting to regenerate a base node.');

      const tolist = this.node.getBaseNode().to;
      const parentList = []
      for (const item of tolist) {
        const node = WbWorld.instance.nodes.get(item.id);
        if (typeof node !== 'undefined') {
          console.log('>>>>> ', item.id, ' is a node, request deletion');
          view.x3dScene.processServerMessage(`delete: ${item.id.replace('n', '')}`);
          console.log('>>>>>> adding parent:', node.parent)
          parentList.push(node.parent)
        }
      }

      // regenerate and parse the body of the associated node
      this.node.parseBody(true);

      console.log('####', Node.cProtoBaseNodeLinks, parentList)

      let j = 0;
      for (let i = 0; i < tolist.length; ++i) {
        if (Node.cProtoBaseNodeLinks.has(tolist[i].id)) {
          console.log('ADDING', tolist[i].id, 'to', parentList[j])
          const protoNode = Node.cProtoBaseNodeLinks
          const x3d = new XMLSerializer().serializeToString(this.node.toX3d());
          console.log('>>>>>>>>>', this.node)
          view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parentList[j++].replace('n', ''));
        }
      }

      if (typeof this.onChange === 'function')
        this.onChange();
      console.log('AFTER REGEN', WbWorld.instance)
      */
    } else {
      console.log('here', this.node.isProto, this.#value instanceof SFNode)
      if (this.node.isProto) {
        // update value on the structure side
        this.#value.setValueFromJavaScript(v);
        return; // webotsJS needs to be notified of parameter changes only if the parameter belongs to a base-node, not PROTO
      }

      if (this.#value instanceof SFNode) {
        const baseNode = this.node.getBaseNode();
        if (this.#value.value !== null) {
          // delete existing node
          const p = baseNode.getParameterByName(this.name);
          const id = p.value.value.getBaseNode().id;

          // console.log('param', p, 'value', p.value.value)
          console.log('request:', `delete: ${id.replace('n', '')}`);
          view.x3dScene.processServerMessage(`delete: ${id.replace('n', '')}`);
        }

        // update value on the structure side
        this.#value.setValueFromJavaScript(v);

        if (v !== null) {
          // get the parent id to insert the new node
          const parentId = baseNode.id.replace('n', '');

          const x3d = new XMLSerializer().serializeToString(v.toX3d());
          console.log('in parent ', parentId, ' insert:' + x3d);
          view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', parentId);
          console.log(WbWorld.instance)
        }
      } else {
        // update value on the structure side
        this.#value.setValueFromJavaScript(v);

        // update value on the webotsJS side
        const action = {};
        action['id'] = this.node.id;
        action[this.name] = this.value.toJson();
        console.log('setPose', action)
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

  clone(deep = false) {
    const copy = new Parameter(this.node, this.name, this.type, this.defaultValue.clone(deep), this.value.clone(deep),
      this.isTemplateRegenerator);

    //copy.defaultValue.parameterRef = 'ERG';
    copy.value.parameterRef = 'ERG';
    //copy.clones = this.clones.slice();

    //if (copy.type === VRML.SFNode) {
    //  console.log('REDIRECTING PARENTPARAM IN PARAMETER CLONE')
    //  copy.defaultValue.parentParameter = copy
    //  copy.value.parentParameter = copy;
    //}

    return copy;
  }
}

export { Parameter };
