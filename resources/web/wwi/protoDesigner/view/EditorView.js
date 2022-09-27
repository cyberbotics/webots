'use strict';

import WbWorld from '../../wwi/nodes/WbWorld.js';

import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';

import {VRML} from '../classes/utility/utility.js';
import Proto from '../classes/Proto.js';

export default class EditorView { // eslint-disable-line no-unused-vars
  constructor(element, view, designer, library) {
    // setup parameter list view
    this.element = element;
    this.view = view;
    this.designer = designer;
    this.library = library;

    this.parameter = undefined; // currently selected parameter in the UI
    this.proto = undefined; // currently referenced proto (based on the active parameter)

    this.forms = new Map();
  }

  refreshParameters() {
    this.element.innerHTML = ''; // clean HTML
    console.log('Cleared EditorView innnerHTML');
    this.setupModalWindow();

    if (typeof this.designer.baseRobot !== 'undefined')
      this.populateDiv(this.designer.baseRobot);
  }

  populateDiv(proto, depth = 0, parent = this.element) {
    console.log('DEPTH ' + depth, proto);
    // add PROTO name label
    if (depth === 0) {
      let nameLabel = document.createElement('p');
      nameLabel.innerHTML = '<span class="proto-name-label">' + proto.protoName + '</span>';

      this.element.appendChild(nameLabel);
    }

    // display parameters
    for (const [key, parameter] of proto.parameters.entries()) {
      const div = this.setupParameter(proto, parameter, key, parent);

      if (parameter.type === VRML.SFNode && parameter.value instanceof Proto)
        this.populateDiv(parameter.value, depth + 1, div);
    }
  }

  setupParameter(proto, parameter, parameterId, parent) {
    const unsupported = [VRML.MFString, VRML.MFBool, VRML.MFFloat, VRML.MFInt32, VRML.MFVec2f, VRML.MFVec3f, VRML.MFRotation, VRML.MFNode];
    if (unsupported.includes(parameter.type))
      return;

    let form = document.createElement('form');
    form.setAttribute('onsubmit', 'return false;');
    form.setAttribute('parameterType', parameter.type);
    form.setAttribute('protoId', proto.id);
    form.setAttribute('parameterId', parameterId);

    let div = document.createElement('div');
    div.classList.add('parameter-div');

    let label = document.createElement('label');
    label.classList.add('parameter-label');
    label.innerText = parameter.name;
    //if (parameter.type === VRML.SFNode)
    //  label.addEventListener('click', () => this.itemSelector(event));

    div.appendChild(label);

    const value = parameter.value;

    switch (parameter.type) {
      case VRML.SFString: {
        let input = document.createElement('input');
        input.setAttribute('type', 'text');
        input.setAttribute('value', value);
        input.addEventListener('input', this.updateValue.bind(this));
        div.appendChild(input);
        break;
      }
      case VRML.SFBool: {
        let input = document.createElement('input');
        input.setAttribute('type', 'checkbox');
        input.checked = value;
        input.addEventListener('input', this.updateValue.bind(this));
        div.appendChild(input);
        break;
      }
      case VRML.SFInt32: {
        let input = document.createElement('input');
        input.setAttribute('type', 'number');
        input.setAttribute('step', '1');
        input.setAttribute('value', value);
        input.addEventListener('input', this.updateValue.bind(this));
        div.appendChild(input);
        break;
      }
      case VRML.SFFloat: {
        let input = document.createElement('input');
        input.setAttribute('type', 'number');
        input.setAttribute('step', '0.1');
        input.setAttribute('value', value);
        input.addEventListener('input', this.updateValue.bind(this));
        div.appendChild(input);
        break;
      }
      case VRML.SFVec2f:
        for (let i = 0; i < 2; ++i) {
          let input = document.createElement('input');
          input.setAttribute('type', 'number');
          input.setAttribute('step', '0.1');
          input.setAttribute('value', i === 0 ? value.x : value.y);
          input.addEventListener('input', this.updateValue.bind(this));
          div.appendChild(input);
        }
        break;
      case VRML.SFVec3f:
      case VRML.SFColor:
        for (let i = 0; i < 3; ++i) {
          let input = document.createElement('input');
          input.setAttribute('type', 'number');
          input.setAttribute('step', '0.1');
          if (parameter.type === VRML.SFColor) {
            input.setAttribute('max', '1');
            input.setAttribute('min', '0');
          }
          input.setAttribute('value', i === 0 ? value.x : i === 1 ? value.y : value.z);
          input.addEventListener('input', this.updateValue.bind(this));
          div.appendChild(input);
        }
        break;
      case VRML.SFRotation:
        for (let i = 0; i < 4; ++i) {
          let input = document.createElement('input');
          input.setAttribute('type', 'number');
          input.setAttribute('step', '0.1');
          input.setAttribute('value', i === 0 ? value.x : i === 1 ? value.y : i === 2 ? value.z : value.w);
          input.addEventListener('input', this.updateValue.bind(this));
          div.appendChild(input);
        }
        break;
      case VRML.SFNode:
        let button = document.createElement('button');
        button.classList.add('sfnode-button');
        if (typeof parameter.value === 'undefined')
          button.innerText = 'NULL';
        else
          button.innerText = parameter.value.protoName;
        button.addEventListener('click', () => this.itemSelector(event));
        div.appendChild(button);
        break;
      default:
        throw new Error('Cannot setup div because parameter type \'' + parameter.type + '\' is unknown.');
    }

    form.appendChild(div);
    parent.appendChild(form);

    return div;
  }

  itemSelector(e) {
    const protoId = parseInt(e.target.form.attributes['protoId'].value);
    const parameterId = e.target.form.attributes['parameterId'].value;
    console.log('Clicked item parameterId = ' + parameterId + ' (protoId = ' + protoId + ')');

    this.proto = this.designer.protoMap.get(protoId);
    this.parameter = this.proto.parameters.get(parameterId);

    // determine filtering condition
    const triggered = this.proto.getTriggeredFields(this.parameter);
    if (triggered.length > 1)
      console.warn('A parameter selection triggered multiple nodes for insertion, is it normal?');
    let filter;
    const slot = WbWorld.instance.nodes.get(triggered[0].nodeRef);
    filter = slot.type.slice(1, -1); // remove quotations

    // button position
    const position = e.target.getBoundingClientRect();
    // get the modal
    let modal = document.getElementById('modalWindow');
    modal.style.display = 'block';
    modal.style.top = position.y + 'px';
    modal.style.left = (position.x + position.width) + 'px';
    this.library.loadAssets(modal, filter, this.nodeInsertion.bind(this));
  }

  nodeRemoval() {
    if (this.parameter.value instanceof Proto) {
      const x3dNodes = this.parameter.value.x3dNodes;
      console.log('Removing node: ' + x3dNodes[0]);
      this.view.x3dScene._deleteObject(parseInt(x3dNodes[0].slice(1)));
      this.view.x3dScene.render();

      let value;
      if (this.parameter.defaultValue instanceof WbVector2 || this.parameter.defaultValue instanceof WbVector3 ||
          this.parameter.defaultValue instanceof WbVector4)
        value = this.parameter.defaultValue.clone();
      else if (typeof defaultValue === 'undefined')
        value = undefined;
      else
        value = this.parameter.defaultValue.valueOf();

      this.parameter.value = value;
      this.refreshParameters();
    }
  }

  nodeInsertion(e) {
    const modal = document.getElementById('modalWindow');
    modal.style.display = 'none';

    const assetKey = e.target.attributes['assetKey'].value;
    console.log('Selected asset: ' + assetKey);

    if (assetKey === 'remove') {
      this.nodeRemoval();
      return;
    }

    // get nodeRef (i.e the parent of the node that will be created now)
    const triggered = this.proto.getTriggeredFields(this.parameter);
    for (let i = 0; i < triggered.length; ++i) {
      const nodeRef = triggered[i].nodeRef;
      const node = WbWorld.instance.nodes.get(nodeRef);
      const nodeId = parseInt(node.id.slice(1));
      const asset = this.library.getAsset(assetKey);
      this.designer.loadProto(asset.url, nodeId, this.parameter);
    }
  }

  setupModalWindow() {
    // create modal window
    let div = document.createElement('div');
    div.className = 'modal';
    div.setAttribute('id', 'modalWindow');

    this.element.appendChild(div);
  }

  closeModalWindow() {
    let modal = document.getElementById('modalWindow');
    modal.style.display = 'none';
  }

  /*
  insertNode(protoUrl) {
    if (this.parameter.type === VRML.SFNode) {
      // get parent
      const nodeRefs = this.parameter.nodeRefs;
      const node = WbWorld.instance.nodes.get(nodeRefs[0]);
      const nodeId = parseInt(node.id.slice(1));

      this.designer.loadProto(protoUrl, nodeId, this.parameter);
    };
  };
  */

  updateValue(e) {
    const protoId = parseInt(e.target.form.attributes['protoId'].value);
    const parameterId = e.target.form.attributes['parameterId'].value;
    console.log('Clicked item parameterId = ' + parameterId + ' (protoId = ' + protoId + ')');

    this.proto = this.designer.protoMap.get(protoId);
    this.parameter = this.proto.parameters.get(parameterId);

    const newValue = this.getValuesFromForm(e.target.form);
    if (typeof this.parameter.value === typeof newValue)
      this.parameter.value = newValue;
    else
      throw new Error('Overwriting value of type ' + typeof this.parameter.value + ' with value of type ' + typeof newValue);

    if (this.parameter.isTemplateRegenerator) {
      console.log('Regeneration triggered by parameter ' + this.parameter.name);
      // save parent reference
      const parentId = this.proto.x3dNodes[0];
      const parentNode = WbWorld.instance.nodes.get(parentId).parent;
      // delete proto nodes
      for (let i = 0; i < this.proto.x3dNodes.length; ++i) {
        const node = this.proto.x3dNodes[i];
        console.log('Deleting node: ' + node);
        this.view.x3dScene._deleteObject(parseInt(node.slice(1))); // remove 'n' prefix
      }
      // regenerate VRML and parse x3d
      this.proto.parseBody();
      const insertPoint = typeof parentNode !== 'undefined' ? parentNode.slice(1) : undefined;
      this.view.x3dScene._loadObject(this.proto.x3d, insertPoint);
      // this.view.x3dScene.loadWorldFileRaw(this.proto.x3d, this.view.finalizeWorld);
      this.view.x3dScene.render();
      return;
    }

    // propagate change to IS references
    const triggered = this.proto.getTriggeredFields(this.parameter);
    for (let i = 0; i < triggered.length; ++i) {
      const nodeRef = triggered[i].nodeRef;
      const fieldName = triggered[i].fieldName;
      console.log('> setting parameter \'' + fieldName + '\' to ', this.parameter.value);
      const node = WbWorld.instance.nodes.get(nodeRef);
      node.setParameter(fieldName, this.parameter.value);
      // if the node has any USE references, propagate change further
      for (let j = 0; j < node.useList.length; ++j) {
        console.log('>> setting (through DEF) parameter \'' + fieldName + '\' to ', this.parameter.value);
        const useNode = WbWorld.instance.nodes.get(node.useList[j]);
        useNode.setParameter(fieldName, this.parameter.value);
      }
    }

    this.view.x3dScene.render();
  }

  getFormByParameterId(id) {
    let forms = document.getElementsByTagName('form');
    for (let i = 0; i < forms.length; i++) {
      if (forms[i].attributes['parameterId'].value === id)
        return forms[i];
    }
  }

  getValuesFromForm(form) {
    if (typeof form === 'undefined')
      throw new Error('Cannot get values from unknown form');

    const elements = form.elements;
    switch (parseInt(form.attributes['parameterType'].value)) {
      case VRML.SFBool:
        return elements[0].checked;
      case VRML.SFString:
        return elements[0].value;
      case VRML.SFInt32:
        return parseInt(elements[0].value);
      case VRML.SFFloat:
        return parseFloat(elements[0].value);
      case VRML.SFVec2f:
        return new WbVector2(parseFloat(elements[0].value), parseFloat(elements[1].value));
      case VRML.SFVec3f:
      case VRML.SFColor:
        return new WbVector3(parseFloat(elements[0].value), parseFloat(elements[1].value), parseFloat(elements[2].value));
      case VRML.SFRotation:
        return new WbVector4(parseFloat(elements[0].value), parseFloat(elements[1].value), parseFloat(elements[2].value), parseFloat(elements[3].value));
      case VRML.SFNode:
        return; // TODO: just bypass for now
      default:
        throw new Error('Unknown form in getValuesFromForm.');
    }
  }
}
