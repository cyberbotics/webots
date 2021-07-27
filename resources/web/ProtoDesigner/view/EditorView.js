'use strict';

import WbWorld from '../../wwi/nodes/WbWorld.js';

import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';

import Proto from '../classes/Proto.js';
import {VRML} from '../classes/utility/utility.js';

export default class EditorView { // eslint-disable-line no-unused-vars
  constructor(element, view, designer, library) {
    // setup parameter list view
    this.element = element;
    this.view = view;
    this.designer = designer;
    this.library = library;

    this.cleanupDiv('No loaded PROTO');

    this.parameter = undefined; // currently selected parameter in the UI
    this.proto = undefined; // currently referenced proto (based on the active parameter)

    this.forms = new Map();
  };

  refreshParameters() {
    this.element.innerHTML = ''; // clean HTML
    console.log('Cleared EditorView innnerHTML');
    this.setupModalWindow();

    if (this.designer.activeProtos.size === 0)
      this.cleanupDiv('No loaded PROTO');
    else {
      const proto = this.designer.activeProtos.get(0);
      this.populateDiv(proto);
    }
  };

  populateDiv(proto, depth = 0, parent) {
    console.log('DEPTH ' + depth, proto);
    // add PROTO name label
    if (depth === 0) {
      let nameLabel = document.createElement('p');
      nameLabel.innerHTML = '<span class="proto-name-label">' + proto.protoName + '</span>';

      this.element.appendChild(nameLabel);
    }
    // display parameters
    for (const [key, parameter] of proto.parameters.entries()) {
      this.setupParameter(proto, parameter, key);
      if (typeof proto.linkedProto !== 'undefined')
        this.populateDiv(proto.linkedProto, depth + 1);
    }
  };

  setupParameter(proto, parameter, parameterId) {
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
        input.checked = value.value;
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
        button.innerText = 'NULL';
        button.addEventListener('click', () => this.itemSelector(event));
        div.appendChild(button);
        break;
      default:
        throw new Error('Cannot setup div because parameter type \'' + parameter.type + '\' is unknown.');
    }

    form.appendChild(div);
    this.element.appendChild(form);
  };

  itemSelector(e) {
    const protoId = parseInt(e.target.form.attributes['protoId'].value);
    const parameterId = e.target.form.attributes['parameterId'].value;
    console.log('Clicked item parameterId = ' + parameterId + ' (protoId = ' + protoId + ')');

    this.proto = this.designer.activeProtos.get(protoId);
    this.parameter = this.proto.parameters.get(parameterId);

    // determine filtering condition
    const nodeRefs = this.parameter.nodeRefs;
    let filter;
    if (typeof nodeRefs !== 'undefined') {
      const slot = WbWorld.instance.nodes.get(nodeRefs[0]);
      filter = slot.type;
    }

    // button position
    const position = e.target.getBoundingClientRect();
    // get the modal
    let modal = document.getElementById('modalWindow');
    modal.style.display = 'block';
    modal.style.top = position.y + 'px';
    modal.style.left = (position.x + position.width) + 'px';
    this.library.loadAssets(modal, filter);
  };

  setupModalWindow() {
    // create modal window
    let div = document.createElement('div');
    div.className = 'modal';
    div.setAttribute('id', 'modalWindow');
    let button = document.createElement('button');
    button.classList.add('modal-close-button');
    button.innerText = 'X';
    button.addEventListener('click', () => this.closeModalWindow(event));
    div.appendChild(button);

    this.element.appendChild(div);
  }

  closeModalWindow() {
    let modal = document.getElementById('modalWindow');
    modal.style.display = 'none';
  }

  insertNode(protoUrl) {
    if (this.parameter.type === VRML.SFNode) {
      // get parent
      const nodeRefs = this.parameter.nodeRefs;
      const node = WbWorld.instance.nodes.get(nodeRefs[0]);
      const nodeId = parseInt(node.id.slice(1));

      this.designer.loadProto(protoUrl, nodeId, this.parameter);
    };
  };

  updateValue(e) {
    const protoId = parseInt(e.target.form.attributes['protoId'].value);
    const parameterId = e.target.form.attributes['parameterId'].value;
    console.log('Clicked item parameterId = ' + parameterId + ' (protoId = ' + protoId + ')');

    this.proto = this.designer.activeProtos.get(protoId);
    this.parameter = this.proto.parameters.get(parameterId);

    const newValue = this.getValuesFromForm(e.target.form);
    if (typeof this.parameter.value === typeof newValue)
      this.parameter.value = newValue;
    else
      throw new Error('Overwriting value of type ' + typeof this.parameter.value + ' with value of type ' + typeof newValue);

    if (this.parameter.isTemplateRegenerator) {
      console.log('Regeneration triggered by parameter ' + this.parameter.name);
      for (const key of WbWorld.instance.nodes.keys()) {
        if (parseInt(key.slice(1)) < -4)
          this.view.x3dScene._deleteObject(key.slice(1));
      }
      console.log(WbWorld.instance.nodes)
      this.proto.regenerate();
      this.view.x3dScene._loadObject(this.proto.x3d, '-4');
      // this.view.x3dScene.loadWorldFileRaw(this.proto.x3d, this.view.finalizeWorld);
      this.view.x3dScene.render();
      return;
    }

    const nodeRefs = this.parameter.nodeRefs;
    const refNames = this.parameter.refNames;

    if (nodeRefs.length === 0 || (nodeRefs.length !== refNames.length))
      console.warn('No nodeRefs links are present for the selected paramter. Was it supposed to?');

    for (let i = 0; i < nodeRefs.length; ++i) {
      if (typeof nodeRefs[i] !== 'undefined' && typeof refNames[i] !== 'undefined') {
        const node = WbWorld.instance.nodes.get(nodeRefs[i]);
        console.log('> setting parameter \'' + refNames[i] + '\' to ', this.parameter.value);
        node.setParameter(refNames[i], this.parameter.value);
        // propagate to USE nodes, if any
        for (let j = 0; j < node.useList.length; ++j) {
          console.log('>> setting (through DEF) parameter \'' + refNames[i] + '\' to ', this.parameter.value);
          const useNode = WbWorld.instance.nodes.get(node.useList[j]);
          useNode.setParameter(refNames[i], this.parameter.value);
        }
      }
    }

    this.view.x3dScene.render();
  };

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
  };

  cleanupDiv(text) {
    this.element.innerHTML = '<p><i>' + text + '</i></p>';
  };
}
