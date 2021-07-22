'use strict';

import WbWorld from '../../wwi/nodes/WbWorld.js';

import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';
import {VRML} from '../classes/FieldModel.js';

import Proto from '../classes/Proto.js';

export default class EditorView { // eslint-disable-line no-unused-vars
  constructor(element, renderer, view, designer) {
    // setup parameter list view
    this.element = element;
    this.cleanupDiv('No loaded PROTO');
    this.renderer = renderer;
    this.view = view;
    this.designer = designer;
    // setup parameter editor view
    this.editorElement = document.getElementById('parameter-editor');
    if (typeof this.editorElement === 'undefined')
      throw new Error('Error, parameter-editor component not found.');

    // adapt selection
    const p = document.createElement('p');
    p.setAttribute('id', 'selection');
    p.innerHTML = 'selection: <b>none</b>';
    this.editorElement.appendChild(p);
    // this.editorElement.innerHTML = '<p><i>selection</i> : none</p>';
    this.forms = new Map();

    this.setupInputForm(VRML.SFBool, 1, 'checkbox', false);
    this.setupInputForm(VRML.SFString, 1, 'text', 'none');
    this.setupInputForm(VRML.SFInt32, 1, 'number', '0', '1');
    this.setupInputForm(VRML.SFFloat, 1, 'number', '0', '0.1');
    this.setupInputForm(VRML.SFVec2f, 2, 'number', '0', '0.1');
    this.setupInputForm(VRML.SFVec3f, 3, 'number', '0', '0.1');
    this.setupInputForm(VRML.SFColor, 3, 'number', '0', '0.1');
    this.setupInputForm(VRML.SFRotation, 4, 'number', '0', '0.1');
    this.setupInputForm(VRML.SFNode, 1, 'checkbox', false);
  };

  refreshParameters() {
    this.element.innerHTML = ''; // clean HTML

    if (this.designer.activeProtos.length === 0)
      this.cleanupDiv('No loaded PROTO');
    else {
      const proto = this.designer.activeProtos[0];
      this.proto = proto; // remove this dependency if possible
      this.populateDiv(proto);
    }
  };

  populateDiv(proto, depth = 0, parent) {
    console.log('DEPTH ' + depth, proto);
    // add PROTO name label
    let nameLabel = document.createElement('p');
    nameLabel.innerHTML = '<span class="proto-name-label">' + proto.protoName + '</span>';

    if (typeof parent !== 'undefined')
      parent.appendChild(nameLabel);
    else
      this.element.appendChild(nameLabel);

    // display parameters
    let ul = document.createElement('ul');
    ul.setAttribute('class', 'designer-list');

    if (typeof parent !== 'undefined')
      parent.appendChild(ul);
    else
      this.element.appendChild(ul);

    for (const [key, parameter] of proto.parameters.entries()) {
      console.log(parameter.name)
      let li = document.createElement('li');
      li.innerText = parameter.name;
      li.setAttribute('class', 'item li-border li');
      li.setAttribute('ref', key);
      li.addEventListener('click', () => this.itemSelector(event));
      ul.appendChild(li);
      if (typeof parameter.linkedProto !== 'undefined')
        this.populateDiv(parameter.linkedProto, depth + 1, li);
    }
  };

  /*
  populateDiv() {
    this.element.innerHTML = '';

    // add PROTO name label
    let nameLabel = document.createElement('p');
    nameLabel.innerHTML = '<span class="proto-name-label">' + this.proto.protoName + '</span>';
    this.element.appendChild(nameLabel);

    // display parameters
    let ol = document.createElement('ol');
    ol.setAttribute('class', 'designer-list');
    this.element.appendChild(ol);

    for (const [key, value] of this.proto.parameters.entries()) {
      let li = document.createElement('li');
      li.innerText = value.name;
      li.setAttribute('class', 'item li-border li');
      li.setAttribute('ref', key);
      li.addEventListener('click', () => this.itemSelector(event));
      ol.appendChild(li);
    }
  };
  */

  itemSelector(event) {
    const ref = parseInt(event.target.getAttribute('ref'));
    console.log('Clicked item ref=' + ref);
    // adapt editor
    this.populateEditor(ref);
  };

  getForm(type) {
    const forms = document.getElementsByTagName('form');
    for (let form of forms) {
      if (parseInt(form.id) === type)
        return form;
    }
  };

  populateEditor(ref) {
    const parameter = this.proto.parameters.get(ref);

    // adapt selection
    const p = document.getElementById('selection');
    p.innerHTML = 'selection: <b>' + parameter.name + '</b>';

    // change form visibility
    for (const [key, value] of this.forms) {
      if (key === parameter.type)
        value.style.display = 'block';
      else
        value.style.display = 'none';
    }

    // show current value
    const form = this.forms.get(parameter.type);
    const elements = form.elements;
    for (let i = 0; i < elements.length; ++i) {
      if (elements[i].type === 'number' || elements[i].type === 'text' || elements[i].type === 'checkbox') {
        switch (parseInt(parameter.type)) {
          case VRML.SFBool:
            elements[i].checked = parameter.value;
            break;
          case VRML.SFString:
          case VRML.SFInt32:
          case VRML.SFFloat:
            elements[i].value = parameter.value;
            break;
          case VRML.SFVec2f:
            if (elements[i].getAttribute('variable') === '0')
              elements[i].value = parameter.value.x;
            else if (elements[i].getAttribute('variable') === '1')
              elements[i].value = parameter.value.y;
            else
              throw new Error('SFVec2f form should not have more than 2 inputs.');
            break;
          case VRML.SFVec3f:
          case VRML.SFColor:
            if (elements[i].getAttribute('variable') === '0')
              elements[i].value = parameter.value.x;
            else if (elements[i].getAttribute('variable') === '1')
              elements[i].value = parameter.value.y;
            else if (elements[i].getAttribute('variable') === '2')
              elements[i].value = parameter.value.z;
            else
              throw new Error('SFVec3f/SFColor forms should not have more than 3 inputs.');
            break;
          case VRML.SFRotation:
            if (elements[i].getAttribute('variable') === '0')
              elements[i].value = parameter.value.x;
            else if (elements[i].getAttribute('variable') === '1')
              elements[i].value = parameter.value.y;
            else if (elements[i].getAttribute('variable') === '2')
              elements[i].value = parameter.value.z;
            else if (elements[i].getAttribute('variable') === '3')
              elements[i].value = parameter.value.w;
            else
              throw new Error('SFRotation form should not have more than 4 inputs.');
            break;
          case VRML.SFNode:
            break;  // TODO: just bypass for now
          default:
            throw new Error('Cannot populate editor because parameterType \'' + parameter.type + '\' is unknown.');
        }
      }
    }

    // update parameter reference
    form.setAttribute('parameterReference', ref);
  };

  setupInputForm(id, nbInputs, type, defaultValue, step) {
    let form = document.createElement('form');
    form.setAttribute('onsubmit', 'return false;');
    form.setAttribute('id', id);
    form.setAttribute('parameterReference', ''); // parameter reference

    for (let i = 0; i < nbInputs; ++i) {
      let input = document.createElement('input');
      input.setAttribute('variable', i); // tracks which input it corresponds to: 0 -> x, 1 -> y, etc
      input.setAttribute('type', type);
      if (type === 'checkbox')
        input.checked = defaultValue;
      else
        input.setAttribute('value', defaultValue);

      if (id === VRML.SFColor) {
        input.setAttribute('max', '1');
        input.setAttribute('min', '0');
      }

      if (type === 'number')
        input.setAttribute('step', step);

      input.addEventListener('input', this.updateValue.bind(this));
      form.appendChild(input);
    }

    form.style.display = 'none'; // make invisible by default
    this.forms.set(id, form); // add to map for fast retrieval
    this.editorElement.appendChild(form);
  };

  updateValue(e) {
    console.log('updateValue');
    // keep proto.parameter value up to date
    const parameterRef = e.target.form.attributes['parameterReference'].value;
    const parameter = this.proto.parameters.get(parseInt(parameterRef));

    if (parameter.type === VRML.SFNode) { // TMP
      const nodeRefs = parameter.nodeRefs;
      const node = WbWorld.instance.nodes.get(nodeRefs[0]);
      const nodeId = parseInt(node.id.slice(1));
      this.designer.loadProto('../wwi/Protos/ProtoSphere.proto', nodeId, parameter);
      return;
    };

    const newValue = this.getValuesFromForm(e.target.form);
    if (typeof parameter.value === typeof newValue)
      parameter.value = newValue;
    else
      throw new Error('Overwriting value of type ' + typeof parameter.value + ' with value of type ' + typeof newValue);

    if (parameter.isTemplateRegenerator) {
      console.log('Regeneration triggered by parameter ' + parameter.name);
      this.view.x3dScene.destroyWorld();
      this.proto.regenerate();
      this.view.x3dScene.loadWorldFileRaw(this.proto.x3d, this.view.finalizeWorld);
      return;
    }

    const nodeRefs = parameter.nodeRefs;
    const refNames = parameter.refNames;

    if (nodeRefs.length === 0 || (nodeRefs.length !== refNames.length))
      console.warn('No nodeRefs links are present for the selected paramter. Was it supposed to?');

    for (let i = 0; i < nodeRefs.length; ++i) {
      if (typeof nodeRefs[i] !== 'undefined' && typeof refNames[i] !== 'undefined') {
        const node = WbWorld.instance.nodes.get(nodeRefs[i]);
        node.setParameter(refNames[i], parameter.value);
      }
    }

    this.view.x3dScene.render();
  };

  getValuesFromForm(form) {
    if (typeof form === 'undefined')
      throw new Error('Cannot get values from unknown form');

    const elements = form.elements;
    switch (parseInt(form.attributes['id'].value)) {
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
