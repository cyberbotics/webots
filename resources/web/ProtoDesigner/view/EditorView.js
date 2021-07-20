'use strict';

import WbWorld from '../../wwi/nodes/WbWorld.js';

import WbSFBool from '../../wwi/nodes/utils/WbSFBool.js';
import WbSFDouble from '../../wwi/nodes/utils/WbSFDouble.js';
import WbSFInt32 from '../../wwi/nodes/utils/WbSFInt32.js';
import WbSFString from '../../wwi/nodes/utils/WbSFString.js';
import WbSFColor from '../../wwi/nodes/utils/WbSFColor.js';
import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';
import {VRML} from '../classes/FieldModel.js';

export default class EditorView { // eslint-disable-line no-unused-vars
  constructor(element, renderer, view) {
    // setup parameter list view
    this.element = element;
    this.cleanupDiv('No loaded PROTO');
    this.renderer = renderer;
    this.view = view;
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
  }

  showParameters(proto) {
    if (proto === null)
      this.cleanupDiv('No loaded PROTO');
    else {
      this.proto = proto;
      this.populateDiv();
    }
  }

  populateDiv() {
    this.element.innerHTML = '';
    // add PROTO name label
    let nameLabel = document.createElement('p');
    nameLabel.innerHTML = '<span class="proto-name-label">' + this.proto.protoName + '</span>';
    this.element.appendChild(nameLabel);

    // display parameters
    /*
    const parameters = this._protoModel.parameters;

    if (parameters.length === 0) {
      let text = document.createElement('p');
      text.innerHTML = '<i>No parameters<i>';
      this.element.appendChild(text);
      return;
    }
    */

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
  }

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
            elements[i].checked = parameter.value.value;
            break;
          case VRML.SFString:
          case VRML.SFInt32:
          case VRML.SFFloat:
            elements[i].value = parameter.value.value;
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
              elements[i].value = parameter.type === VRML.SFVec3f ? parameter.value.x : parameter.value.r;
            else if (elements[i].getAttribute('variable') === '1')
              elements[i].value = parameter.type === VRML.SFVec3f ? parameter.value.y : parameter.value.g;
            else if (elements[i].getAttribute('variable') === '2')
              elements[i].value = parameter.type === VRML.SFVec3f ? parameter.value.z : parameter.value.b;
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
          default:
            throw new Error('Cannot populate editor because parameterType \'' + parameter.type + '\' is unknown.');
        }
      }
    }

    // update parameter reference
    form.setAttribute('parameterReference', ref);
  }

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

      if (type === 'number')
        input.setAttribute('step', step);

      input.addEventListener('input', this.updateValue.bind(this));
      form.appendChild(input);
    }

    form.style.display = 'none'; // make invisible by default
    this.forms.set(id, form); // add to map for fast retrieval
    this.editorElement.appendChild(form);
  }

  _createInput(type, value, step) {
    let input = document.createElement('input');
    input.addEventListener('input', this._updateValue.bind(this));

    input.setAttribute('type', type);
    if (type === 'checkbox' && typeof value === 'boolean')
      input.checked = value;
    if ((type === 'number' || type === 'text') && typeof value !== 'undefined')
      input.setAttribute('value', value);

    if (typeof step !== 'undefined')
      input.setAttribute('step', step);
    return input;
  };

  updateValue(e) {
    console.log('updateValue');
    // keep proto.parameter value up to date
    const parameterRef = e.target.form.attributes['parameterReference'].value;
    const parameter = this.proto.parameters.get(parseInt(parameterRef));

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
  }

  getValuesFromForm(form) {
    if (typeof form === 'undefined')
      throw new Error('Cannot get values from unknown form');

    const elements = form.elements;
    switch (parseInt(form.attributes['id'].value)) {
      case VRML.SFBool:
        return new WbSFBool(elements[0].value === 'true');
      case VRML.SFString:
        return new WbSFString(elements[0].value);
      case VRML.SFInt32:
        return new WbSFInt32(parseInt(elements[0].value));
      case VRML.SFFloat:
        return new WbSFDouble(parseFloat(elements[0].value));
      case VRML.SFVec2f:
        return new WbVector2(parseFloat(elements[0].value), parseFloat(elements[1].value));
      case VRML.SFVec3f:
        return new WbVector3(parseFloat(elements[0].value), parseFloat(elements[1].value), parseFloat(elements[2].value));
      case VRML.SFColor:
        return new WbSFColor(parseFloat(elements[0].value), parseFloat(elements[1].value), parseFloat(elements[2].value));
      case VRML.SFRotation:
        return new WbVector4(parseFloat(elements[0].value), parseFloat(elements[1].value), parseFloat(elements[2].value), parseFloat(elements[3].value));
      default:
        throw new Error('Unknown form in getValuesFromForm.');
    }
  }

  _populateDivRaw(parameters) {
    /*
    this._element.innerHTML = '';

    // TODO: use methods instead... document.createElement, document.createTextNode, etc
    let model = {};
    model.name = 'label';

    let nameLabel = document.createElement('p');
    nameLabel.innerHTML = 'Name: <span class="part-name-label">' + model.name + '</span>';
    this._element.appendChild(nameLabel);

    let integerInput = document.createElement('p');
    integerInput.innerHTML = '<label>Integer Input: <input type="number" step="1" value="10"></label>';
    this._element.appendChild(integerInput);

    let floatInput = document.createElement('p');
    floatInput.innerHTML = '<label>Floating Input: <input type="number" step="0.01" value="3.14"></label>';
    this._element.appendChild(floatInput);

    let textInput = document.createElement('p');
    textInput.innerHTML = '<label>String Input: <input type="text" value="this is a string"></label>';
    this._element.appendChild(textInput);

    let boolInput = document.createElement('p');
    boolInput.innerHTML = '<label>Boolean Input: <input type="checkbox">TRUE</label>';
    this._element.appendChild(boolInput);
    */
  };

  cleanupDiv(text) {
    this.element.innerHTML = '<p><i>' + text + '</i></p>';
  }
}
