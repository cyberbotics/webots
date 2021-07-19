'use strict';

import WbWorld from '../../wwi/nodes/WbWorld.js';

import {VRML_TYPE} from '../classes/FieldModel.js';

export default class EditorView { // eslint-disable-line no-unused-vars
  constructor(element, renderer) {
    // setup parameter list view
    this.element = element;
    this.cleanupDiv('No loaded PROTO');
    this.renderer = renderer;
    // setup parameter editor view
    this.editorElement = document.getElementById('parameter-editor');
    if (typeof this.editorElement === 'undefined')
      throw new Error('Error, parameter-editor component not found.');

    // adapt selection
    // this.editorElement.innerHTML = '<p><i>selection</i> : none</p>';
    this.forms = new Map();

    this.setupForms(VRML_TYPE.SF_INT32, 1, 'number', '0', '1', '');
    this.setupForms(VRML_TYPE.SF_FLOAT, 1, 'number', '0', '0.01', '');
    this.setupForms(VRML_TYPE.SF_VECT2F, 2, 'number', '0', '0.01', '');
    this.setupForms(VRML_TYPE.SF_VECT3F, 3, 'number', '0', '0.01', '');
    this.setupForms(VRML_TYPE.SF_COLOR, 3, 'number', '0', '0.01', '');
    this.setupForms(VRML_TYPE.SF_ROTATION, 4, 'number', '0', '0.01', '');
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
    // this.editorElement.innerHTML = '<p><i>selection</i> : ' + parameter.name + '</p>';

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
      if (elements[i].type === 'number') {
        switch (parseInt(parameter.type)) {
          case VRML_TYPE.SF_INT32:
          case VRML_TYPE.SF_FLOAT:
            elements[i].value = parameter.value.value;
            break;
          case VRML_TYPE.SF_VECT2F:
            if (elements[i].getAttribute('variable') === '0')
              elements[i].value = parameter.value.x;
            else if (elements[i].getAttribute('variable') === '1')
              elements[i].value = parameter.value.y;
            else
              throw new Error('SF_VECT2F form should not have more than 2 inputs.');
            break;
          case VRML_TYPE.SF_VECT3F:
          case VRML_TYPE.SF_COLOR:
            if (elements[i].getAttribute('variable') === '0')
              elements[i].value = parameter.type === VRML_TYPE.SF_VECT3F ? parameter.value.x : parameter.value.r;
            else if (elements[i].getAttribute('variable') === '1')
              elements[i].value = parameter.type === VRML_TYPE.SF_VECT3F ? parameter.value.y : parameter.value.g;
            else if (elements[i].getAttribute('variable') === '2')
              elements[i].value = parameter.type === VRML_TYPE.SF_VECT3F ? parameter.value.z : parameter.value.b;
            else
              throw new Error('SF_VECT3F/SF_COLOR forms should not have more than 3 inputs.');
            break;
          case VRML_TYPE.SF_ROTATION:
            if (elements[i].getAttribute('variable') === '0')
              elements[i].value = parameter.value.x;
            else if (elements[i].getAttribute('variable') === '1')
              elements[i].value = parameter.value.y;
            else if (elements[i].getAttribute('variable') === '2')
              elements[i].value = parameter.value.z;
            else if (elements[i].getAttribute('variable') === '3')
              elements[i].value = parameter.value.w;
            else
              throw new Error('SF_ROTATION form should not have more than 4 inputs.');
            break;
          default:
            throw new Error('Cannot populate editor because parameterType \'' + parameter.type + '\' is unknown.');
        }
      }
    }
  }

  setupForms(id, nbInputs, type, defaultValue, step, ref) {
    let form = document.createElement('form');
    form.setAttribute('id', id);
    form.setAttribute('ref', ref); // parameter reference
    for (let i = 0; i < nbInputs; ++i) {
      let input = document.createElement('input');
      input.setAttribute('variable', i);
      input.setAttribute('type', type);
      input.setAttribute('step', step);
      input.setAttribute('value', defaultValue);
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
    if (type === 'checkbox' && typeof value !== 'undefined')
      input.checked = value;
    if ((type === 'number' || type === 'text') && typeof value !== 'undefined')
      input.setAttribute('value', value);

    if (typeof step !== 'undefined')
      input.setAttribute('step', step);
    return input;
  };

  updateValue(e) {
    console.log('updateValue');
  }

  _updateValue(e) {
    // get parent form of the input
    let vals = [];
    const elements = e.target.form.elements;
    for (let i = 0; i < elements.length; ++i) {
      if (elements[i].type === 'number') {
        vals.push(elements[i].value);
        console.log(i + '  >  ' + elements[i].value + ' :: ');
      }
    }

    const ref = parseInt(e.target.form.attributes['parameterRef'].value);
    console.log('ref: ' + ref)

    const nodeid = this._protoModel.parameters[ref];
    console.log(nodeid);
    console.log(e.target.type);
    console.log(WbWorld.instance.nodes);
    const n = WbWorld.instance.nodes.get('n-6'); // nodeid should allow me to get this n-6

    // change parameter
    //this._protoModel.parameters[..something..].value.x = vals[0];
    //this._protoModel.parameters[..something..].value.y = vals[1];
    // change node
    n.size.x = vals[0];
    n.size.y = vals[1];

    n.updateSize();
    this.renderer.render();
  };

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
