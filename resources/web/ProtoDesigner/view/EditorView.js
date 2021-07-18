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
    this.editorElement.innerHTML = ''; // remove current
    const ref = event.target.getAttribute('ref');
    console.log('Clicked item ref=' + ref);
    const parameter = this.proto.parameters.get(ref);
    // adapt selection
    this.editorElement.innerHTML = '<p><i>selection</i> : ' + parameter.name + '</p>';
    // adapt editor
    this.populateEditor(parameter);
  };

  populateEditor(parameter) {
    /*
    let div = document.createElement('div');
    div.style.textAlign = 'center';

    if (parameter.type === VRML_TYPE.SF_BOOL)
      div.appendChild(this._createInput('checkbox', parameter.value));
    else if (parameter.type === VRML_TYPE.SF_STRING) {
      // TODO: if parameterName is 'texture', add 'select' button
      div.appendChild(this._createInput('text', parameter.value));
    } else if (parameter.type === VRML_TYPE.SF_INT32)
      div.appendChild(this._createInput('number', '1', parameter.value));
    else if (parameter.type === VRML_TYPE.SF_FLOAT)
      div.appendChild(this._createInput('number', '0.1', parameter.value));
    else if (parameter.type === VRML_TYPE.SF_VECT2F) {
      let form = document.createElement('form');
      form.setAttribute('id', 'inputForm');
      form.setAttribute('parameterRef', id.toString());

      let input1 = document.createElement('input');
      input1.setAttribute('type', 'number');
      input1.setAttribute('value', '1');
      input1.setAttribute('id', '111');
      let input2 = document.createElement('input');
      input2.setAttribute('type', 'number');
      input2.setAttribute('value', '2');
      input1.setAttribute('id', '222');

      input1.addEventListener('input', this._updateValue.bind(this));
      input2.addEventListener('input', this._updateValue.bind(this));

      form.appendChild(input1);
      form.appendChild(input2);

      this._editorElement.appendChild(form);
      return;


      /*
      div.appendChild(document.createTextNode('x '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.x));
      div.appendChild(document.createTextNode('\u00A0\u00A0\u00A0\u00A0y '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.y));
      */

    /*
    } else if (parameterType === VRML_TYPE.SF_VECT3F) {
      div.appendChild(document.createTextNode('x '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.x));
      div.appendChild(document.createTextNode('\u00A0\u00A0\u00A0\u00A0y '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.y));
      div.appendChild(document.createTextNode('\u00A0\u00A0\u00A0\u00A0z '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.z));
    } else if (parameterType === VRML_TYPE.SF_COLOR) {
      div.appendChild(document.createTextNode('r '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.r));
      div.appendChild(document.createTextNode('\u00A0\u00A0\u00A0\u00A0g '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.g));
      div.appendChild(document.createTextNode('\u00A0\u00A0\u00A0\u00A0b '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.b));
    } else if (parameterType === VRML_TYPE.SF_ROTATION) {
      div.appendChild(document.createTextNode('x '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.x));
      div.appendChild(document.createTextNode('\u00A0\u00A0\u00A0\u00A0y '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.y));
      div.appendChild(document.createTextNode('\u00A0\u00A0\u00A0\u00A0z '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.z));
      div.appendChild(document.createTextNode('\u00A0\u00A0\u00A0\u00A0a '));
      div.appendChild(this._createInput('number', '0.1', parameterValue.w));
    }

    this._editorElement.appendChild(div);
    */
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
