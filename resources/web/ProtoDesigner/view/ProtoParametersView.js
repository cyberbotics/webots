'use strict';

import {FIELD_TYPES} from '../../wwi/WbFieldModel.js';

export default class ProtoParametersView { // eslint-disable-line no-unused-vars
  constructor(element) {
    // setup parameter list view
    this._element = element;
    this._cleanupDiv('No loaded PROTO');
    // setup parameter editor view
    this._editorElement = document.getElementById('parameter-editor');
    if (typeof this._editorElement === 'undefined')
      throw new Error('Error, parameter-editor component not found.');
  }

  showParameters(protoModel) {
    if (protoModel === null)
      this._cleanupDiv('No loaded PROTO');
    else {
      this._protoModel = protoModel;
      this._populateDiv();
    }
  }

  _populateDiv() {
    this._element.innerHTML = '';
    // add PROTO name label
    let nameLabel = document.createElement('p');
    nameLabel.innerHTML = '<span class="proto-name-label">' + this._protoModel.protoName + '</span>';
    this._element.appendChild(nameLabel);

    // display parameters
    const parameters = this._protoModel.parameters;

    if (parameters.length === 0) {
      let text = document.createElement('p');
      text.innerHTML = '<i>No parameters<i>';
      this._element.appendChild(text);
      return;
    }

    let ol = document.createElement('ol');
    ol.setAttribute('class', 'designer-list');
    this._element.appendChild(ol);

    for (let i = 0; i < parameters.length; ++i) {
      let li = document.createElement('li');
      li.innerText = parameters[i].name;
      li.setAttribute('class', 'item li-border li');
      li.setAttribute('id', i); // id of the list items is their position in the list, not the parameter id
      li.addEventListener('click', () => this.itemSelector(event));//this.itemSelector);
      ol.appendChild(li);
    }
  };

  itemSelector(event) {
    console.log('Clicked item id=' + event.target.id);
    const parameterName = this._protoModel.parameters[event.target.id].name;
    const parameterType = this._protoModel.parameters[event.target.id].type;
    const parameterValue = this._protoModel.parameters[event.target.id].value;

    this._editorElement.innerHTML = '<p><i><center>selection</i> : ' + parameterName + '</center></p>';
    // TODO: remove current

    // adapt editor
    this._populateEditor(parameterType, parameterValue);
  };

  _populateEditor(parameterType, parameterValue) {
    if (parameterType === FIELD_TYPES.SF_STRING) {
      let textInput = document.createElement('p');
      textInput.innerHTML = '<input type="text" value=' + parameterValue + '>';
      this._editorElement.appendChild(textInput);
    }
  }

  _populateDivRaw(parameters) {
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
  };

  _cleanupDiv(text) {
    this._element.innerHTML = '<p><i>' + text + '</i></p>';
  }
}
