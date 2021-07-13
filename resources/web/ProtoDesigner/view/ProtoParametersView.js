'use strict';

export default class ProtoParametersView { // eslint-disable-line no-unused-vars
  constructor(element) {
    this._element = element;
    this._cleanupDiv('No loaded PROTO');
  }

  showParameters(protoModel) {
    if (protoModel === null)
      this._cleanupDiv('No loaded PROTO');
    else
      this._populateDivRaw(protoModel);
  }

  _populateDiv(parameters) {
    this._element.innerHTML = '';
  };

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
    textInput.innerHTML = '<label>Floating Input: <input type="text" value="this is a string"></label>';
    this._element.appendChild(textInput);

    let boolInput = document.createElement('p');
    boolInput.innerHTML = '<label>Boolean Input: <input type="checkbox">TRUE</label>';
    this._element.appendChild(boolInput);
  };

  _cleanupDiv(text) {
    this._element.innerHTML = '<p><i>' + text + '</i></p>';
  }
}
