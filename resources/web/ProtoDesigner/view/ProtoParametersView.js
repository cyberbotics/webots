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
      this._populateDiv(protoModel);
  }

  _populateDiv(protoModel) {
    this._element.innerHTML = '';
    // add PROTO name label
    let nameLabel = document.createElement('p');
    nameLabel.innerHTML = '<span class="proto-name-label">' + protoModel.protoName + '</span>';
    this._element.appendChild(nameLabel);

    // display parameters
    const parameters = protoModel.parameters;

    if (parameters.length === 0) {
      let text = document.createElement('p');
      text.innerHTML = '<i>No parameters<i>';
      this._element.appendChild(text);
      return;
    }

    let ol = document.createElement('ol');
    ol.setAttribute('class', 'designer-list');
    this._element.appendChild(ol);

    parameters.forEach((item) => {
      let li = document.createElement('li');
      li.innerText = item.name;
      li.setAttribute('class', 'item li-border');
      li.setAttribute('onclick', 'itemSelector($(item.id))');
      ol.appendChild(li);
    });
  };

  itemSelector(id) {
    console.log(id);
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
