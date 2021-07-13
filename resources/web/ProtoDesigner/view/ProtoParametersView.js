'use strict';

export default class ProtoParametersView { // eslint-disable-line no-unused-vars
  constructor(element) {
    this._element = element;
    this._cleanupDiv('No parameters');
  }

  _showPart(part) {

  }

  _populateDiv(parameters) {

  }

  _cleanupDiv(text) {
    this._element.innerHTML = '<p><i>' + text + '</i></p>';
  }
}
