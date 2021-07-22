'use strict';

export default class LibraryView { // eslint-disable-line no-unused-vars
  constructor(element) {
    this.element = element;
    this.cleanupDiv('Library is empty');
  };

  parseLibrary(library) {
    this.protoList = JSON.parse(library);
    for (let i = 0; i < this.protoList.length; ++i) {
      console.log(this.protoList[i])
    }
  }

  cleanupDiv(text) {
    this.element.innerHTML = '<p><i>' + text + '</i></p>';
  };
}
