'use strict';

export default class LibraryView {
  constructor(libraryElement, assetLibrary) {
    this.libraryElement = libraryElement;
    this.assetLibrary = assetLibrary;
    this.partIconDivs = [];
  };

  dragStart(e) {
    console.log('Drag start: ' + e.target.getAttribute('assetKey'));
    const key = e.target.getAttribute('assetKey');
    e.dataTransfer.setData('text/plain', JSON.stringify(this.assetLibrary.assets.get(key)));
  };

  loadAssets() {
    for (const [key, asset] of this.assetLibrary.assets.entries()) {
      let div = document.createElement('div');
      div.setAttribute('class', 'part-icon');
      let img = document.createElement('img');
      img.setAttribute('draggable', true);
      img.setAttribute('src', asset.icon);
      img.setAttribute('assetKey', key);
      img.addEventListener('dragstart', this.dragStart.bind(this), false);

      div.appendChild(img);

      this.partIconDivs.push(div.firstChild);
      this.libraryElement.appendChild(div.firstChild);
    }
  };
};
