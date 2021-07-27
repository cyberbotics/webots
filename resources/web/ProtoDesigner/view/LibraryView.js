'use strict';

export default class LibraryView {
  constructor(libraryElement, assetLibrary) {
    this.libraryElement = libraryElement;
    this.assetLibrary = assetLibrary;
    this.partIconDivs = [];
  };

  /*
  dragStart(e) {
    console.log('Drag start: ' + e.target.getAttribute('assetKey'));
    const key = e.target.getAttribute('assetKey');
    e.dataTransfer.setData('text/plain', JSON.stringify(this.assetLibrary.assets.get(key)));
  };
  */

  loadAssets(modalElement, filter) {
    if (typeof modalElement === 'undefined')
      throw new Error('Cannot load assets because modal element is undefined.');

    // clear contents
    modalElement.innerHTML = '';
    this.partIconDivs = [];

    // populate
    console.log('Filtering assets except: ', filter);
    for (const [key, asset] of this.assetLibrary.assets.entries()) {
      if (typeof filter !== 'undefined' && asset.slotType !== filter)
        continue;

      let div = document.createElement('div');
      div.setAttribute('class', 'part-icon');
      let img = document.createElement('img');
      img.setAttribute('draggable', false);
      img.setAttribute('src', asset.icon);
      img.setAttribute('assetKey', key);
      // img.addEventListener('dragstart', this.dragStart.bind(this), false);
      img.setAttribute('title', asset.name);
      div.appendChild(img);

      this.partIconDivs.push(div.firstChild);
      modalElement.appendChild(div.firstChild);
    }
  };
};
