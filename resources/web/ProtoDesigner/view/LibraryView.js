'use strict';

export default class LibraryView {
  constructor(libraryElement, assetLibrary) {
    this.libraryElement = libraryElement;
    this.assetLibrary = assetLibrary;
    this.partIconDivs = [];
  };

  getAsset(assetKey) {
    if (!this.assetLibrary.assets.has(assetKey))
      throw new Error('Asset ' + assetKey + ' does not exist in the library.');

    return this.assetLibrary.assets.get(assetKey);
  };

  loadAssets(modalElement, filter, callback, parameterRef) {
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
      let img = document.createElement('img');
      img.classList.add('proto-icon');
      img.setAttribute('draggable', false);
      img.setAttribute('src', asset.icon);
      img.setAttribute('assetKey', key);
      //img.addEventListener('dragstart', this.dragStart.bind(this), false);
      img.addEventListener('click', callback, false);
      img.setAttribute('title', asset.name);
      div.appendChild(img);

      this.partIconDivs.push(div.firstChild);
      modalElement.appendChild(div.firstChild);
    }
  };
};
