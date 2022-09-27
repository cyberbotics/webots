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
    if (typeof filter === 'undefined')
      return;

    console.log('Filtering assets except: ', filter);
    let compatibleSlots;
    if (filter.endsWith('+'))
      compatibleSlots = filter.slice(0, -1) + '-';
    else if (filter.endsWith('-'))
      compatibleSlots = filter.slice(0, -1) + '+';
    else
      compatibleSlots = filter;

    this.loadRemoveNode(modalElement, callback);

    for (const [key, asset] of this.assetLibrary.assets.entries()) {
      if (asset.slotType !== compatibleSlots)
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

  loadRemoveNode(modalElement, callback) {
    let div = document.createElement('div');
    let img = document.createElement('img');
    img.classList.add('proto-icon');
    img.setAttribute('draggable', false);
    img.setAttribute('src', './view/assets/remove_node.png');
    img.setAttribute('assetKey', 'remove');
    img.addEventListener('click', callback, false);
    div.appendChild(img);

    this.partIconDivs.push(div.firstChild);
    modalElement.appendChild(div.firstChild);
  }
};
