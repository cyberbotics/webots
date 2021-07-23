'use strict';

export default class PartBrowser {
  constructor(assetLibraryElement, assetLibrary) {
    this.assetLibraryElement = assetLibraryElement;
    this.assetLibrary = assetLibrary;
    this.partIconDivs = [];
  };

  dragStart(e) {
    console.log('Drag start: ' + e.target.getAttribute('proto'));
  };

  loadAssets() {
    this.assetLibrary.assets.forEach((asset) => {
      let div = document.createElement('div');
      // div.innerHTML = '<div class="part-icon" draggable="true" proto="' + asset.name + '" >' + '<img draggable="false" src="' + asset.icon + '" />' + '</div>';
      div.setAttribute('proto', asset.name);
      div.setAttribute('class', 'part-icon');

      let img = document.createElement('img');
      img.setAttribute('draggable', true);
      img.setAttribute('src', asset.icon);
      img.setAttribute('proto', asset.name);
      img.addEventListener('dragstart', this.dragStart);

      div.appendChild(img);

      this.partIconDivs.push(div.firstChild);
      this.assetLibraryElement.appendChild(div.firstChild);
    });
  };
};
