'use strict';

export default class PartBrowser {
  constructor(assetLibraryElement, assetLibrary) {
    this.assetLibraryElement = assetLibraryElement;
    this.assetLibrary = assetLibrary;
    this.partIconDivs = [];
  }

  loadAssets() {
    this.assetLibrary.assets.forEach((asset) => {
      let div = document.createElement('div');
      div.innerHTML = '' +
        '<div class="part-icon" draggable="true" ondragstart="dragStart(event)" part="' + asset.name + '" >' +
          '<img draggable="false" src="' + asset.icon + '" />' +
        '</div>';
      this.partIconDivs.push(div.firstChild);
      this.assetLibraryElement.appendChild(div.firstChild);
    });
  }

  update(robot) {
    /*
    var availableSlotTypes = robot.getAvailableSlotTypes();
    for (let d = 0; d < this.partIconDivs.length; d++) {
      var div = this.partIconDivs[d];
      if (availableSlotTypes.length === 0) {
        if (div.getAttribute('slotType'))
          div.classList.add('hidden');
        else
          div.classList.remove('hidden');
      } else {
        div.classList.remove('hidden');
        if (div.getAttribute('slotType')) {
          if (availableSlotTypes.indexOf(div.getAttribute('slotType')) > -1) {
            div.classList.remove('part-icon-disabled');
            div.draggable = true;
          } else {
            div.classList.add('part-icon-disabled');
            div.draggable = false;
          }
        } else
          div.classList.add('hidden');
      }
    }
    */
  }
}
