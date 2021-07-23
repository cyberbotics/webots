'use strict';

import Asset from './Asset.js';
import Observable from './Observable.js';

export default class AssetLibrary extends Observable { // eslint-disable-line no-unused-vars
  constructor() {
    super();
    this.assets = [];
    fetch('./library/library.json')
      .then(response => response.text())
      .then((txt) => this._loadAssets(JSON.parse(txt)));
  }

  getAssetByName(assetName) {
    for (let a = 0; a < this.assets.length; a++) {
      if (this.assets[a].name === assetName)
        return this.assets[a];
    }
    return undefined;
  }

  _loadAssets(assetsData) {
    Object.keys(assetsData).forEach((assetName) => {
      var assetData = assetsData[assetName];
      var asset = new Asset(assetName, assetData);
      this.assets.push(asset);
    });
    this.notify('loaded', null);
  }
}
