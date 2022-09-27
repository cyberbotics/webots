'use strict';

import Asset from './Asset.js';
import Observable from './Observable.js';

export default class AssetLibrary extends Observable { // eslint-disable-line no-unused-vars
  constructor() {
    super();
    this.assets = new Map();
    fetch('./library/library.json')
      .then(response => response.text())
      .then((txt) => this._loadAssets(JSON.parse(txt)));
  }

  getAssetByName(name) {
    this.assets.get(name);
  };

  _loadAssets(assetsData) {
    Object.keys(assetsData).forEach((assetName) => {
      console.log('Loading library asset: ' + assetName)
      let assetData = assetsData[assetName];
      let asset = new Asset(assetName, assetData);
      this.assets.set(assetName, asset);
    });
    this.notify('loaded', null);
  }
}
