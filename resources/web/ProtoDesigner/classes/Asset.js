'use strict';

export default class Asset { // eslint-disable-line no-unused-vars
  constructor(name, assetData) {
    this.name = name;
    this.proto = assetData.proto;
    this.baseNode = assetData.baseNode;
    this.icon = assetData.icon;
  }
}
