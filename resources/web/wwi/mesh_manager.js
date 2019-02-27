'use strict';

class MeshManager { // eslint-disable-line no-unused-vars
  constructor() {
    if (!MeshManager.instance) {
      MeshManager.instance = this;
      this.geometries = [];
      this.materials = [];
    }
    return MeshManager.instance;
  }

  getGeometry(name) {
    return this.geometries[name];
  }

  getMaterial(name) {
    return this.materials[name];
  }

  reset() {
    this.geometries = [];
    this.materials = [];
  }

  addGeometry(name, data) {
    // TODO assert this.geometries[name] is empty
    this.geometries[name] = data;
  }

  addMaterial(name, data) {
    // TODO assert this.materials[name] is empty
    this.materials[name] = data;
  }
}
