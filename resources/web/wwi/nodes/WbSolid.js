import WbTransform from './WbTransform.js';
import WbWorld from './WbWorld.js';

export default class WbSolid extends WbTransform {
  #name;
  constructor(id, translation, scale, rotation, name) {
    super(id, translation, scale, rotation);
    this.#name = name;
  }

  get name() {
    return this.#name;
  }

  clone(customID) {
    console.error('Trying to clone a solid');
  }

  createWrenObjects() {
    super.createWrenObjects(true);

    this.boundingObject?.createWrenObjects();
  }

  delete(isBoundingObject) {
    this.boundingObject?.delete(true);

    super.delete(isBoundingObject);
  }

  preFinalize() {
    super.preFinalize();

    this.boundingObject?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.boundingObject?.postFinalize();
  }

  updateBoundingObjectVisibility() {
    super.updateBoundingObjectVisibility();

    this.boundingObject?.updateBoundingObjectVisibility();
  }
}
