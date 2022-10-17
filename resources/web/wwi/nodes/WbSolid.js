import WbTransform from './WbTransform.js';

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

    if (typeof this.boundingObject !== 'undefined')
      this.boundingObject.createWrenObjects();
  }

  delete(isBoundingObject) {
    if (typeof this.boundingObject !== 'undefined')
      this.boundingObject.delete(true);

    super.delete(isBoundingObject);
  }

  preFinalize() {
    super.preFinalize();

    if (typeof this.boundingObject !== 'undefined')
      this.boundingObject.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    if (typeof this.boundingObject !== 'undefined')
      this.boundingObject.postFinalize();
  }

  updateBoundingObjectVisibility() {
    super.updateBoundingObjectVisibility();

    if (typeof this.boundingObject !== 'undefined')
      this.boundingObject.updateBoundingObjectVisibility();
  }
}
