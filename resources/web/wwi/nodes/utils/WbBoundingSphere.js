import WbVector3 from './WbVector3.js';
export default class WbBoundingSphere {
  #center;
  #owner;
  #radius;
  #subBoundingSpheres;
  constructor(owner, center, radius) {
    this.#center = center;
    this.#owner = owner;
    this.#radius = radius;
    this.#subBoundingSpheres = new Set();
    this.parentCoordinatesDirty = true;
    if (typeof center !== 'undefined' || typeof radius !== 'undefined')
      this.boundSpaceDirty = true;
    else
      this.boundSpaceDirty = false;
  }

  empty() {
    this.set(new WbVector3(), 0);
  }

  set(center, radius) {
    this.#center = center;
    this.#radius = radius;
  }

  addSubBoundingSphere(subBoundingSphere) {
    if (typeof subBoundingSphere === 'undefined' || this.#subBoundingSpheres.has(subBoundingSphere))
      return;

    this.#subBoundingSpheres.add(subBoundingSphere);
    subBoundingSphere.parentBoundingSphere = this;
    this.boundSpaceDirty = true;
    this.parentCoordinatesDirty = true;
    this.parentUpdateNotification();
  }

  enclose(point) {
    if (this.isEmpty()) {
      this.set(point, 0);
      return;
    }

    // Test if the sphere contains the point.
    if (point.sub(this.#center).length() <= this.radius)
      return;

    const delta = this.#center.sub(point);
    const newRadius = (delta.length() + this.#radius) / 2;
    this.set(point.add(delta.normalized().mul(newRadius)), newRadius);
  }

  isEmpty() {
    return this.#radius === 0 && this.#center.equal(new WbVector3());
  }

  parentUpdateNotification() {
    if (typeof this.parentBoundingSphere !== 'undefined') {
      let parent = this.parentBoundingSphere;
      while (typeof parent !== 'undefined') {
        parent.boundSpaceDirty = true;
        parent.parentCoordinatesDirty = true;
        parent = parent.parentBoundingSphere;
      }
    }
  }
}
