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

  get radius() {
    if (this.boundSpaceDirty)
      this.recomputeIfNeeded();
    return this.#radius;
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

  centerInParentCoordinates() {
    if (this.boundSpaceDirty)
      this.recomputeIfNeeded();
    if (this.parentCoordinatesDirty)
      this.recomputeSphereInParentCoordinates();
    return this.centerInParentCoordinates;
  }

  empty() {
    this.set(new WbVector3(), 0);
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

  encloseBoundingSphere(other) {
    if (other.isEmpty())
      return false;

    const otherCenter = const_cast<WbBoundingSphere *>(other)->centerInParentCoordinates();
    const double otherRadius = const_cast<WbBoundingSphere *>(other)->radiusInParentCoordinates();
    if (isEmpty()) {
      set(otherCenter, otherRadius);
      return true;
    }

    // Test matching centers
    if (mCenter == otherCenter) {
      if (otherRadius > mRadius) {
        set(mCenter, otherRadius);
        return true;
      }
      return false;
    }

    const WbVector3 &distanceVector = otherCenter - mCenter;
    const double distance = distanceVector.length();
    const double sum = mRadius + distance + otherRadius;

    // Other is inside the instance
    if (sum <= mRadius * 2)
      return false;
    // Other contains the instance
    if (sum <= otherRadius * 2) {
      set(otherCenter, otherRadius);
      return true;
    }

    // General case
    // compute radius of the sphere which includes the two spheres.
    const double newRadius = sum / 2.0;
    set(mCenter + distanceVector.normalized() * (newRadius - mRadius), newRadius);
    return true;
  }

  isEmpty() {
    return this.#radius === 0 && this.#center.equal(new WbVector3());
  }

  recomputeIfNeeded(dirtyOnly) {
    if (dirtyOnly && !this.boundSpaceDirty)
      return;

    if (this.#subBoundingSpheres.size === 0) {
      if (this.geomOwner)
        this.#owner.recomputeBoundingSphere();
      this._boundSpaceDirty = false;
      return;
    }

    const prevCenter = this.#center;
    const prevRadius = this.#radius;
    this.#center = new WbVector3();
    this.#radius = 0;
    for (const sub of this.#subBoundingSpheres.values()) {
      sub.recomputeIfNeeded(true);
      if (!sub.isEmpty())
        this.encloseBoundingSphere(sub);
    }

    if (this.parentBoundingSphere && (!this.#center.equal(prevCenter) || this.#radius !== prevRadius))
      this.parentCoordinatesDirty = true;
    this.boundSpaceDirty = false;
  }

  recomputeSphereInParentCoordinates() {
    if (!this.parentCoordinatesDirty)
      return;

    if (this.transformOwner) {
      const scale = this.#owner.scale();
      this.radiusInParentCoordinates = Math.max(Math.max(scale.x, scale.y), scale.z) * this.#radius;
      this.centerInParentCoordinates = this.#owner.vrmlMatrix() * this.#center;
    } else {
      this.radiusInParentCoordinates = this.#radius;
      this.centerInParentCoordinates = this.#center;
    }
    this.parentCoordinatesDirty = false;
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

  set(center, radius) {
    this.#center = center;
    this.#radius = radius;
  }
}
