import WbVector3 from './WbVector3.js';
import { findUpperPose } from './node_utilities.js';
import { WbNodeType } from '../wb_node_type.js';

export default class WbBoundingSphere {
  #center;
  #centerInParentCoordinates;
  #owner;
  #radius;
  #radiusInParentCoordinates;
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

  get center() {
    if (this.boundSpaceDirty)
      this.recomputeIfNeeded();
    return this.#center;
  }

  get owner() {
    return this.#owner;
  }

  addSubBoundingSphere(subBoundingSphere) {
    if (typeof subBoundingSphere === 'undefined' || this.#subBoundingSpheres.has(subBoundingSphere))
      return;

    this.#subBoundingSpheres.add(subBoundingSphere);
    subBoundingSphere.parentBoundingSphere = this;
    if (!this.boundSpaceDirty) {
      this.boundSpaceDirty = true;
      this.parentCoordinatesDirty = true;
      this.parentUpdateNotification();
    }
  }

  centerInParentCoordinates() {
    if (this.boundSpaceDirty)
      this.recomputeIfNeeded();
    if (this.parentCoordinatesDirty)
      this.recomputeSphereInParentCoordinates();
    return this.#centerInParentCoordinates;
  }

  computeSphereInGlobalCoordinates() {
    let radius;
    let center;
    const upperPose = !this.poseOwner ? findUpperPose(this.#owner) : this.#owner;

    if (typeof upperPose !== 'undefined') {
      let t = upperPose;
      if (t.nodeType !== WbNodeType.WB_NODE_TRANSFORM)
        t = t.upperTransform;
      if (t) {
        const scale = t.absoluteScale();
        radius = Math.max(Math.max(scale.x, scale.y), scale.z) * this.#radius;
      } else
        radius = this.#radius;
      center = upperPose.matrix().mulByVec3(this.#center);
    } else {
      radius = this.#radius;
      center = this.#center;
    }
    return [center, radius];
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

    const otherCenter = other.centerInParentCoordinates();
    const otherRadius = other.radiusInParentCoordinates();
    if (this.isEmpty()) {
      this.set(otherCenter, otherRadius);
      return true;
    }

    // Test matching centers
    if (this.#center === otherCenter) {
      if (otherRadius > this.#radius) {
        this.set(this.#center, otherRadius);
        return true;
      }
      return false;
    }

    const distanceVector = otherCenter.sub(this.#center);
    const distance = distanceVector.length();
    const sum = this.#radius + distance + otherRadius;

    // Other is inside the instance
    if (sum <= this.#radius * 2)
      return false;
    // Other contains the instance
    if (sum <= otherRadius * 2) {
      this.set(otherCenter, otherRadius);
      return true;
    }
    // General case
    // compute radius of the sphere which includes the two spheres.
    const newRadius = sum / 2;
    this.set(this.#center.add(distanceVector.normalized().mul(newRadius - this.#radius)), newRadius);
    return true;
  }

  isEmpty() {
    return this.#radius === 0 && this.#center.equal(new WbVector3());
  }

  radiusInParentCoordinates() {
    if (this.boundSpaceDirty)
      this.recomputeIfNeeded();
    if (this.parentCoordinatesDirty)
      this.recomputeSphereInParentCoordinates();
    return this.#radiusInParentCoordinates;
  }

  recomputeIfNeeded(dirtyOnly) {
    if (dirtyOnly && !this.boundSpaceDirty)
      return;

    if (this.#subBoundingSpheres.size === 0) {
      if (this.geomOwner)
        this.#owner.recomputeBoundingSphere();
      this.boundSpaceDirty = false;
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

    if (this.poseOwner) {
      const scale = this.#owner.nodeType === WbNodeType.WB_NODE_TRANSFORM ? this.#owner.scale : new WbVector3(1, 1, 1);
      this.#radiusInParentCoordinates = Math.max(Math.max(scale.x, scale.y), scale.z) * this.#radius;
      this.#centerInParentCoordinates = this.#owner.vrmlMatrix().mulByVec3(this.#center);
    } else {
      this.#radiusInParentCoordinates = this.#radius;
      this.#centerInParentCoordinates = this.#center;
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
