import WbGeometry from './WbGeometry.js';
import {resetIfNonPositive, resetIfNotInRangeWithIncludedBounds} from './utils/WbFieldChecker.js';
import WbVector3 from './utils/WbVector3.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbCone extends WbGeometry {
  #bottom;
  #bottomRadius;
  #height;
  #side;
  #subdivision;
  constructor(id, bottomRadius, height, subdivision, side, bottom) {
    super(id);
    this.#bottomRadius = bottomRadius;
    this.#height = height;
    this.#subdivision = subdivision;
    this.#side = side;
    this.#bottom = bottom;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_CONE;
  }

  get bottom() {
    return this.#bottom;
  }

  set bottom(newBottom) {
    this.#bottom = newBottom;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  get bottomRadius() {
    return this.#bottomRadius;
  }

  set bottomRadius(newBottomRadius) {
    this.#bottomRadius = newBottomRadius;
    if (this.wrenObjectsCreatedCalled)
      this.#updateSize();
  }

  get height() {
    return this.#height;
  }

  set height(newHeight) {
    this.#height = newHeight;
    if (this.wrenObjectsCreatedCalled)
      this.#updateSize();
  }

  get side() {
    return this.#side;
  }

  set side(newSide) {
    this.#side = newSide;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  get subdivision() {
    return this.#subdivision;
  }

  set subdivision(newSubdivision) {
    this.#subdivision = newSubdivision;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbCone(customID, this.#bottomRadius, this.#height, this.#subdivision, this.#side, this.#bottom);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();

    this.#sanitizeFields();
    this.#buildWrenMesh();
  }

  delete() {
    _wr_static_mesh_delete(this._wrenMesh);

    super.delete();
  }

  recomputeBoundingSphere() {
    const r = this.#bottomRadius;
    const h = this.#height;
    const halfHeight = h / 2.0;

    if (!this.#side || h <= r) // consider it as disk
      this._boundingSphere.set(new WbVector3(0, 0, -halfHeight), r);
    else {
      const newRadius = halfHeight + r * r / (2 * h);
      this._boundingSphere.set(new WbVector3(0, 0, halfHeight - newRadius), newRadius);
    }
  }

  scaledBottomRadius() {
    const scale = this.absoluteScale();
    return Math.abs(this.#bottomRadius * Math.max(scale.x, scale.z));
  }

  scaledHeight() {
    return Math.abs(this.#height * this.absoluteScale().y);
  }

  #buildWrenMesh() {
    super._deleteWrenRenderable();

    _wr_static_mesh_delete(this._wrenMesh);
    this._wrenMesh = undefined;

    if (!this.#bottom && !this.#side)
      return;

    this._computeWrenRenderable();

    this._wrenMesh = _wr_static_mesh_unit_cone_new(this.#subdivision, this.#side, this.#bottom);

    // Restore pickable state
    super.setPickable(this.isPickable);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);

    this.#updateScale();
  }

  #sanitizeFields() {
    if (this.isInBoundingObject())
      return false;

    const newSubdivision = resetIfNotInRangeWithIncludedBounds(this.#subdivision, 3, 1000, 3);
    if (newSubdivision !== false)
      this.#subdivision = newSubdivision;

    const newRadius = resetIfNonPositive(this.#bottomRadius, 1.0);
    if (newRadius !== false)
      this.#bottomRadius = newRadius;

    const newHeight = resetIfNonPositive(this.#height, 1.0);
    if (newHeight !== false)
      this.#height = newHeight;
  }

  #updateScale() {
    const scale = _wrjs_array3(this.#bottomRadius, this.#bottomRadius, this.#height);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  #updateMesh() {
    this.#sanitizeFields();

    this.#buildWrenMesh();

    if (typeof this.onRecreated === 'function')
      this.onRecreated();
  }

  #updateSize() {
    this.#sanitizeFields();

    this.#updateScale();

    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
