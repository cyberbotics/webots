import WbGeometry from './WbGeometry.js';

import {resetIfNonPositive, resetIfNotInRangeWithIncludedBounds} from './utils/WbFieldChecker.js';

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
      this.subdivision = newSubdivision;

    const newRadius = resetIfNonPositive(this.#bottomRadius, 1.0);
    if (newRadius !== false)
      this.radius = newRadius;

    const newHeight = resetIfNonPositive(this.#height, 1.0);
    if (newHeight !== false)
      this.height = newHeight;

    return newSubdivision === false && newRadius === false && newHeight === false;
  }

  #updateScale() {
    const scale = _wrjs_array3(this.#bottomRadius, this.#bottomRadius, this.#height);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  #updateMesh() {
    if (!this.#sanitizeFields())
      return;

    this.#buildWrenMesh();

    if (typeof this.onRecreated === 'function')
      this.onRecreated();
  }

  #updateSize() {
    if (!this.#sanitizeFields())
      return;

    this.#updateScale();

    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
