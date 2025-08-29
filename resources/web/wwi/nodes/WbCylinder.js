import WbGeometry from './WbGeometry.js';
import {resetIfNotInRangeWithIncludedBounds, resetIfNonPositive} from './utils/WbFieldChecker.js';
import WbVector3 from './utils/WbVector3.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbCylinder extends WbGeometry {
  #bottom;
  #height;
  #radius;
  #side;
  #subdivision;
  #top;
  constructor(id, radius, height, subdivision, bottom, side, top) {
    super(id);
    this.#radius = radius;
    this.#height = height;
    this.#subdivision = subdivision;
    this.#bottom = bottom;
    this.#side = side;
    this.#top = top;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_CYLINDER;
  }

  get bottom() {
    return this.#bottom;
  }

  set bottom(newBottom) {
    this.#bottom = newBottom;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  get height() {
    return this.#height;
  }

  set height(newHeight) {
    this.#height = newHeight;
    if (this.wrenObjectsCreatedCalled)
      this.#updateSize();
  }

  get radius() {
    return this.#radius;
  }

  set radius(newRadius) {
    this.#radius = newRadius;
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

  get top() {
    return this.#top;
  }

  set top(newTop) {
    this.#top = newTop;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbCylinder(customID, this.#radius, this.#height, this.#subdivision, this.#bottom, this.#side, this.#top);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();

    if (this.isInBoundingObject() && this.#subdivision < WbGeometry.MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION)
      this.#subdivision = WbGeometry.MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION;

    this.#sanitizeFields();
    this.#buildWrenMesh();
  }

  delete() {
    super.delete();

    _wr_static_mesh_delete(this._wrenMesh);
  }

  recomputeBoundingSphere() {
    const halfHeight = this.#height / 2;
    const r = this.#radius;

    if (this.#top + this.#side + this.#bottom === 0) // it is empty
      this._boundingSphere.empty();
    else if (this.#top + this.#side + this.#bottom === 1 && !this.#side) { // just one disk
      const center = this.#top ? halfHeight : -halfHeight;
      this._boundingSphere.set(new WbVector3(0, 0, center), r);
    } else
      this._boundingSphere.set(new WbVector3(), new WbVector3(r, halfHeight, 0).length());
  }

  scaledRadius() {
    const scale = this.absoluteScale();
    return Math.abs(this.#radius * Math.max(scale.x, scale.y));
  }

  scaledHeight() {
    return Math.abs(this.#height * this.absoluteScale().z);
  }

  // Private functions

  #sanitizeFields() {
    const minSubdivision = this.isInBoundingObject() ? WbGeometry.MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION : 3;
    const newSubdivision = resetIfNotInRangeWithIncludedBounds(this.#subdivision, minSubdivision, 1000, minSubdivision);
    if (newSubdivision !== false)
      this.#subdivision = newSubdivision;

    const newRadius = resetIfNonPositive(this.#radius, 1.0);
    if (newRadius !== false)
      this.#radius = newRadius;

    const newHeight = resetIfNonPositive(this.#height, 1.0);
    if (newHeight !== false)
      this.#height = newHeight;
  }

  #buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    if (!this.#bottom && !this.#side && !this.#top)
      return;

    this._computeWrenRenderable();

    const createOutlineMesh = this.isInBoundingObject();
    this._wrenMesh = _wr_static_mesh_unit_cylinder_new(this.#subdivision, this.#side, this.#top, this.#bottom,
      createOutlineMesh);

    if (createOutlineMesh)
      this.#updateLineScale();
    else
      this.#updateScale();

    // Restore pickable state
    super.setPickable(this.isPickable);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  }

  #isSuitableForInsertionInBoundingObject() {
    const invalidRadius = this.#radius <= 0.0;
    const invalidHeight = this.#height <= 0.0;

    return (!invalidRadius && !invalidHeight);
  }

  _isAValidBoundingObject() {
    return super._isAValidBoundingObject() && this.#isSuitableForInsertionInBoundingObject();
  }

  #updateMesh() {
    this.#sanitizeFields();

    this.#buildWrenMesh();

    if (typeof this.onRecreated === 'function')
      this.onRecreated();
  }

  #updateSize() {
    this.#sanitizeFields();

    if (this.isInBoundingObject())
      this.#updateLineScale();
    else
      this.#updateScale();

    if (typeof this.onChange === 'function')
      this.onChange();
  }

  #updateScale() {
    const scale = _wrjs_array3(this.#radius, this.#radius, this.#height);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  #updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;
    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.#radius * (1.0 + offset), this.#radius * (1.0 + offset),
      this.#height * (1.0 + offset)));
  }
}
