import WbGeometry from './WbGeometry.js';
import {resetIfNotInRangeWithIncludedBounds, resetIfNonPositive} from './utils/WbFieldChecker.js';
import WbVector3 from './utils/WbVector3.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbSphere extends WbGeometry {
  #ico;
  #radius;
  #subdivision;
  constructor(id, radius, ico, subdivision) {
    super(id);
    this.#radius = radius;
    this.#ico = ico;
    this.#subdivision = subdivision;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_SPHERE;
  }

  get ico() {
    return this.#ico;
  }

  set ico(newIco) {
    this.#ico = newIco;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  get radius() {
    return this.#radius;
  }

  set radius(newRadius) {
    this.#radius = newRadius;
    if (this.wrenObjectsCreatedCalled)
      this.#updateRadius();
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
    return new WbSphere(customID, this.#radius, this.#ico, this.#subdivision);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();

    this.#sanitizeFields();
    this.#buildWrenMesh();
  }

  delete() {
    super.delete();

    _wr_static_mesh_delete(this._wrenMesh);
  }

  recomputeBoundingSphere() {
    this._boundingSphere.set(new WbVector3(), this.#radius);
  }

  scaledRadius() {
    const scale = this.absoluteScale();
    return Math.abs(this.#radius * Math.max(Math.max(scale.x, scale.y), scale.z));
  }

  // Private functions

  #updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;
    const scaledRadius = this.#radius * (1.0 + offset);
    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(scaledRadius, scaledRadius, scaledRadius));
  }

  #updateScale() {
    this.#sanitizeFields();

    const scaledRadius = this.#radius;

    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(scaledRadius, scaledRadius, scaledRadius));
  }

  #updateRadius() {
    this.#sanitizeFields();

    if (this.isInBoundingObject())
      this.#updateLineScale();
    else
      this.#updateScale();

    if (typeof this.onChange === 'function')
      this.onChange();
  }

  #updateMesh() {
    this.#sanitizeFields();

    this.#buildWrenMesh();

    if (typeof this.onRecreated === 'function')
      this.onRecreated();
  }

  _isAValidBoundingObject() {
    return super._isAValidBoundingObject() && this.#radius > 0;
  }

  #buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    super._computeWrenRenderable();

    const createOutlineMesh = this.isInBoundingObject();
    this._wrenMesh = _wr_static_mesh_unit_sphere_new(this.#subdivision, this.#ico, createOutlineMesh);

    // Restore pickable state
    super.setPickable(this.isPickable);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);

    if (createOutlineMesh)
      this.#updateLineScale();
    else
      this.#updateScale();
  }

  #sanitizeFields() {
    let newSubdivision;
    if (this.#ico)
      newSubdivision = resetIfNotInRangeWithIncludedBounds(this.#subdivision, 1, 5, 1);
    else
      newSubdivision = resetIfNotInRangeWithIncludedBounds(this.#subdivision, 3, 32, 24);

    if (newSubdivision !== false)
      this.#subdivision = newSubdivision;

    const newRadius = resetIfNonPositive(this.#radius, 1.0);
    if (newRadius !== false)
      this.#radius = newRadius;
  }
}
