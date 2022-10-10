import WbGeometry from './WbGeometry.js';
import {resetIfNotInRangeWithIncludedBounds, resetIfNonPositive} from './utils/WbFieldChecker.js';

export default class WbSphere extends WbGeometry {
  constructor(id, radius, ico, subdivision) {
    super(id);
    this.radius = radius;
    this.ico = ico;
    this.subdivision = subdivision;
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbSphere(customID, this.radius, this.ico, this.subdivision);
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

  updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;
    const scaledRadius = this.radius * (1.0 + offset);
    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(scaledRadius, scaledRadius, scaledRadius));
  }

  updateScale() {
    if (!this.#sanitizeFields())
      return;

    const scaledRadius = this.radius;

    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(scaledRadius, scaledRadius, scaledRadius));
  }

  // Private functions

  _isAValidBoundingObject() {
    return super._isAValidBoundingObject() && this.radius > 0;
  }

  #buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    super._computeWrenRenderable();

    const createOutlineMesh = this.isInBoundingObject();
    this._wrenMesh = _wr_static_mesh_unit_sphere_new(this.subdivision, this.ico, createOutlineMesh);

    // Restore pickable state
    super.setPickable(this.isPickable);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);

    if (createOutlineMesh)
      this.updateLineScale();
    else
      this.updateScale();
  }

  #sanitizeFields() {
    let newSubdivision;
    if (this.ico)
      newSubdivision = resetIfNotInRangeWithIncludedBounds(this.subdivision, 1, 5, 1);
    else
      newSubdivision = resetIfNotInRangeWithIncludedBounds(this.subdivision, 3, 32, 24);

    if (newSubdivision !== false)
      this.subdivision = newSubdivision;

    const newRadius = resetIfNonPositive(this.radius, 1.0);
    if (newRadius !== false)
      this.radius = newRadius;

    return newSubdivision === false && newRadius === false;
  }
}
