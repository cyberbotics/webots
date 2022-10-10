import WbGeometry from './WbGeometry.js';
import {resetIfNotInRangeWithIncludedBounds, resetIfNonPositive} from './utils/WbFieldChecker.js';

export default class WbCylinder extends WbGeometry {
  constructor(id, radius, height, subdivision, bottom, side, top) {
    super(id);
    this.radius = radius;
    this.height = height;
    this.subdivision = subdivision;
    this.bottom = bottom;
    this.side = side;
    this.top = top;
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbCylinder(customID, this.radius, this.height, this.subdivision, this.bottom, this.side, this.top);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();

    if (this.isInBoundingObject() && this.subdivision < WbGeometry.MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION)
      this.subdivision = WbGeometry.MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION;

    this.#sanitizeFields();
    this.#buildWrenMesh();
  }

  delete() {
    _wr_static_mesh_delete(this._wrenMesh);

    super.delete();
  }

  updateScale() {
    const scale = _wrjs_array3(this.radius, this.radius, this.height);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;
    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.radius * (1.0 + offset), this.radius * (1.0 + offset),
      this.height * (1.0 + offset)));
  }

  // Private functions

  #sanitizeFields() {
    const minSubdivision = this.isInBoundingObject() ? WbGeometry.MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION : 3;
    const newSubdivision = resetIfNotInRangeWithIncludedBounds(this.subdivision, minSubdivision, 1000, minSubdivision);
    if (newSubdivision !== false)
      this.subdivision = newSubdivision;

    const newRadius = resetIfNonPositive(this.radius, 1.0);
    if (newRadius !== false)
      this.radius = newRadius;

    const newHeight = resetIfNonPositive(this.height, 1.0);
    if (newHeight !== false)
      this.height = newHeight;

    return newSubdivision === false && newRadius === false && newHeight === false;
  }

  #buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    if (!this.bottom && !this.side && !this.top)
      return;

    this._computeWrenRenderable();

    const createOutlineMesh = this.isInBoundingObject();
    this._wrenMesh = _wr_static_mesh_unit_cylinder_new(this.subdivision, this.side, this.top, this.bottom, createOutlineMesh);

    if (createOutlineMesh)
      this.updateLineScale();
    else
      this.updateScale();

    // Restore pickable state
    super.setPickable(this.isPickable);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  }

  #isSuitableForInsertionInBoundingObject() {
    const invalidRadius = this.radius <= 0.0;
    const invalidHeight = this.height <= 0.0;

    return (!invalidRadius && !invalidHeight);
  }

  _isAValidBoundingObject() {
    return super._isAValidBoundingObject() && this.#isSuitableForInsertionInBoundingObject();
  }
}
