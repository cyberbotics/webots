import WbGeometry from './WbGeometry.js';
import {resetIfNonPositive, resetIfNotInRangeWithIncludedBounds} from './utils/WbFieldChecker.js';

export default class WbCapsule extends WbGeometry {
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
    return new WbCapsule(customID, this.radius, this.height, this.subdivision, this.bottom, this.side, this.top);
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

  updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;

    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(1.0 + offset, 1.0 + offset, 1.0 + offset));
  }

  // Private functions

  #buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    if (!this.bottom && !this.side && !this.top)
      return;

    super._computeWrenRenderable();

    // This must be done after super._computeWrenRenderable() otherwise
    // the outline scaling is applied to the wrong WREN transform
    if (this.isInBoundingObject())
      this.updateLineScale();

    // Restore pickable state
    super.setPickable(this.isPickable);

    const createOutlineMesh = this.isInBoundingObject();
    this._wrenMesh = _wr_static_mesh_capsule_new(this.subdivision, this.radius, this.height, this.side, this.top, this.bottom,
      createOutlineMesh);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  }

  #sanitizeFields() {
    const minSubdivision = this.isInBoundingObject() ? WbGeometry.MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION : 4;
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

  #isSuitableForInsertionInBoundingObject() {
    const invalidRadius = this.radius <= 0.0;
    const invalidHeight = this.height <= 0.0;

    return (!invalidRadius && !invalidHeight);
  }

  _isAValidBoundingObject() {
    return super._isAValidBoundingObject() && this.#isSuitableForInsertionInBoundingObject();
  }
}
