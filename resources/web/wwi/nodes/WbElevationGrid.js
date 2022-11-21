import WbGeometry from './WbGeometry.js';
import {resetIfNegative, resetIfNonPositive} from './utils/WbFieldChecker.js';
import {arrayXPointerFloat} from './utils/utils.js';

export default class WbElevationGrid extends WbGeometry {
  constructor(id, height, xDimension, xSpacing, yDimension, ySpacing, thickness) {
    super(id);
    this.height = height;
    this.xDimension = xDimension;
    this.xSpacing = xSpacing;
    this.yDimension = yDimension;
    this.ySpacing = ySpacing;
    this.thickness = thickness;
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbElevationGrid(customID, this.height, this.xDimension, this.xSpacing, this.yDimension, this.ySpacing,
      this.thickness);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();
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

    const scalePointer = _wrjs_array3(this.xSpacing, this.ySpacing, 1.0 + offset);

    _wr_transform_set_scale(this.wrenNode, scalePointer);
  }

  updateScale() {
    const scalePointer = _wrjs_array3(this.xSpacing, this.ySpacing, 1.0);
    _wr_transform_set_scale(this.wrenNode, scalePointer);
  }

  preFinalize() {
    super.preFinalize();
    this.#sanitizeFields();
  }
  // Private functions

  #buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    if (this.xDimension < 2 || this.yDimension < 2)
      return;

    if (this.xSpacing === 0.0 || this.ySpacing === 0.0)
      return;

    super._computeWrenRenderable();

    // Restore pickable state
    super.setPickable(this.pickable);

    // convert height values to float, pad with zeroes if necessary
    const numValues = this.xDimension * this.yDimension;
    const heightData = new Array(numValues).fill(0);

    const availableValues = Math.min(numValues, this.height.length);
    for (let i = 0; i < availableValues; ++i)
      heightData[i] = this.height[i];

    const createOutlineMesh = this.isInBoundingObject();

    const heightDataPointer = arrayXPointerFloat(heightData);
    this._wrenMesh = _wr_static_mesh_unit_elevation_grid_new(this.xDimension, this.yDimension, heightDataPointer,
      this.thickness, createOutlineMesh);

    _free(heightDataPointer);

    if (createOutlineMesh)
      this.updateLineScale();
    else
      this.updateScale();

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  }

  _isAValidBoundingObject() {
    return this.#isSuitableForInsertionInBoundingObject() && super._isAValidBoundingObject();
  }

  #isSuitableForInsertionInBoundingObject() {
    const invalidDimensions = this.xDimension < 2 || this.yDimension < 2;
    const invalidSpacings = this.xSpacing <= 0.0 || this.ySpacing < 0.0;
    const invalid = invalidDimensions || invalidSpacings;

    return !invalid;
  }

  #sanitizeFields() {
    const newTickness = resetIfNegative(this.thickness, 0.0);
    if (newTickness !== false)
      this.thickness = newTickness;

    const newXDimension = resetIfNegative(this.xDimension, 0);
    if (newXDimension !== false)
      this.xDimension = newXDimension;

    const newXSpacing = resetIfNonPositive(this.xSpacing, 1.0);
    if (newXSpacing !== false)
      this.xSpacing = newXSpacing;

    const newYDimension = resetIfNegative(this.yDimension, 0);
    if (newYDimension !== false)
      this.yDimension = newYDimension;

    const newYSpacing = resetIfNonPositive(this.ySpacing, 1.0);
    if (newYSpacing !== false)
      this.ySpacing = newYSpacing;

    this.#checkHeight();

    return newTickness === false && newXDimension === false && newXSpacing === false && newYDimension === false &&
     newYSpacing === false;
  }

  #checkHeight() {
    const xdyd = this.xDimension * this.yDimension;

    const extra = this.height.length - xdyd;
    if (extra > 0)
      console.warn('"height" contains ' + extra + ' ignored extra value(s).');
  }
}
