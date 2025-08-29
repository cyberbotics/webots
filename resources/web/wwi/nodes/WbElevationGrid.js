import WbGeometry from './WbGeometry.js';
import {resetIfNegative, resetIfNonPositive} from './utils/WbFieldChecker.js';
import {arrayXPointerFloat} from './utils/utils.js';
import WbVector3 from './utils/WbVector3.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbElevationGrid extends WbGeometry {
  #height;
  #xDimension;
  #xSpacing;
  #yDimension;
  #ySpacing;
  #thickness;
  constructor(id, height, xDimension, xSpacing, yDimension, ySpacing, thickness) {
    super(id);
    this.#height = height;
    this.#xDimension = xDimension;
    this.#xSpacing = xSpacing;
    this.#yDimension = yDimension;
    this.#ySpacing = ySpacing;
    this.#thickness = thickness;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_ELEVATION_GRID;
  }

  get height() {
    return this.#height;
  }

  set height(newHeight) {
    this.#height = newHeight;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  get xDimension() {
    return this.#xDimension;
  }

  set xDimension(newXDimension) {
    this.#xDimension = newXDimension;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  get xSpacing() {
    return this.#xSpacing;
  }

  set xSpacing(newXSpacing) {
    this.#xSpacing = newXSpacing;
    if (this.wrenObjectsCreatedCalled)
      this.#updateSpacing();
  }

  get yDimension() {
    return this.#yDimension;
  }

  set yDimension(newYDimension) {
    this.#yDimension = newYDimension;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  get ySpacing() {
    return this.#ySpacing;
  }

  set ySpacing(newYSpacing) {
    this.#ySpacing = newYSpacing;
    if (this.wrenObjectsCreatedCalled)
      this.#updateSpacing();
  }

  get thickness() {
    return this.#thickness;
  }

  set thickness(newTickness) {
    this.#thickness = newTickness;
    if (this.wrenObjectsCreatedCalled)
      this.#updateMesh();
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbElevationGrid(customID, this.#height, this.#xDimension, this.#xSpacing, this.#yDimension, this.#ySpacing,
      this.#thickness);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();
    this.#buildWrenMesh();
  }

  delete() {
    super.delete();

    _wr_static_mesh_delete(this._wrenMesh);
  }

  recomputeBoundingSphere() {
    this._boundingSphere.empty();

    // create list of vertices
    const xd = this.#xDimension;
    const yd = this.#yDimension;
    const xs = this.#xSpacing;
    const ys = this.#ySpacing;
    const size = yd * xd;
    const h = [];
    for (let i = 0; i < size; i++)
      h[i] = typeof this.#height[i] === 'undefined' ? 0 : this.#height[i];

    const vertices = [];
    let index = 0;
    let posY = 0;
    for (let y = 0; y < yd; y++, posY += ys) {
      let posX = 0;
      for (let x = 0; x < xd; x++, posX += xs) {
        vertices[index] = new WbVector3(posX, posY, h[index]);
        ++index;
      }
    }

    // Ritter's bounding sphere approximation
    // (see description in WbIndexedFaceSet.recomputeBoundingSphere)
    let p2 = vertices.length > 0 ? new WbVector3(vertices[0].x, vertices[0].y, vertices[0].z) : new WbVector3();
    let p1;
    let maxDistance; // squared distance
    for (let i = 0; i < 2; ++i) {
      maxDistance = 0;
      p1 = p2;
      for (let j = 0; j < size; ++j) {
        const d = p1.distance2(vertices[j]);
        if (d > maxDistance) {
          maxDistance = d;
          p2 = vertices[j];
        }
      }
    }
    this._boundingSphere.set(p2.add(p1).mul(0.5), Math.sqrt(maxDistance) * 0.5);

    for (let j = 0; j < size; ++j)
      this._boundingSphere.enclose(vertices[j]);
  }

  #updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;

    const scalePointer = _wrjs_array3(this.#xSpacing, this.#ySpacing, 1.0 + offset);

    _wr_transform_set_scale(this.wrenNode, scalePointer);
  }

  #updateScale() {
    const scalePointer = _wrjs_array3(this.#xSpacing, this.#ySpacing, 1.0);
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

    if (this.#xDimension < 2 || this.#yDimension < 2)
      return;

    if (this.#xSpacing === 0.0 || this.#ySpacing === 0.0)
      return;

    super._computeWrenRenderable();

    // Restore pickable state
    super.setPickable(this.pickable);

    // convert height values to float, pad with zeroes if necessary
    const numValues = this.#xDimension * this.#yDimension;
    const heightData = new Array(numValues).fill(0);

    const availableValues = Math.min(numValues, this.#height?.length);
    for (let i = 0; i < availableValues; ++i)
      heightData[i] = this.#height[i];

    const createOutlineMesh = this.isInBoundingObject();

    const heightDataPointer = arrayXPointerFloat(heightData);
    this._wrenMesh = _wr_static_mesh_unit_elevation_grid_new(this.#xDimension, this.#yDimension, heightDataPointer,
      this.#thickness, createOutlineMesh);

    _free(heightDataPointer);

    if (createOutlineMesh)
      this.#updateLineScale();
    else
      this.#updateScale();

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  }

  _isAValidBoundingObject() {
    return this.#isSuitableForInsertionInBoundingObject() && super._isAValidBoundingObject();
  }

  #isSuitableForInsertionInBoundingObject() {
    const invalidDimensions = this.#xDimension < 2 || this.#yDimension < 2;
    const invalidSpacings = this.#xSpacing <= 0.0 || this.#ySpacing < 0.0;
    const invalid = invalidDimensions || invalidSpacings;

    return !invalid;
  }

  #sanitizeFields() {
    const newTickness = resetIfNegative(this.#thickness, 0.0);
    if (newTickness !== false)
      this.#thickness = newTickness;

    const newXDimension = resetIfNegative(this.#xDimension, 0);
    if (newXDimension !== false)
      this.#xDimension = newXDimension;

    const newXSpacing = resetIfNonPositive(this.#xSpacing, 1.0);
    if (newXSpacing !== false)
      this.#xSpacing = newXSpacing;

    const newYDimension = resetIfNegative(this.#yDimension, 0);
    if (newYDimension !== false)
      this.#yDimension = newYDimension;

    const newYSpacing = resetIfNonPositive(this.#ySpacing, 1.0);
    if (newYSpacing !== false)
      this.#ySpacing = newYSpacing;

    this.#checkHeight();
  }

  #checkHeight() {
    const xdyd = this.#xDimension * this.#yDimension;

    const extra = this.#height?.length - xdyd;
    if (extra > 0)
      console.warn('"height" contains ' + extra + ' ignored extra value(s).');
  }

  #updateMesh() {
    this.#sanitizeFields();

    this.#buildWrenMesh();

    if (typeof this.onRecreated === 'function')
      this.onRecreated();
  }

  #updateSpacing() {
    this.#sanitizeFields();

    if (this.isInBoundingObject())
      this.#updateLineScale();
    else
      this.#updateScale();

    if (typeof this.onChange === 'function')
      this.onChange();
  }
}
