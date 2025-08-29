import WbGeometry from './WbGeometry.js';
import {resetVector2IfNonPositive} from './utils/WbFieldChecker.js';
import WbVector2 from './utils/WbVector2.js';
import WbVector3 from './utils/WbVector3.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbPlane extends WbGeometry {
  #size;
  constructor(id, size) {
    super(id);
    this.#size = size;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_PLANE;
  }

  get size() {
    return this.#size;
  }

  set size(newSize) {
    this.#size = newSize;
    if (this.wrenObjectsCreatedCalled) {
      this.#updateSize();
      if (typeof this.onChange === 'function')
        this.onChange();
    }
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbPlane(customID, this.#size);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();

    this._computeWrenRenderable();

    this.#sanitizeFields();

    const createOutlineMesh = this.isInBoundingObject();
    const wrenMesh = _wr_static_mesh_unit_rectangle_new(createOutlineMesh);

    _wr_renderable_set_mesh(this._wrenRenderable, wrenMesh);

    this.#updateSize();
  }

  delete() {
    super.delete();

    _wr_static_mesh_delete(this._wrenMesh);
  }

  recomputeBoundingSphere() {
    this._boundingSphere.set(new WbVector3(), this.#size.length() / 2);
  }

  scaledSize() {
    const s1 = this.#size;
    const s2 = this.absoluteScale();
    return new WbVector2(Math.abs(s2.x * s1.x), Math.abs(s2.y * s1.y));
  }

  updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;

    // allow the bounding sphere to scale down
    const scaleZ = 0.1 * Math.min(this.#size.x, this.#size.y);
    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.#size.x * (1.0 + offset), this.#size.y * (1.0 + offset), scaleZ));
  }

  updateScale() {
    // allow the bounding sphere to scale down
    const scaleZ = 0.1 * Math.min(this.#size.x, this.#size.y);

    const scale = _wrjs_array3(this.#size.x, this.#size.y, scaleZ);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  #updateSize() {
    this.#sanitizeFields();

    if (this.isInBoundingObject())
      this.updateLineScale();
    else
      this.updateScale();
  }

  // Private functions

  #isSuitableForInsertionInBoundingObject() {
    return !(this.#size.x <= 0.0 || this.#size.y <= 0.0);
  }

  _isAValidBoundingObject() {
    return super._isAValidBoundingObject() && this.#isSuitableForInsertionInBoundingObject();
  }

  #sanitizeFields() {
    const newSize = resetVector2IfNonPositive(this.#size, new WbVector2(1.0, 1.0));
    if (newSize !== false)
      this.#size = newSize;
  }
}
