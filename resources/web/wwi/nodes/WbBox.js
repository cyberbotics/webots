import WbGeometry from './WbGeometry.js';
import {resetVector3IfNonPositive} from './utils/WbFieldChecker.js';
import WbVector2 from './utils/WbVector2.js';
import WbVector3 from './utils/WbVector3.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbBox extends WbGeometry {
  #size;
  constructor(id, size) {
    super(id);
    this.#size = size;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_BOX;
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
    return new WbBox(customID, this.#size);
  }

  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();
    super._computeWrenRenderable();

    this.#sanitizeFields();

    const createOutlineMesh = this.isInBoundingObject();
    this._wrenMesh = _wr_static_mesh_unit_box_new(createOutlineMesh);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);

    this.#updateSize();
  }

  delete() {
    super.delete();

    _wr_static_mesh_delete(this._wrenMesh);
  }

  recomputeBoundingSphere() {
    this._boundingSphere.set(new WbVector3(), this.#size.length() / 2);
  }

  updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = Math.min(this.#size.x, Math.min(this.#size.y, this.#size.z)) * _wr_config_get_line_scale() /
      WbGeometry.LINE_SCALE_FACTOR;
    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.#size.x + offset, this.#size.y + offset, this.#size.z + offset));
  }

  #updateSize() {
    if (!this.#sanitizeFields())
      return;

    if (this.isInBoundingObject())
      this.updateLineScale();
    else
      _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.#size.x, this.#size.y, this.#size.z));
  }

  static findIntersectedFace(minBound, maxBound, intersectionPoint) {
    const tolerance = 1e-9;

    // determine intersected face
    if (Math.abs(intersectionPoint.x - maxBound.x) < tolerance)
      return WbBox.IntersectedFace.RIGHT_FACE;
    else if (Math.abs(intersectionPoint.x - minBound.x) < tolerance)
      return WbBox.IntersectedFace.LEFT_FACE;
    else if (Math.abs(intersectionPoint.z - minBound.z) < tolerance)
      return WbBox.IntersectedFace.BACK_FACE;
    else if (Math.abs(intersectionPoint.z - maxBound.z) < tolerance)
      return WbBox.IntersectedFace.FRONT_FACE;
    else if (Math.abs(intersectionPoint.y - maxBound.y) < tolerance)
      return WbBox.IntersectedFace.TOP_FACE;
    else if (Math.abs(intersectionPoint.y - minBound.y) < tolerance)
      return WbBox.IntersectedFace.BOTTOM_FACE;

    return -1;
  }

  static computeTextureCoordinate(minBound, maxBound, point, nonRecursive, intersectedFace) {
    let u, v;
    if (intersectedFace < 0)
      intersectedFace = this.findIntersectedFace(minBound, maxBound, point);

    const vertex = point.sub(minBound);
    const size = maxBound.sub(minBound);
    switch (intersectedFace) {
      case WbBox.IntersectedFace.FRONT_FACE:
        u = vertex.x / size.x;
        v = 1 - vertex.y / size.y;
        if (nonRecursive) {
          u = 0.25 * u + 0.50;
          v = 0.50 * v + 0.50;
        }
        break;
      case WbBox.IntersectedFace.BACK_FACE:
        u = 1 - vertex.x / size.x;
        v = 1 - vertex.y / size.y;
        if (nonRecursive) {
          u = 0.25 * u;
          v = 0.50 * v + 0.50;
        }
        break;
      case WbBox.IntersectedFace.LEFT_FACE:
        u = vertex.z / size.z;
        v = 1 - vertex.y / size.y;
        if (nonRecursive) {
          u = 0.25 * u + 0.25;
          v = 0.50 * v + 0.50;
        }
        break;
      case WbBox.IntersectedFace.RIGHT_FACE:
        u = 1 - vertex.z / size.z;
        v = 1 - vertex.y / size.y;
        if (nonRecursive) {
          u = 0.25 * u + 0.75;
          v = 0.50 * v + 0.50;
        }
        break;
      case WbBox.IntersectedFace.TOP_FACE:
        u = vertex.x / size.x;
        v = vertex.z / size.z;
        if (nonRecursive) {
          u = 0.25 * u + 0.50;
          v = 0.50 * v;
        }
        break;
      case WbBox.IntersectedFace.BOTTOM_FACE:
        u = vertex.x / size.x;
        v = 1 - vertex.z / size.z;
        if (nonRecursive) {
          u = 0.25 * u;
          v = 0.50 * v;
        }
        break;
      default:
        v = 0;
        u = 0;
        break;
    }

    return new WbVector2(u, v);
  }

  #sanitizeFields() {
    const newSize = resetVector3IfNonPositive(this.#size, new WbVector3(1.0, 1.0, 1.0));
    if (newSize !== false) {
      this.size = newSize;
      return false;
    }

    return true;
  }

  #isSuitableForInsertionInBoundingObject() {
    return this.#sanitizeFields();
  }

  _isAValidBoundingObject() {
    return super._isAValidBoundingObject() && this.#isSuitableForInsertionInBoundingObject();
  }
}

WbBox.IntersectedFace = {
  FRONT_FACE: 0,
  BACK_FACE: 1,
  LEFT_FACE: 2,
  RIGHT_FACE: 3,
  TOP_FACE: 4,
  BOTTOM_FACE: 5
};
