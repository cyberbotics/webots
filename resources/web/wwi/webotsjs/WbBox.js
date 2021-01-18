// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import {WbGeometry} from "./WbGeometry.js"

class WbBox extends WbGeometry {
  constructor(id, size) {
    super(id);
    this.size = size;

    this.wrenMesh = undefined;
  }

  delete() {
    _wr_static_mesh_delete(this.wrenMesh);
    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();
    super.computeWrenRenderable();

    const createOutlineMesh = super.isInBoundingObject();
    this.wrenMesh = _wr_static_mesh_unit_box_new(createOutlineMesh);

    _wr_renderable_set_mesh(this.wrenRenderable, this.wrenMesh);

    this.updateSize();
  }

  updateSize() {
    if (super.isInBoundingObject())
      this.updateLineScale();
    else
      _wr_transform_set_scale(this.wrenNode, _wrjs_color_array(this.size.x, this.size.y, this.size.z));
  }

  updateLineScale() {
    if (!isAValidBoundingObject())
      return;

    const offset = Math.min(this.size.x, Math.min(this.size.y, this.size.z)) * _wr_config_get_line_scale() / this.LINE_SCALE_FACTOR;
    _wr_transform_set_scale(this.wrenNode, _wrjs_color_array(this.size.x + offset, this.size.y + offset, this.size.z + offset));
  }



  static findIntersectedFace(minBound, maxBound, intersectionPoint) {
    const TOLERANCE = 1e-9;

    // determine intersected face
    if (Math.abs(intersectionPoint.x - maxBound.x) < TOLERANCE)
      return WbBox.IntersectedFace.RIGHT_FACE;
    else if (Math.abs(intersectionPoint.x - minBound.x) < TOLERANCE)
      return WbBox.IntersectedFace.LEFT_FACE;
    else if (Math.abs(intersectionPoint.z - minBound.z) < TOLERANCE)
      return WbBox.IntersectedFace.BACK_FACE;
    else if (Math.abs(intersectionPoint.z - maxBound.z) < TOLERANCE)
      return WbBox.IntersectedFace.FRONT_FACE;
    else if (Math.abs(intersectionPoint.y - maxBound.y) < TOLERANCE)
      return WbBox.IntersectedFace.TOP_FACE;
    else if (Math.abs(intersectionPoint.y - minBound.y) < TOLERANCE)
      return WbBox.IntersectedFace.BOTTOM_FACE;

    return -1;
  }

  static computeTextureCoordinate(minBound, maxBound, point, nonRecursive, intersectedFace) {
    let u, v;
    if (intersectedFace < 0)
      intersectedFace = this.findIntersectedFace(minBound, maxBound, point);

    let vertex = point.sub(minBound);
    let size = maxBound.sub(minBound);
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
        assert(false);
        break;
    }

    return new WbVector2(u, v);
  }
}

WbBox.IntersectedFace = {
  FRONT_FACE : 0,
  BACK_FACE : 1,
  LEFT_FACE : 2,
  RIGHT_FACE : 3,
  TOP_FACE : 4,
  BOTTOM_FACE : 5};

export {WbBox}
