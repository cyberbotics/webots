// Copyright 1996-2021 Cyberbotics Ltd.
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

import {WbGeometry} from './wbGeometry.js';

class WbBox extends WbGeometry {
  constructor(id, size) {
    super(id);
    this.size = size;
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
      _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.size.x, this.size.y, this.size.z));
  }

  updateLineScale() {
    if (!this.isAValidBoundingObject())
      return;

    const offset = Math.min(this.size.x, Math.min(this.size.y, this.size.z)) * _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;
    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.size.x + offset, this.size.y + offset, this.size.z + offset));
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbBox(customID, this.size);
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

export {WbBox};
