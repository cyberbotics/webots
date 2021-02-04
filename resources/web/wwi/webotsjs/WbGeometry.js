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

import {WbBaseNode} from "./WbBaseNode.js"
import {WbWrenShaders} from "./WbWrenShaders.js"
import {WbWrenMeshBuffers} from "./utils/WbWrenMeshBuffers.js"
import {World} from "./World.js";
import {WbWrenPicker} from "./WbWrenPicker.js"
import {Selector} from "./Selector.js"
import {WbWrenRenderingContext} from "./WbWrenRenderingContext.js"

class WbGeometry extends WbBaseNode {
  constructor(id) {
    super(id);
    this.wrenScaleTransform = undefined;
    this.wrenRenderable = undefined;
    this.wrenMesh = undefined;

    this.pickable = false;
    this.isShadedGeometryPickable = true;

    this.LINE_SCALE_FACTOR = 250.0;
  }

  delete(){
    if(typeof this.parent !== 'undefined'){
      let parent = World.instance.nodes.get(this.parent);
      if(typeof parent !== 'undefined')
        parent.geometry = undefined;
    }

    if (this.wrenObjectsCreatedCalled)
      this.deleteWrenRenderable();

    super.delete()
  }

  setPickable(pickable) {
    if (!this.wrenRenderable || super.isInBoundingObject())
      return;

    this.pickable = pickable && this.isShadedGeometryPickable;

    WbWrenPicker.setPickable(this.wrenRenderable, this.id, pickable);
  }

  computeWrenRenderable() {
    if (!this.wrenObjectsCreatedCalled)
      super.createWrenObjects();

    this.wrenScaleTransform = _wr_transform_new();
    _wr_transform_attach_child(this.wrenNode, this.wrenScaleTransform);
    this.wrenNode = this.wrenScaleTransform;

    this.wrenRenderable = _wr_renderable_new();
    if (super.isInBoundingObject()) {
      _wr_renderable_set_cast_shadows(this.wrenRenderable, false);
      _wr_renderable_set_receive_shadows(this.wrenRenderable, false);
      _wr_renderable_set_drawing_mode(this.renRenderable, ENUM.WR_RENDERABLE_DRAWING_MODE_LINES);
    }

    _wr_transform_attach_child(this.wrenScaleTransform, this.wrenRenderable);

    this.applyVisibilityFlagToWren(this.isSelected());

    this.computeCastShadows(true);
  }

  applyVisibilityFlagToWren(selected) {
    if (!this.wrenScaleTransform)
      return;

    if (super.isInBoundingObject()) {
      if (selected) {
        _wr_renderable_set_visibility_flags(this.wrenRenderable, WbWrenRenderingContext.VF_SELECTED_OUTLINE);
        _wr_node_set_visible(this.wrenScaleTransform, true);
      } else if (_wr_node_get_parent(this.wrenScaleTransform))
        _wr_node_set_visible(this.wrenScaleTransform, false);
    } else {
      _wr_renderable_set_visibility_flags(this.wrenRenderable, WbWrenRenderingContext.VM_REGULAR);
      _wr_node_set_visible(this.wrenScaleTransform, true);
    }
  }

  updateBoundingObjectVisibility() {
    this.applyVisibilityFlagToWren(this.isSelected());
  }

  setWrenMaterial(material, castShadows) {
    if (typeof this.wrenRenderable !== 'undefined') {
      _wr_renderable_set_material(this.wrenRenderable, material, null);
      this.computeCastShadows(castShadows);
    }
  }

  deleteWrenRenderable() {
    if (typeof this.wrenRenderable !== 'undefined') {
      // Delete picking material
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [this.wrenRenderable, "picking"]));
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [this.wrenRenderable, "depth"]));

      _wr_node_delete(this.wrenRenderable);
      this.wrenRenderable = undefined;

      this.wrenNode = _wr_node_get_parent(this.wrenScaleTransform);
      _wr_node_delete(this.wrenScaleTransform);
      this.wrenScaleTransform = undefined;
    }
  }

  createMeshBuffers(verticesCount, indicesCount) {
    if (verticesCount <= 0 || indicesCount <= 0)
      return undefined;

    return new WbWrenMeshBuffers(verticesCount, indicesCount, super.isInBoundingObject() ? 0 : 2, 0);
  }

  computeCastShadows(enabled) {
    if (typeof this.wrenRenderable === 'undefined')
      return;

    if (super.isInBoundingObject()) {
      _wr_renderable_set_cast_shadows(this.wrenRenderable, false);
      _wr_renderable_set_receive_shadows(this.wrenRenderable, false);
    } else
      _wr_renderable_set_cast_shadows(this.wrenRenderable, enabled);
  }

  isAValidBoundingObject() {
    if (!super.isInBoundingObject())
      return false;

    let ut = undefined;//TODO: this.upperTransform();
    if (typeof ut !== 'undefined' && ut.isInBoundingObject() && ut.geometry!== this)
      return false;

    return true;
  }

  isSelected() {
    if(Selector.selectedId === this.id)
      return true;
    else if (typeof this.parent !== 'undefined')
      return Selector.checkIfParentisSelected(this);
    return false;
  }
}

export {WbGeometry}
