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

import {WbBaseNode} from "./WbBaseNode.js"
import {WbWrenShaders} from "./WbWrenShaders.js"
import {WbWrenMeshBuffers} from "./utils/WbWrenMeshBuffers.js"
import {World} from "./World.js";
import {WbWrenPicker} from "./WbWrenPicker.js"

class WbGeometry extends WbBaseNode {
  constructor(id) {
    super(id);
    this.wrenScaleTransform = undefined;
    this.wrenRenderable = undefined;
    this.wrenEncodeDepthMaterial = undefined;
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
    if (!this.wrenRenderable || this.isInBoundingObject)
      return;

    this.pickable = pickable && this.isShadedGeometryPickable;
    WbWrenPicker.setPickable(this.wrenRenderable, this.id, pickable);
  }

  computeWrenRenderable() {
    if (!this.wrenObjectsCreatedCalled)
      super.createWrenObjects();

    //assert(wrenScaleTransform == NULL);
    //assert(wrenRenderable == NULL);

    this.wrenScaleTransform = _wr_transform_new();
    _wr_transform_attach_child(this.wrenNode, this.wrenScaleTransform);

    this.wrenNode = this.wrenScaleTransform;

    this.wrenRenderable = _wr_renderable_new();

    Module.ccall('wr_renderable_set_material', null, ['number', 'number', 'string'], [this.wrenRenderable, this.wrenEncodeDepthMaterial, "encodeDepth"])

    _wr_transform_attach_child(this.wrenScaleTransform, this.wrenRenderable);

    this.applyVisibilityFlagToWren();

    this.computeCastShadows(true);
  }

  applyVisibilityFlagToWren() {
    if (!this.wrenScaleTransform)
      return;

    _wr_renderable_set_visibility_flags(this.wrenRenderable, 0xFFF00000);
    _wr_node_set_visible(this.wrenScaleTransform, true);
  }

  setWrenMaterial(material, castShadows) {
    if (this.wrenRenderable) {
      _wr_renderable_set_material(this.wrenRenderable, material, null);
      this.computeCastShadows(castShadows);
    }
  }

  deleteWrenRenderable() {
    if (this.wrenRenderable) {
      if (this.wrenMaterial)
        this.setWrenMaterial(null, false);

      // Delete outline material
      _wr_material_delete(this.wrenMaterial);
      this.wrenMaterial = undefined;

      // Delete encode depth material
      _wr_material_delete(this.wrenEncodeDepthMaterial);
      this.wrenEncodeDepthMaterial = undefined;

      // Delete picking material
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [this.wrenRenderable, "picking"]));

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

    return new WbWrenMeshBuffers(verticesCount, indicesCount, 2, 0); //isInBoundingObject() ? 0 : 2 3rd arg
  }

  computeCastShadows(enabled) {
    if (typeof this.wrenRenderable === 'undefined')
      return;

    if (this.isInBoundingObject) {
      _wr_renderable_set_cast_shadows(this.wrenRenderable, false);
      _wr_renderable_set_receive_shadows(this.wrenRenderable, false);
    } else
      _wr_renderable_set_cast_shadows(this.wrenRenderable, enabled);
  }

  isAValidBoundingObject() {
    if (!this.isInBoundingObject)
      return false;

    let ut = this.upperTransform();
    if (typeof ut !== 'undefined' && ut.isInBoundingObject && ut.geometry!== this)
      return false;

    return true;
  }
}

export {WbGeometry}
