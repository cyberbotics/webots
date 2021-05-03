import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';
import WbWrenMeshBuffers from './utils/WbWrenMeshBuffers.js';
import WbWrenPicker from './../wren/WbWrenPicker.js';
import WbWrenRenderingContext from './../wren/WbWrenRenderingContext.js';

import Selector from './../Selector.js';

export default class WbGeometry extends WbBaseNode {
  constructor(id) {
    super(id);

    this.pickable = false;
    this.isShadedGeometryPickable = true;
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

  delete() {
    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined')
        parent.geometry = undefined;
    }

    if (this.wrenObjectsCreatedCalled)
      this._deleteWrenRenderable();

    super.delete();
  }

  setPickable(pickable) {
    if (typeof this.wrenRenderable === 'undefined'|| super.isInBoundingObject())
      return;

    this.pickable = pickable && this.isShadedGeometryPickable;

    WbWrenPicker.setPickable(this.wrenRenderable, this.id, pickable);
  }

  setWrenMaterial(material, castShadows) {
    if (typeof this.wrenRenderable !== 'undefined') {
      _wr_renderable_set_material(this.wrenRenderable, material, null);
      this.computeCastShadows(castShadows);
    }
  }

  updateBoundingObjectVisibility() {
    this._applyVisibilityFlagToWren(this._isSelected());
  }

  // Private functions

  _applyVisibilityFlagToWren(selected) {
    if (typeof this.wrenScaleTransform === 'undefined')
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

  _computeWrenRenderable() {
    if (!this.wrenObjectsCreatedCalled)
      super.createWrenObjects();

    this.wrenScaleTransform = _wr_transform_new();
    _wr_transform_attach_child(this.wrenNode, this.wrenScaleTransform);
    this.wrenNode = this.wrenScaleTransform;

    this.wrenRenderable = _wr_renderable_new();
    if (super.isInBoundingObject()) {
      _wr_renderable_set_cast_shadows(this.wrenRenderable, false);
      _wr_renderable_set_receive_shadows(this.wrenRenderable, false);
      _wr_renderable_set_drawing_mode(this.wrenRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
    }

    _wr_transform_attach_child(this.wrenScaleTransform, this.wrenRenderable);

    this.updateBoundingObjectVisibility();

    this.computeCastShadows(true);
  }

  _createMeshBuffers(verticesCount, indicesCount) {
    if (verticesCount <= 0 || indicesCount <= 0)
      return undefined;

    return new WbWrenMeshBuffers(verticesCount, indicesCount, super.isInBoundingObject() ? 0 : 2, 0);
  }

  _deleteWrenRenderable() {
    if (typeof this.wrenRenderable !== 'undefined') {
      // Delete picking material
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [this.wrenRenderable, 'picking']));
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [this.wrenRenderable, 'depth']));

      _wr_node_delete(this.wrenRenderable);
      this.wrenRenderable = undefined;

      this.wrenNode = _wr_node_get_parent(this.wrenScaleTransform);
      _wr_node_delete(this.wrenScaleTransform);
      this.wrenScaleTransform = undefined;
    }
  }

  _isAValidBoundingObject() {
    if (!super.isInBoundingObject())
      return false;

    const upperTransform = super.upperTransform();
    if (typeof upperTransform !== 'undefined' && upperTransform.isInBoundingObject() && upperTransform.geometry !== this)
      return false;

    return true;
  }

  _isSelected() {
    if (Selector.selectedId === this.id)
      return true;
    else if (typeof this.parent !== 'undefined')
      return Selector.checkIfParentIsSelected(this);

    return false;
  }
}

WbGeometry.LINE_SCALE_FACTOR = 250.0;
