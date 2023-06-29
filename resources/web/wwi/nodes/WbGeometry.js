import WbBaseNode from './WbBaseNode.js';
import WbBillboard from './WbBillboard.js';
import WbSolid from './WbSolid.js';
import { WbNodeType } from './wb_node_type.js';
import WbWorld from './WbWorld.js';
import WbBoundingSphere from './utils/WbBoundingSphere.js';
import WbVector3 from './utils/WbVector3.js';
import WbWrenMeshBuffers from './utils/WbWrenMeshBuffers.js';
import WbWrenPicker from '../wren/WbWrenPicker.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import Selector from '../Selector.js';
import { findUpperPose, nodeIsInBoundingObject } from './utils/node_utilities.js';
import WbGroup from './WbGroup.js';

export default class WbGeometry extends WbBaseNode {
  #boundingObjectFirstTimeSearch;
  #isInBoundingObject;
  #upperPoseFirstTimeSearch;
  #wrenScaleTransform;
  constructor(id) {
    super(id);

    this.pickable = false;
    this._isShadedGeometryPickable = true;

    this.#boundingObjectFirstTimeSearch = true;
    this.#isInBoundingObject = false;

    this.#upperPoseFirstTimeSearch = true;
    this.upperPose = false;
  }

  absoluteScale() {
    const up = this.upperTransform;
    return up ? up.absoluteScale() : new WbVector3(1, 1, 1);
  }

  boundingSphere() {
    return this._boundingSphere;
  }

  computeCastShadows(enabled) {
    if (typeof this._wrenRenderable === 'undefined')
      return;

    if (this.isInBoundingObject() || WbBillboard.isDescendantOfBillboard(this)) {
      _wr_renderable_set_cast_shadows(this._wrenRenderable, false);
      _wr_renderable_set_receive_shadows(this._wrenRenderable, false);
    } else
      _wr_renderable_set_cast_shadows(this._wrenRenderable, enabled);
  }

  delete() {
    if (typeof this.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined')
        parent.geometry = undefined;

      if (this.isInBoundingObject()) {
        if (parent instanceof WbSolid)
          parent.boundingObject = undefined;
        else if (parent instanceof WbGroup) {
          const index = parent.children.indexOf(this);
          console.assert(index !== -1, 'The parent node should have this node as a child for it to be deleted.');
          parent.children.splice(index, 1);
        }
      }
    }

    if (this.wrenObjectsCreatedCalled)
      this._deleteWrenRenderable();

    super.delete();
  }

  isInBoundingObject() {
    if (this.#boundingObjectFirstTimeSearch) {
      this.#isInBoundingObject = nodeIsInBoundingObject(this);
      if (this.wrenObjectsCreatedCalled)
        this.#boundingObjectFirstTimeSearch = false;
    }

    return this.#isInBoundingObject;
  }

  postFinalize() {
    super.postFinalize();

    this._boundingSphere = new WbBoundingSphere(this);
    this._boundingSphere.geomOwner = true;
    this.recomputeBoundingSphere();
  }

  recomputeBoundingSphere() { }

  setPickable(pickable) {
    if (typeof this._wrenRenderable === 'undefined' || this.isInBoundingObject())
      return;

    this.pickable = pickable && this._isShadedGeometryPickable;

    WbWrenPicker.setPickable(this._wrenRenderable, this.id, pickable);
  }

  setWrenMaterial(material, castShadows) {
    if (typeof this._wrenRenderable !== 'undefined') {
      _wr_renderable_set_material(this._wrenRenderable, material, null);
      this.computeCastShadows(castShadows);
    }
  }

  updateBoundingObjectVisibility() {
    this.#applyVisibilityFlagToWren(this.#isSelected());
  }

  #upperPose() {
    if (this.#upperPoseFirstTimeSearch) {
      this.upperPose = findUpperPose(this);
      if (this.wrenObjectsCreatedCalled)
        this.#upperPoseFirstTimeSearch = false;
    }

    return this.upperPose;
  }

  // Private functions

  #applyVisibilityFlagToWren(selected) {
    if (typeof this.#wrenScaleTransform === 'undefined')
      return;

    if (this.isInBoundingObject()) {
      if (selected) {
        _wr_renderable_set_visibility_flags(this._wrenRenderable, WbWrenRenderingContext.VF_INVISIBLE_FROM_CAMERA);
        _wr_node_set_visible(this.#wrenScaleTransform, true);
      } else {
        let shouldRender = false;
        let parent = WbWorld.instance.nodes.get(this.parent);
        while (parent) {
          if (parent.nodeType === WbNodeType.WB_NODE_TOUCH_SENSOR || parent.nodeType === WbNodeType.WB_NODE_VACUUM_GRIPPER) {
            shouldRender = parent.showOptionalRendering;
            break;
          } else
            parent = WbWorld.instance.nodes.get(parent.parent);
        }

        if (shouldRender) {
          _wr_renderable_set_visibility_flags(this._wrenRenderable, WbWrenRenderingContext.VF_INVISIBLE_FROM_CAMERA);
          _wr_node_set_visible(this.#wrenScaleTransform, true);
        } else if (_wr_node_get_parent(this.#wrenScaleTransform))
          _wr_node_set_visible(this.#wrenScaleTransform, false);
      }
    } else if (WbBillboard.isDescendantOfBillboard(this)) {
      _wr_renderable_set_visibility_flags(this._wrenRenderable, WbWrenRenderingContext.VF_INVISIBLE_FROM_CAMERA);
      _wr_node_set_visible(this.#wrenScaleTransform, true);
    } else {
      _wr_renderable_set_visibility_flags(this._wrenRenderable, WbWrenRenderingContext.VM_REGULAR);
      _wr_node_set_visible(this.#wrenScaleTransform, true);
    }
  }

  _computeWrenRenderable() {
    if (!this.wrenObjectsCreatedCalled)
      super.createWrenObjects();

    this.#wrenScaleTransform = _wr_transform_new();
    _wr_transform_attach_child(this.wrenNode, this.#wrenScaleTransform);
    this.wrenNode = this.#wrenScaleTransform;

    this._wrenRenderable = _wr_renderable_new();
    if (this.isInBoundingObject()) {
      if (typeof this.wrenMaterial === 'undefined') {
        this.wrenMaterial = _wr_phong_material_new();
        _wr_phong_material_set_color(this.wrenMaterial, _wrjs_array3(1.0, 1.0, 1.0));
        _wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.lineSetShader());
      }

      _wr_renderable_set_cast_shadows(this._wrenRenderable, false);
      _wr_renderable_set_receive_shadows(this._wrenRenderable, false);
      _wr_renderable_set_drawing_mode(this._wrenRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);

      this.setWrenMaterial(this.wrenMaterial, false);
    } else if (this.isMarker) {
      _wr_renderable_set_drawing_order(this._wrenRenderable, Enum.WR_RENDERABLE_DRAWING_ORDER_AFTER_1);
      _wr_renderable_set_receive_shadows(this._wrenRenderable, false);
    }

    _wr_transform_attach_child(this.#wrenScaleTransform, this._wrenRenderable);

    this.updateBoundingObjectVisibility();

    this.computeCastShadows(true);
  }

  _createMeshBuffers(verticesCount, indicesCount) {
    if (verticesCount <= 0 || indicesCount <= 0)
      return undefined;

    return new WbWrenMeshBuffers(verticesCount, indicesCount, this.isInBoundingObject() ? 0 : 2, 0);
  }

  _deleteWrenRenderable() {
    if (typeof this._wrenRenderable !== 'undefined') {
      if (this.wrenMaterial) {
        this.setWrenMaterial(null, false);
        _wr_material_delete(this.wrenMaterial);
        this.wrenMaterial = undefined;
      }

      // Delete picking material
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'],
        [this._wrenRenderable, 'picking']));
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'],
        [this._wrenRenderable, 'depth']));

      _wr_node_delete(this._wrenRenderable);
      this._wrenRenderable = undefined;

      this.wrenNode = _wr_node_get_parent(this.#wrenScaleTransform);
      _wr_node_delete(this.#wrenScaleTransform);
      this.#wrenScaleTransform = undefined;
    }
  }

  _isAValidBoundingObject() {
    if (!this.isInBoundingObject())
      return false;

    const upperPose = this.#upperPose();
    if (typeof upperPose !== 'undefined' && upperPose.isInBoundingObject() && upperPose.geometry() !== this)
      return false;

    return true;
  }

  #isSelected() {
    if (Selector.selectedId === this.id)
      return true;
    else if (typeof this.parent !== 'undefined')
      return Selector.checkIfParentIsSelected(this);

    return false;
  }
}

WbGeometry.LINE_SCALE_FACTOR = 250.0;
WbGeometry.MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION = 16;
