import {WbBaseNode} from "./WbBaseNode.js"
import {WbWrenShaders} from "./WbWrenShaders.js"
import {WbWrenMeshBuffers} from "./utils/WbWrenMeshBuffers.js"


class WbGeometry extends WbBaseNode {
  constructor(id) {
    super(id);
    this.wrenScaleTransform = undefined;
    this.wrenRenderable = undefined;
    this.wrenEncodeDepthMaterial = undefined;
    this.wrenMesh = undefined;
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

    // used for rendering range finder camera
    if (!this.wrenEncodeDepthMaterial) {
      this.wrenEncodeDepthMaterial = _wr_phong_material_new();
      _wr_material_set_default_program(this.wrenEncodeDepthMaterial, WbWrenShaders.encodeDepthShader());
    }

    _wr_renderable_set_material(this.wrenRenderable, this.wrenEncodeDepthMaterial, "encodeDepth");

    _wr_transform_attach_child(this.wrenScaleTransform, this.wrenRenderable);

    this.applyVisibilityFlagToWren();

    _wr_renderable_set_cast_shadows(this.wrenRenderable, true);
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
      _wr_renderable_set_cast_shadows(this.wrenRenderable, castShadows);
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
}

export {WbGeometry}
