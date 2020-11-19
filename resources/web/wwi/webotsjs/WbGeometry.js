import {WbBaseNode} from "./WbBaseNode.js"
import {WbWrenShaders} from "./WbWrenShaders.js"


class WbGeometry extends WbBaseNode {
  constructor(id) {
    super(id);
    this.wrenScaleTransform = undefined;
    this.wrenRenderable = undefined;
    this.wrenEncodeDepthMaterial = undefined;
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

  //TODO: check if necessary
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
  //Not use for now, normaly in cone
  /*
  deleteWrenRenderable() {
    if (this.wrenRenderable) {
      if (this.wrenMaterial)
        setWrenMaterial(NULL, false);

      // Delete outline material
      wr_material_delete(mWrenMaterial);
      mWrenMaterial = NULL;

      // Delete encode depth material
      wr_material_delete(mWrenEncodeDepthMaterial);
      mWrenEncodeDepthMaterial = NULL;

      // Delete picking material
      wr_material_delete(wr_renderable_get_material(mWrenRenderable, "picking"));

      wr_node_delete(WR_NODE(mWrenRenderable));
      mWrenRenderable = NULL;

      setWrenNode(wr_node_get_parent(WR_NODE(mWrenScaleTransform)));
      wr_node_delete(WR_NODE(mWrenScaleTransform));
      mWrenScaleTransform = NULL;
    }
  }*/

}

export {WbGeometry}
