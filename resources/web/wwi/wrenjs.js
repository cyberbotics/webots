class Scene {
  constructor(id) {
    this.id = id;
    _wrjs_init_context(canvas.clientWidth, canvas.clientHeight);
    let mainFrameBuffer = _wr_frame_buffer_new();
    _wr_frame_buffer_set_size(mainFrameBuffer, canvas.width, canvas.height);
    let mainFrameBufferTexture = _wr_texture_rtt_new();
    _wr_texture_rtt_enable_initialize_data(mainFrameBufferTexture, true);
    _wr_texture_set_internal_format(mainFrameBufferTexture, 3);
    _wr_frame_buffer_append_output_texture(mainFrameBuffer, mainFrameBufferTexture);
    _wr_frame_buffer_enable_depth_buffer(mainFrameBuffer, true);
    _wr_frame_buffer_setup(mainFrameBuffer);

    let vp = _wr_scene_get_viewport(_wr_scene_get_instance());
    _wr_viewport_set_frame_buffer(vp, mainFrameBuffer);
    _wr_viewport_set_size(vp, canvas.width, canvas.height);
    _wr_gl_state_set_context_active(true);
    _wr_scene_init(_wr_scene_get_instance());
  }
}

classs BaseNode {
  constructor(){
    this.mWrenObjectsCreatedCalled = false;
    this.parent = undefined;
    this.mWrenNode = undefined;
  }

  createWrenObjects() {
    this.mWrenObjectsCreatedCalled = true;

    if (this.parent !== undefined {
      this.mWrenNode = this.parent.wrenNode;
    } else
      this.mWrenNode = wr_scene_get_root(wr_scene_get_instance());
  }
}

class Viewpoint {
  /*Information à disposition
  Id
  Orientation: vec4
  Position: vec3
  Exposure : float ou int
  bloomThreshold: float ou int
  zNear = float
  followsmoothness
  */
}

class Shape {
//compile and link the shaders
  constructor(id, castShadow, material, geometry) {
    this.id = id;
    this.castShadow = castShadow;
    this.material = material;
    this.geometry = geometry;
    //shaders!
  }
}

class Geometry exenteds BaseNode {
  constructor() {
    super();
    this.mWrenScaleTransform = undefined;
    this.mWrenRenderable = undefined;
    this.mWrenEncodeDepthMaterial = undefined;
  }

  computeWrenRenderable() {
    if (!this.mWrenObjectsCreatedCalled)
      super.createWrenObjects();

    //assert(mWrenScaleTransform == NULL);
    //assert(mWrenRenderable == NULL);

    this.mWrenScaleTransform = wr_transform_new();
    wr_transform_attach_child(this.wrenNode, this.mWrenScaleTransform);
    this.wrenNode = this.mWrenScaleTransform;

    this.mWrenRenderable = wr_renderable_new();

    // used for rendering range finder camera
    if (!this.mWrenEncodeDepthMaterial) {
      this.mWrenEncodeDepthMaterial = wr_phong_material_new();
      wr_material_set_default_program(this.mWrenEncodeDepthMaterial, WbWrenShaders::encodeDepthShader());
    }

    wr_renderable_set_material(this.mWrenRenderable, mWrenEncodeDepthMaterial, "encodeDepth");

    wr_transform_attach_child(this.mWrenScaleTransform, this.mWrenRenderable);

    this.applyVisibilityFlagToWren();

    wr_renderable_set_cast_shadows(this.mWrenRenderable, true);
  }

  //TODO: check if necessary
  applyVisibilityFlagToWren() {
    if (!this.mWrenScaleTransform)
      return;

    wr_renderable_set_visibility_flags(mWrenRenderable, WbWrenRenderingContext::VM_REGULAR);
    wr_node_set_visible(WR_NODE(mWrenScaleTransform), true);
  }

}

class Box extends Geometry{
  constructor(id, size) {
    super();
    this.id = id;
    this.size = new glm::vec3(size);

    createWrenBox()
  }

  createWrenBox() {
    super.createWrenObjects();
    super.computeWrenRenderable();

    sanitizeFields();

    mWrenMesh = wr_static_mesh_unit_box_new(false);

    wr_renderable_set_mesh(mWrenRenderable, mWrenMesh);

    updateSize();
  }
}
