class WbScene {
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

class WbBaseNode {
  constructor(id){
    this.id = id;
    this.mWrenObjectsCreatedCalled = false;
    this.parent = undefined;
    this.mWrenNode = undefined;
  }

  createWrenObjects() {
    this.mWrenObjectsCreatedCalled = true;

    if (this.parent !== undefined) {
      this.mWrenNode = this.parent.wrenNode;
    } else
      this.mWrenNode = wr_scene_get_root(wr_scene_get_instance());
  }
}

class WbViewpoint {
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

class WbShape extends WbBaseNode {
//compile and link the shaders
  constructor(id, castShadow, appearance, geometry) {
    super(id);
    this.castShadow = castShadow;
    this.appearance = appearance;
    this.geometry = geometry;

    this.wrenMaterial = undefined;
  }

  createWrenObjects() {
    super.createWrenObjects();

    if (this.appearance)
      this.appearance.createWrenObjects();

    if (this.geometry){
      this.geometry.parent = this
      this.geometry.createWrenObjects();
    }

    //not sur of the place
    this.applyMaterialToGeometry()
  }

  applyMaterialToGeometry() {
    if (!this.wrenMaterial)
      createWrenMaterial(1); //enum
    /*
    if (this.geometry) {
      if (this.appearance) {
        if (this.appearance.mWrenObjectsCreatedCalled)
          console.error("appearance not implemented yet");
          //this.wrenMaterial = appearance()->modifyWrenMaterial(this.wrenMaterial);
        else
          this.wrenMaterial = WbAppearance::fillWrenDefaultMaterial(this.wrenMaterial);
      } else
      this.wrenMaterial = WbAppearance::fillWrenDefaultMaterial(this.wrenMaterial);
      */
      this.geometry.setWrenMaterial(this.wrenMaterial, this.castShadow);
    //}
  }

  createWrenMaterial(type) {
    let defaultColor = [1.0, 1.0, 1.0];
    if (this.wrenMaterial)
      wr_material_delete(this.wrenMaterial);

    if (type === 1) { //enum
      this.wrenMaterial = wr_phong_material_new();
      wr_phong_material_set_color(this.wrenMaterial, defaultColor);
      wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.defaultShader());
    } else {
      console.error("pbr material not implemented yet");
      /*
      this.wrenMaterial = wr_pbr_material_new();
      wr_pbr_material_set_base_color(this.wrenMaterial, defaultColor);
      wr_material_set_default_program(this.wrenMaterial, WbWrenShaders::pbrShader());
      */
    }
  }
}

class WbGeometry extends WbBaseNode {
  constructor() {
    super(id);
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
      wr_material_set_default_program(this.mWrenEncodeDepthMaterial, WbWrenShaders.encodeDepthShader());
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

    wr_renderable_set_visibility_flags(mWrenRenderable, 0xFFF00000);
    wr_node_set_visible(WR_NODE(mWrenScaleTransform), true);
  }

  setWrenMaterial(material, castShadows) {
    if (this.mWrenRenderable) {
      wr_renderable_set_material(this.mWrenRenderable, material, NULL);
      wr_renderable_set_cast_shadows(this.mWrenRenderable, castShadows);
    }
  }

}

class WbBox extends WbGeometry{
  constructor(id, size) {
    super(id);
    this.size = new glm.vec3(size);
  }

  createWrenObjects() {
    super.createWrenObjects();
    super.computeWrenRenderable();

    mWrenMesh = wr_static_mesh_unit_box_new(false);

    wr_renderable_set_mesh(mWrenRenderable, mWrenMesh);

    updateSize();
  }

  updateSize() {
      //TODO set in a array
      wr_transform_set_scale(wrenNode(), glm.vec3(size));
  }

}

/*
class WbAppearance extends WbBaseNode {
  static fillWrenDefaultMaterial(wrenMaterial) {
    if (!wrenMaterial || wrenMaterial->type != WR_MATERIAL_PHONG) {
      wr_material_delete(wrenMaterial);
      wrenMaterial = wr_phong_material_new();
    }
    wr_material_set_default_program(wrenMaterial, WbWrenShaders::defaultShader());
    return wrenMaterial;
  }
}
*/

class WbWrenShaders {

  static buildShader(shader, vertexShaderPath, fragmentShaderpath) {
    //WbWrenOpenGlContext::makeWrenCurrent();
    wr_shader_program_set_vertex_shader_path(shader, vertexShaderPath);
    wr_shader_program_set_fragment_shader_path(shader, fragmentShaderpath);
    wr_shader_program_setup(shader);
    //WbWrenOpenGlContext::doneWren();

    if (!wr_shader_program_get_gl_name(shader)) {
      console.error("Shader compilation failed!");
      console.error(wr_shader_program_get_compilation_log(shader)+"\n");
    }
  }

  static encodeDepthShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH] = wr_shader_program_new();

      wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], 16); //enum

      wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], 4); //enum

      let minRange = 0.0;
      let maxRange = 1.0;
      wr_shader_program_create_custom_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "minRange", 0, minRange); //enum
      wr_shader_program_create_custom_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "maxRange", 0, maxRange); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "../../wren/shaders/encode_depth.vert", "../../wren/shaders/encode_depth.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH];
  }

  static defaultShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT] = wr_shader_program_new();

      wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 0);  // main texture //enum
      wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 1);  // pen texture //enum
      wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 2);  // background texture //enum
      wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 16); //enum
      wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 17); //enum

      wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 0); //enum
      wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 4); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], "../../wren/shaders/default.vert", "../../wren/shaders/default.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT];
  }
}

//gShaders static variable
WbWrenShaders.gShaders = {};

WbWrenShaders.SHADER = { //enum
    SHADER_BLOOM_BLEND : 1,
    SHADER_BOUNDING_VOLUME : 2,
    SHADER_BRIGHT_PASS : 3,
    SHADER_COLOR_NOISE : 4,
    SHADER_COORDINATE_SYSTEM : 5,
    SHADER_DEFAULT : 6,
    SHADER_DEPTH_OF_FIELD : 7,
    SHADER_DEPTH_ONLY : 8,
    SHADER_DEPTH_RESOLUTION : 9,
    SHADER_ENCODE_DEPTH : 10,
    SHADER_FOG : 11,
    SHADER_GAUSSIAN_BLUR : 12,
    SHADER_GAUSSIAN_BLUR_5_TAP : 13,
    SHADER_GAUSSIAN_BLUR_9_TAP : 14,
    SHADER_GAUSSIAN_BLUR_13_TAP : 15,
    SHADER_GTAO : 16,
    SHADER_GTAO_SPATIAL_DENOISE : 17,
    SHADER_GTAO_TEMPORAL_DENOISE : 18,
    SHADER_GTAO_COMBINE : 19,
    SHADER_HANDLES : 20,
    SHADER_HANDLES_PICKING : 21,
    SHADER_HDR_CLEAR : 22,
    SHADER_HDR_RESOLVE : 23,
    SHADER_IBL_DIFFUSE_IRRADIANCE_BAKE : 24,
    SHADER_IBL_SPECULAR_IRRADIANCE_BAKE : 25,
    SHADER_IBL_BRDF_BAKE : 26,
    SHADER_LENS_DISTORTION : 27,
    SHADER_LENS_FLARE : 28,
    SHADER_LENS_FLARE_BLEND : 29,
    SHADER_LIGHT_REPRESENTATION : 30,
    SHADER_LINE_SET : 31,
    SHADER_MERGE_SPHERICAL : 32,
    SHADER_MOTION_BLUR : 33,
    SHADER_NOISE_MASK : 34,
    SHADER_OVERLAY : 35,
    SHADER_PASS_THROUGH : 36,
    SHADER_PBR : 37,
    SHADER_PBR_STENCIL_AMBIENT_EMISSIVE : 38,
    SHADER_PBR_STENCIL_DIFFUSE_SPECULAR : 39,
    SHADER_PHONG : 40,
    SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE : 41,
    SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR : 42,
    SHADER_PICKING : 43,
    SHADER_POINT_SET : 44,
    SHADER_RANGE_NOISE : 45,
    SHADER_SHADOW_VOLUME : 46,
    SHADER_SIMPLE : 47,
    SHADER_SKYBOX : 48,
    SHADER_SMAA_EDGE_DETECT_PASS : 49,
    SHADER_SMAA_BLENDING_WEIGHT_CALCULATION_PASS : 50,
    SHADER_SMAA_FINAL_BLEND_PASS : 51,
    SHADER_COUNT : 52
  };
