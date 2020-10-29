class WbScene {
  constructor(id) {
    this.id = id;
    _wrjs_init_context(canvas.clientWidth, canvas.clientHeight);

    let vp = _wr_scene_get_viewport(_wr_scene_get_instance());
    _wr_viewport_set_size(vp, canvas.width, canvas.height);
    _wr_gl_state_set_context_active(true);
    _wr_scene_init(_wr_scene_get_instance());
  }
}

class WbBaseNode {
  constructor(id){
    this.id = id;
    this.wrenObjectsCreatedCalled = false;
    this.parent = undefined;
    this.wrenNode = undefined;
  }

  createWrenObjects() {
    this.wrenObjectsCreatedCalled = true;

    if (this.parent !== undefined) {
      this.wrenNode = this.parent.wrenNode;
    } else{
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
    }
  }
}

class WbViewpoint extends WbBaseNode {
  /*Information à disposition
  Id
  Orientation: vec4
  Position: vec3
  Exposure : float ou int
  bloomThreshold: float ou int
  zNear = float
  followsmoothness
  */
  constructor(id, orientation, position, exposure, bloomThreshold, zNear, far, followsmoothness) {
    super(id);
    this.orientation = orientation;
    this.position = position;

    this.exposure = exposure;
    this.bloomThreshold = bloomThreshold;
    this.zNear = zNear;
    this.far = far;
    this.fieldOfViewY = M_PI_4;
    this.followsmoothness = followsmoothness;

    this.inverseViewMatrix;


    this.wrenViewport = undefined;
    this.wrenCamera = undefined;

    this.createWrenObjects();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.wrenViewport = _wr_scene_get_viewport(_wr_scene_get_instance());
    //wr_viewport_set_visibility_mask(wrenViewport, WbWrenRenderingContext::instance()->optionalRenderingsMask());

    _wr_viewport_set_clear_color_rgb(this.wrenViewport, _wrjs_color_array(0.0, 0.0, 0.0));

    this.wrenCamera = _wr_viewport_get_camera(this.wrenViewport);

    this.applyPositionToWren();
    this.applyOrientationToWren();
    this.applyNearToWren();
    this.applyFarToWren();
    this.applyFieldOfViewToWren();
    //applyOrthographicViewHeightToWren();
    //updateLensFlare();
    //updatePostProcessingEffects();

    this.inverseViewMatrix = _wr_transform_get_matrix(this.wrenCamera);

    // once the camera and viewport are created, update everything in the world instance
    //WbWorld::instance()->setViewpoint(this);
  }

  applyPositionToWren() {
    _wr_camera_set_position(this.wrenCamera, _wrjs_color_array(this.position.x, this.position.y, this.position.z)) ;
  }

  applyOrientationToWren() {
    _wr_camera_set_orientation(this.wrenCamera, _wrjs_array4(this.orientation.w, this.orientation.x, this.orientation.y, this.orientation.z));
  }

  applyNearToWren() {
    _wr_camera_set_near(this.wrenCamera, this.zNear);
  }

  applyFarToWren() {
    if (this.far > 0.0)
      _wr_camera_set_far(this.wrenCamera, this.far);
    else
      _wr_camera_set_far(this.wrenCamera, 0);//DEFAULT_FAR ??? seems very far
  }

  applyFieldOfViewToWren() {
    _wr_camera_set_fovy(this.wrenCamera, this.fieldOfViewY);
  }
}

WbViewpoint.DEFAULT_FAR = 1000000.0;

class WbShape extends WbBaseNode {
  constructor(id, castShadow) {
    super(id);
    this.castShadow = castShadow;
    this.appearance = undefined;
    this.geometry = undefined;

    this.wrenMaterial = undefined;
  }


  createWrenObjects() {
    super.createWrenObjects();

    if (this.appearance)
      this.appearance.createWrenObjects();

    if (this.geometry){
      this.geometry.parent = this
      this.geometry.createWrenObjects();
      //not sure of the place
      this.applyMaterialToGeometry()
    }
  }

  applyMaterialToGeometry() {
    if (!this.wrenMaterial)
      this.createWrenMaterial(1); //enum
    /*
    if (this.geometry) {
      if (this.appearance) {
        if (this.appearance.wrenObjectsCreatedCalled)
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
    let defaultColor = _wrjs_color_array(0.0, 0.0, 0.0);
    if (this.wrenMaterial)
      _wr_material_delete(this.wrenMaterial);

    if (type === 1) { //enum
      this.wrenMaterial = _wr_phong_material_new();
      _wr_phong_material_set_color(this.wrenMaterial, defaultColor);
      _wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.defaultShader());

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
      //_wr_material_set_default_program(this.wrenEncodeDepthMaterial, WbWrenShaders.encodeDepthShader());
    }

    //_wr_renderable_set_material(this.wrenRenderable, this.wrenEncodeDepthMaterial, "encodeDepth");

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

}

class WbBox extends WbGeometry{
  constructor(id, size) {
    super(id);
    this.size = size;
  }

  createWrenObjects() {
    super.createWrenObjects();
    super.computeWrenRenderable();

    let wrenMesh = _wr_static_mesh_unit_box_new(false);

    _wr_renderable_set_mesh(this.wrenRenderable, wrenMesh);

    this.updateSize();

  }

  updateSize() {
      _wr_transform_set_scale(this.wrenNode, _wrjs_color_array(this.size.x, this.size.y, this.size.z));
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
    Module.ccall('wr_shader_program_set_vertex_shader_path', null, ['number', 'string'], [shader, vertexShaderPath]);
    Module.ccall('wr_shader_program_set_fragment_shader_path', null, ['number', 'string'], [shader, fragmentShaderpath]);
    _wr_shader_program_setup(shader);
    //WbWrenOpenGlContext::doneWren();

    if (!_wr_shader_program_get_gl_name(shader)) {
      console.error("Shader Error");
      if (_wr_shader_program_has_vertex_shader_compilation_failed(shader))
        console.error("Vertex shader compilation failed");
      else if (_wr_shader_program_has_fragment_shader_compilation_failed(shader))
        console.error("Fragment shader compilation failed");
      else
        console.error("Linkage failed");

    }
  }

  static encodeDepthShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], 16); //enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], 4); //enum

      let minRange = 0.0;
      Module.ccall('_wr_shader_program_create_custom_uniform', null, ['number, string, number, number'], [WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "minRange", 0, _wrjs_pointerOnFloat(minRange)]); //enum

      let maxRange = 1.0;
      Module.ccall('_wr_shader_program_create_custom_uniform', null, ['number, string, number, number'], [WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "maxRange", 0, _wrjs_pointerOnFloat(MaxRange)]); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "../../../wren/shaders/encode_depth.vert", "../../../wren/shaders/encode_depth.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH];
  }

  static defaultShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 0);  // main texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 1);  // pen texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 2);  // background texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 16); //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 17); //enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 0); //enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], 4); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_DEFAULT], "../../../resources/wren/shaders/default.vert", "../../../resources/wren/shaders/default.frag");
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
    SHADER_PBR_STENCIL_DIFFUSE_SPECULAR : 39,
    SHADER_PHONG : 40,
    SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE : 41,
    SHADER_PBR_STENCIL_AMBIENT_EMISSIVE : 38,
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

  class WrenRenderer {
    constructor () {
    }

    setSize ( width, height ) {
      canvas.width = width;
      canvas.height = height;
    }

    render() {
      try {
        _wr_scene_render(_wr_scene_get_instance(), null, true);
      }
      catch(error) {
        console.log("No Context");
      }
    }
  }
