class WbScene {
  constructor(id) {
    this.id = id;
    this.wrenMainFrameBuffer = null;
    this.wrenMainFrameBufferTexture = null;
    this.wrenNormalFrameBufferTexture = null;
    this.wrenDepthFrameBufferTexture = null;


    //_wrjs_init_context(canvas.clientWidth, canvas.clientHeight);
    //To have the same display size as in webots
    _wrjs_init_context(798, 598);
    _wr_scene_init(_wr_scene_get_instance());

    _wr_gl_state_set_context_active(true);

    this.updateFrameBuffer();

    _wr_scene_set_fog_program(_wr_scene_get_instance(), WbWrenShaders.fogShader());
    _wr_scene_set_shadow_volume_program(_wr_scene_get_instance(), WbWrenShaders.shadowVolumeShader());

    //WbWrenPostProcessingEffects.loadResources();
    this.updateWrenViewportDimensions();
  }

  updateFrameBuffer() {
    if (this.wrenMainFrameBuffer)
      _wr_frame_buffer_delete(this.wrenMainFrameBuffer);

    if (this.wrenMainFrameBufferTexture)
      _wr_texture_delete(this.wrenMainFrameBufferTexture);

    if (this.wrenNormalFrameBufferTexture)
      _wr_texture_delete(this.wrenNormalFrameBufferTexture);

    if (this.wrenDepthFrameBufferTexture)
      _wr_texture_delete(this.wrenDepthFrameBufferTexture);

    this.wrenMainFrameBuffer = _wr_frame_buffer_new();
    _wr_frame_buffer_set_size(this.wrenMainFrameBuffer, canvas.width, canvas.height);

    this.wrenMainFrameBufferTexture = _wr_texture_rtt_new();
    _wr_texture_set_internal_format(this.wrenMainFrameBufferTexture, 6);//enum

    this.wrenNormalFrameBufferTexture = _wr_texture_rtt_new();
    _wr_texture_set_internal_format(this.wrenNormalFrameBufferTexture, 3);//enum

    _wr_frame_buffer_append_output_texture(this.wrenMainFrameBuffer, this.wrenMainFrameBufferTexture);
    _wr_frame_buffer_append_output_texture(this.wrenMainFrameBuffer, this.wrenNormalFrameBufferTexture);
    _wr_frame_buffer_enable_depth_buffer(this.wrenMainFrameBuffer, true);

    this.wrenDepthFrameBufferTexture = _wr_texture_rtt_new();
    _wr_texture_set_internal_format(this.wrenDepthFrameBufferTexture, 11);//enum
    _wr_frame_buffer_set_depth_texture(this.wrenMainFrameBuffer, this.wrenDepthFrameBufferTexture);

    _wr_frame_buffer_setup(this.wrenMainFrameBuffer);
    _wr_viewport_set_frame_buffer(_wr_scene_get_viewport(_wr_scene_get_instance()), this.wrenMainFrameBuffer);

    _wr_viewport_set_size(_wr_scene_get_viewport(_wr_scene_get_instance()), canvas.width, canvas.height);
  }

  updateWrenViewportDimensions() {
    _wr_viewport_set_pixel_ratio(_wr_scene_get_viewport(_wr_scene_get_instance()), 1);// TODO get the right pixel ratio
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

    if (typeof this.parent !== 'undefined') {
      this.wrenNode = this.parent.wrenNode;
    } else{
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
    }
  }
}

class WbViewpoint extends WbBaseNode {
  constructor(id, orientation, position, exposure, bloomThreshold, zNear, far, followsmoothness) {
    super(id);
    this.orientation = orientation;
    this.position = position;

    WbViewpoint.exposure = exposure;
    this.bloomThreshold = bloomThreshold;
    this.zNear = zNear;
    this.far = far;
    this.aspectRatio = 1.0;
    this.fieldOfView = M_PI_4;
    this.fieldOfViewY = M_PI_4;
    this.tanHalfFieldOfViewY = TAN_M_PI_8;
    this.followsmoothness = followsmoothness;

    this.inverseViewMatrix;

    this.wrenHdr = new WbWrenHdr();
    this.wrenGtao = new WbWrenGtao();

    this.wrenViewport = undefined;
    this.wrenCamera = undefined;
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
    this.updatePostProcessingEffects();
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
    console.log(this.far);
    if (this.far > 0.0)
      _wr_camera_set_far(this.wrenCamera, this.far);
    else
      _wr_camera_set_far(this.wrenCamera, WbViewpoint.DEFAULT_FAR);
  }

  applyFieldOfViewToWren() {
    _wr_camera_set_fovy(this.wrenCamera, this.fieldOfViewY);
  }

  updateNear() {
    //TODO
    //if (WbFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.05))
      //return;

    if (this.far > 0.0 && this.far < this.near) {
      this.near = this.far;
    }

    if (this.wrenObjectsCreatedCalled)
      this.applyNearToWren();
  }

  updateFar() {
    //TODO
    //if (WbFieldChecker::resetDoubleIfNegative(this, mFar, 0.0))
      //return;

    if (this.far > 0.0 && this.far < this.near) {
      this.far = this.near + 1.0;
      //TOCHECK it is strange to return here but not in update near
      //return;
    }

    if (this.wrenObjectsCreatedCalled)
      this.applyFarToWren();
  }

  updateFieldOfViewY() {
    this.tanHalfFieldOfViewY = tan(0.5 * this.fieldOfView);  // stored for reuse in viewpointRay()

    // According to VRML standards, the meaning of mFieldOfView depends on the aspect ratio:
    // the view angle is taken with respect to the largest dimension
    if (this.aspectRatio < 1.0)
      this.fieldOfViewY = this.fieldOfView;
    else {
      this.tanHalfFieldOfViewY /= this.aspectRatio;
      this.fieldOfViewY = 2.0 * Math.atan(this.tanHalfFieldOfViewY);
    }
  }

  updatePostProcessingEffects(){
    if(!this.wrenObjectsCreatedCalled)
      return;

    if (this.wrenHdr) {
      this.wrenHdr.setup(this.wrenViewport);
      this.updateExposure();
    }

    /*
    if (this.wrenGtao) {
      //TODO
      //let qualityLevel = WbPreferences::instance()->value("OpenGL/GTAO", 2).toInt();
      let qualityLevel = 2;
      if (qualityLevel === 0)
        this.wrenGtao.detachFromViewport();
      else {
        this.wrenGtao.setHalfResolution(qualityLevel <= 2);
        this.wrenGtao.setup(this.wrenViewport);
        this.updateNear();
        this.updateFar();
        this.updateFieldOfViewY();
      }
    }*/
  }

  updateExposure() {
    //TODO
    //if (WbFieldChecker::resetDoubleIfNegative(this, this.exposure, 1.0))
      //return;
    if (this.wrenObjectsCreatedCalled && this.wrenHdr)
      this.wrenHdr.setExposure(Viewpoint.exposure);
  }

}

WbViewpoint.DEFAULT_FAR = 1000000.0;
//TODO: remove this global value once we have a scenetree;
WbViewpoint.exposure = 1;
class WbWrenAbstractPostProcessingEffect {
  constructor() {
    this.wrenPostProcessingEffect = undefined;
    this.wrenViewport = undefined;

    this.hasBeenSetup = false;
  }
}

class WbWrenHdr extends WbWrenAbstractPostProcessingEffect{
  constructor(){
    super();
    this.exposure = 1.0;
  }

  setup(viewport){
    if (this.wrenPostProcessingEffect) {
      // In case we want to update the viewport, the old postProcessingEffect has to be removed first
      if (this.wrenViewport == viewport)
        _wr_viewport_remove_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);

      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
    }

    this.wrenViewport = viewport;

    let width = _wr_viewport_get_width(this.wrenViewport);
    let height = _wr_viewport_get_height(this.wrenViewport);

    this.wrenPostProcessingEffect = this.hdrResolve(width, height);

    this.applyParametersToWren();

    _wr_viewport_add_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);
    _wr_post_processing_effect_setup(this.wrenPostProcessingEffect);

    this.hasBeenSetup = true;
  }

  setExposure(exposure){
    this.exposure = exposure;

    this.applyParametersToWren();
  }

  applyParametersToWren() {
    if (!this.wrenPostProcessingEffect)
      return;
    let firstPass = _wr_post_processing_effect_get_first_pass(this.wrenPostProcessingEffect)
    let exposurePointer = _wrjs_pointerOnFloat(this.exposure);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [firstPass, "exposure", exposurePointer]);
  }

  hdrResolve(width, height) {
    let hdrResolveEffect = _wr_post_processing_effect_new();
    _wr_post_processing_effect_set_drawing_index(hdrResolveEffect, 7); //enum

    let hdrPass = _wr_post_processing_effect_pass_new();
    Module.ccall('wr_post_processing_effect_pass_set_name', null, ['number', 'string'], [hdrPass, "hdrResolve"]);
    _wr_post_processing_effect_pass_set_program(hdrPass, WbWrenShaders.hdrResolveShader());
    _wr_post_processing_effect_pass_set_output_size(hdrPass, width, height);
    _wr_post_processing_effect_pass_set_alpha_blending(hdrPass, false);
    _wr_post_processing_effect_pass_set_input_texture_count(hdrPass, 1);
    _wr_post_processing_effect_pass_set_output_texture_count(hdrPass, 1);
    _wr_post_processing_effect_pass_set_output_texture_format(hdrPass, 0, 2); //enum
    _wr_post_processing_effect_append_pass(hdrResolveEffect, hdrPass);

    _wr_post_processing_effect_set_result_program(hdrResolveEffect, WbWrenShaders.passThroughShader());

     return hdrResolveEffect
  }
}

class WbWrenGtao extends WbWrenAbstractPostProcessingEffect {
  constructor(){
    super();
    this.halfResolution = false;

    this.gtaoPass = null;
    this.temporalPass = null
    this.near = 0.0;
    this.far = 0.0
    this.fov = 0.78
    this.radius = 2.0;
    this.flipNormalY = 0.0
    this.frameCounter = 0;

    this.clipInfo = [0, 0, 0, 0];
    this.params = [0, 0, 0, 0];
    this.rotations = [60.0, 300.0, 180.0, 240.0, 120.0, 0.0];
    this.offsets = [0.0, 0.5, 0.25, 0.75];

  }

  setHalfResolution(halfResolution) {
    this.halfResolution = halfResolution;
  }

  detachFromViewport() {
    if (this.wrenViewport) {
      _wr_viewport_set_ambient_occlusion_effect(this.wrenViewport, null);
      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
      this.wrenPostProcessingEffect = null;
      this.wrenViewport = null;
      this.hasBeenSetup = false;
    }
  }

  setup(viewport) {
    if (this.wrenPostProcessingEffect) {
      // In case we want to update the viewport, the old postProcessingEffect has to be removed first
      if (this.wrenViewport == viewport)
        _wr_viewport_remove_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);

      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
    }

    this.wrenViewport = viewport;

    let width = _wr_viewport_get_width(this.wrenViewport);
    let height = _wr_viewport_get_height(this.wrenViewport);

    if (this.halfResolution) {
      width = width <= 1.0 ? 2.0 : width;
      height = height <= 1.0 ? 2.0 : height;
    }

    let viewportFramebuffer = _wr_viewport_get_frame_buffer(this.wrenViewport);

    let depthTexture = _wr_frame_buffer_get_depth_texture(viewportFramebuffer);
    let normalTexture = _wr_frame_buffer_get_output_texture(viewportFramebuffer, 1);

    //this.wrenPostProcessingEffect = WbWrenPostProcessingEffects.gtao(width, height, WR_TEXTURE_INTERNAL_FORMAT_RGB16F, depthTexture, normalTexture, mHalfResolution);

    this.gtaoPass = Module.ccall('wr_post_processing_effect_get_pass', 'number', ['number', 'string'], [this.wrenPostProcessingEffect, "gtaoForwardPass"]);
    this.temporalPass = Module.ccall('wr_post_processing_effect_get_pass', 'number', ['number', 'string'], [this.wrenPostProcessingEffect, "temporalDenoise"]);

    this.applyParametersToWren();

    _wr_viewport_set_ambient_occlusion_effect(this.wrenViewport, this.wrenPostProcessingEffect);
    _wr_post_processing_effect_setup(this.wrenPostProcessingEffect);

    this.hasBeenSetup = true;
  }

  applyParametersToWren() {
    if (!this.wrenPostProcessingEffect)
      return;

    this.clipInfo[0] = this.near;
    this.clipInfo[1] = this.far ? this.far : 1000000.0;
    this.clipInfo[2] = 0.5 * (_wr_viewport_get_height(this.wrenViewport) / (2.0 * Math.tanf(this.fov * 0.5)));

    let array4 = _wrjs_array4(this.clipInfo[0], this.clipInfo[1], this.clipInfo[2], this.clipInfo[3])
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.gtaoPass, "clipInfo", array4]);

    this.params[0] = this.rotations[this.frameCounter % 6] / 360.0;
    this.params[1] = this.offsets[(this.frameCounter / 6) % 4];

    array4 = _wrjs_array4(this.params[0], this.params[1], this.params[2], this.params[3])
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.gtaoPass, "params", array4]);

    let radiusPointer = _wrjs_pointerOnFloat(this.radius);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.gtaoPass, "radius", radiusPointer]);

    let flipNormalYPointer = _wrjs_pointerOnFloat(this.flipNormalY);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.gtaoPass, "flipNormalY", flipNormalYPointer]);

    ++this.frameCounter;
  }
}

class WbBackground extends WbBaseNode {
  constructor(id, skyColor, luminosity, cubeArray, irradianceCubeArray) {
    super(id);
    this.skyColor = skyColor;
    this.luminosity = luminosity;
    this.cubeArray = cubeArray;
    this.irradianceCubeArray = irradianceCubeArray;

    this.skyboxShaderProgram = undefined;
    this.skyboxMaterial = undefined;
    this.skyboxRenderable = undefined;
    this.skyboxMesh = undefined;
    this.skyboxTransform = undefined;

    this.hdrClearShaderProgram = undefined;
    this.hdrClearMaterial = undefined;
    this.hdrClearRenderable = undefined;
    this.hdrClearMesh = undefined;
    this.hdrClearTransform = undefined;

    this.cubeMapTexture = undefined;
    this.irradianceCubeTexture = undefined;
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.skyboxShaderProgram = WbWrenShaders.skyboxShader();
    this.skyboxMaterial = _wr_phong_material_new();
    this.skyboxRenderable = _wr_renderable_new();
    this.skyboxMesh = _wr_static_mesh_unit_box_new(false);

    _wr_material_set_default_program(this.skyboxMaterial, this.skyboxShaderProgram);
    _wr_renderable_set_cast_shadows(this.skyboxRenderable, false);
    _wr_renderable_set_receive_shadows(this.skyboxRenderable, false);
    _wr_renderable_set_mesh(this.skyboxRenderable, this.skyboxMesh);
    _wr_renderable_set_material(this.skyboxRenderable, this.skyboxMaterial, null);
    _wr_renderable_set_drawing_mode(this.skyboxRenderable, 0x4); //enum
    _wr_renderable_set_face_culling(this.skyboxRenderable, false);

    this.skyboxTransform = _wr_transform_new();
    _wr_transform_attach_child(this.skyboxTransform, this.skyboxRenderable);

    this.hdrClearShaderProgram = WbWrenShaders.hdrClearShader();
    this.hdrClearMaterial = _wr_phong_material_new();
    this.hdrClearRenderable = _wr_renderable_new();
    this.hdrClearMesh = _wr_static_mesh_quad_new();

    _wr_material_set_default_program(this.hdrClearMaterial, this.hdrClearShaderProgram);
    _wr_renderable_set_cast_shadows(this.hdrClearRenderable, false);
    _wr_renderable_set_receive_shadows(this.hdrClearRenderable, false);
    _wr_renderable_set_mesh(this.hdrClearRenderable, this.hdrClearMesh);
    _wr_renderable_set_material(this.hdrClearRenderable, this.hdrClearMaterial, null);
    _wr_renderable_set_drawing_mode(this.hdrClearRenderable, 0x4);

    this.hdrClearTransform = _wr_transform_new();
    _wr_transform_attach_child(this.hdrClearTransform, this.hdrClearRenderable);

    //TODO ask why several background?
    //if (isFirstInstance())
    this.applyColourToWren();
  }

  applyColourToWren() {
    let colorPointer = _wrjs_color_array(this.skyColor.x, this.skyColor.y, this.skyColor.z);

    _wr_viewport_set_clear_color_rgb(_wr_scene_get_viewport(_wr_scene_get_instance()), colorPointer);
    if (this.wrenObjectsCreatedCalled) {
      // use wren's set_diffuse to transform to linear color space
      _wr_phong_material_set_diffuse(this.hdrClearMaterial, colorPointer);

      // de-gamma correct
      let hdrColor = [Math.pow(this.skyColor.x, 2.2), Math.pow(this.skyColor.y, 2.2), Math.pow(this.skyColor.z, 2.2)];
      //TODO get rid of the gloabl variable
      // reverse tone map
      let exposure = WbViewpoint.exposure;
      for (let i = 0; i < 3; ++i)
        hdrColor[i] = -Math.log(1.000000001 - hdrColor[i]) / exposure;

      let hdrColorPointer = _wrjs_color_array(hdrColor[0], hdrColor[1], hdrColor[2]);
      _wr_phong_material_set_linear_diffuse(this.hdrClearMaterial, hdrColorPointer);
      _wr_scene_set_hdr_clear_quad(_wr_scene_get_instance(), this.hdrClearRenderable);
    }
  }

  destroySkyBox() {
    _wr_scene_set_skybox(_wr_scene_get_instance(), null);

    if (typeof this.skyboxMaterial !== 'undefined')
      _wr_material_set_texture_cubemap(this.skyboxMaterial, null, 0);

    if (typeof this.cubeMapTexture !== 'undefined') {
      _wr_texture_delete(this.cubeMapTexture);
      this.cubeMapTexture = undefined;
    }

    if (typeof this.irradianceCubeTexture !== 'undefined') {
      _wr_texture_delete(this.irradianceCubeTexture);
      this.irradianceCubeTexture = undefined;
    }
  }

  applySkyBoxToWren() {
    this.destroySkyBox();

    let hdrImageData;

    // 1. Load the background.
    if(this.cubeArray.length === 6) {
      console.log(this.cubeArray);
      this.cubeMapTexture = _wr_texture_cubemap_new();
      _wr_texture_set_internal_format(this.cubeMapTexture, 3); //enum

      let bitsPointers = []
      for (let i = 0; i < 6; i++) {
        // TODO Check if some rotations are needed for ENU
        bitsPointers[i] = arrayXPointer(this.cubeArray[i].bits);
        _wr_texture_cubemap_set_data(this.cubeMapTexture, bitsPointers[i], i);
      }

      _wr_texture_set_size(this.cubeMapTexture, this.cubeArray[0].width, this.cubeArray[0].height);
      _wr_texture_setup(this.cubeMapTexture);
      _wr_material_set_texture_cubemap(this.skyboxMaterial, this.cubeMapTexture, 0);
      _wr_material_set_texture_cubemap_wrap_r(this.skyboxMaterial, 0x812F, 0); //enum
      _wr_material_set_texture_cubemap_wrap_s(this.skyboxMaterial, 0x812F, 0); //enum
      _wr_material_set_texture_cubemap_wrap_t(this.skyboxMaterial, 0x812F, 0); //enum
      _wr_scene_set_skybox(_wr_scene_get_instance(), this.skyboxRenderable);

      for(let i = 0; i < 6; i++) {
        _free(bitsPointers[i]);
      }
    }

    // 2. Load the irradiance map.
    /*let cm = _wr_texture_cubemap_new();

    try {
      // Check first that every fields are present.
      bool allUrlDefined = true;
      bool atLeastOneUrlDefined = false;
      for (int i = 0; i < 6; ++i) {
        if (mIrradianceUrlFields[i]->size() == 0) {
          allUrlDefined = false;
          continue;
        } else
          atLeastOneUrlDefined = true;
      }
      if (!allUrlDefined)
        throw tr(atLeastOneUrlDefined ? "Incomplete irradiance cubemap" : "");

      // Actually load the irradiance map.
      int w, h, components;
      for (int i = 0; i < 6; ++i) {
        QString url = WbUrl::computePath(this, "textureBaseName", mIrradianceUrlFields[gCoordinateSystemSwap(i)]->item(0), false);
        if (url.isEmpty())
          throw QString();

        _wr_texture_set_internal_format(cm, 9); //enum
        float *data = stbi_loadf(url.toUtf8().constData(), &w, &h, &components, 0);
        const int rotate = gCoordinateSystemRotate(i);
        // FIXME: this texture rotation should be performed by OpenGL or in the shader to get a better performance
        if (rotate != 0) {
          float *rotated = (float *)stbi__malloc(sizeof(float) * w * h * components);
          if (rotate == 90) {
            for (int x = 0; x < w; x++) {
              for (int y = 0; y < h; y++) {
                const int u = y * w * components + x * components;
                const int v = (w - 1 - x) * w * components + y * components;
                for (int c = 0; c < components; c++)
                  rotated[u + c] = data[v + c];
              }
            }
            const int swap = w;
            w = h;
            h = swap;
          } else if (rotate == -90) {
            for (int x = 0; x < w; x++) {
              for (int y = 0; y < h; y++) {
                const int u = y * w * components + x * components;
                const int v = x * w * components + (h - 1 - y) * components;
                for (int c = 0; c < components; c++)
                  rotated[u + c] = data[v + c];
              }
            }
            const int swap = w;
            w = h;
            h = swap;
          } else if (rotate == 180) {
            for (int x = 0; x < w; x++) {
              for (int y = 0; y < h; y++) {
                const int u = y * w * components + x * components;
                const int v = (h - 1 - y) * w * components + (w - 1 - x) * components;
                for (int c = 0; c < components; c++)
                  rotated[u + c] = data[v + c];
              }
            }
          }
          stbi_image_free(data);
          data = rotated;
        }
        _wr_texture_cubemap_set_data(cm, data, i);
        hdrImageData << data;
      }

      wr_texture_set_size(WR_TEXTURE(cm), w, h);
      wr_texture_set_texture_unit(WR_TEXTURE(cm), 13);
      wr_texture_setup(WR_TEXTURE(cm));

      mIrradianceCubeTexture =
        wr_texture_cubemap_bake_specular_irradiance(cm, WbWrenShaders::iblSpecularIrradianceBakingShader(), w);
      wr_texture_cubemap_disable_automatic_mip_map_generation(mIrradianceCubeTexture);

    } catch (QString &error) {
      if (error.length() > 0)
        parsingWarn(error);

      if (mIrradianceCubeTexture) {
        wr_texture_delete(WR_TEXTURE(mIrradianceCubeTexture));
        mIrradianceCubeTexture = NULL;
      }

      // Fallback: a cubemap is found but no irradiance map: bake a small irradiance map to have right colors.
      // Reflections won't be good in such case.
      if (mCubeMapTexture) {
        mIrradianceCubeTexture =
          wr_texture_cubemap_bake_specular_irradiance(mCubeMapTexture, WbWrenShaders::iblSpecularIrradianceBakingShader(), 64);
        wr_texture_cubemap_disable_automatic_mip_map_generation(mIrradianceCubeTexture);
      }
    }

    wr_texture_delete(WR_TEXTURE(cm));


    while (hdrImageData.size() > 0)
      stbi_image_free(hdrImageData.takeFirst());*/
  }
}

class WbGroup extends WbBaseNode{
  constructor(id){
    super(id);
    this.children = [];
  }

  createWrenObjects(){
    super.createWrenObjects();
  }
}

//Is also used to represent a solid
class WbTransform extends WbGroup {
  constructor(id, isSolid, translation, scale, rotation) {
    super(id);
    this.isSolid = isSolid;
    this.translation = translation;
    this.scale = scale;
    this.rotation = rotation;
  }

  createWrenObjects() {
    super.createWrenObjects();
    let transform = _wr_transform_new();

    _wr_transform_attach_child(this.wrenNode, transform);
    this.wrenNode = transform;

    this.children.forEach(child => {
      child.createWrenObjects()
    });

    this.applyTranslationToWren();
    this.applyRotationToWren();
    this.applyScaleToWren();
  }

  applyTranslationToWren() {
    let translation = _wrjs_color_array(this.translation.x, this.translation.y, this.translation.z);
    _wr_transform_set_position(this.wrenNode, translation);
  }

  applyRotationToWren() {
    let rotation = _wrjs_array4(this.rotation.w, this.rotation.x, this.rotation.y, this.rotation.z);
    _wr_transform_set_orientation(this.wrenNode, rotation);
  }

  applyScaleToWren() {
    let scale = _wrjs_color_array(this.scale.x, this.scale.y, this.scale.z);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  //TODO add children (in group)
}

class WbShape extends WbBaseNode {
  constructor(id, castShadow, geometry, appearance) {
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
      this.geometry.createWrenObjects();
      //not sure of the place
      this.applyMaterialToGeometry()
    }
  }

  applyMaterialToGeometry() {
    if (!this.wrenMaterial)
      this.createWrenMaterial(1); //enum
    if (this.geometry) {
      if (this.appearance instanceof WbAppearance) {
        if (this.appearance.wrenObjectsCreatedCalled)
          this.wrenMaterial = this.appearance.modifyWrenMaterial(this.wrenMaterial);
        else
          this.wrenMaterial = WbAppearance.fillWrenDefaultMaterial(this.wrenMaterial);
      } else if (this.appearance instanceof WbPBRAppearance && !(this.geometry instanceof WbPointSet)) {
        this.createWrenMaterial(2);
        if (this.appearance.wrenObjectsCreatedCalled){
          this.wrenMaterial = this.appearance.modifyWrenMaterial(this.wrenMaterial);
        }
      } else {
        this.wrenMaterial = WbAppearance.fillWrenDefaultMaterial(this.wrenMaterial);
      }
    }

    this.geometry.setWrenMaterial(this.wrenMaterial, this.castShadow);
  }

  createWrenMaterial(type) {
    let defaultColor = _wrjs_color_array(1.0, 1.0, 1.0);
    if (this.wrenMaterial)
      _wr_material_delete(this.wrenMaterial);

    if (type === 1) { //enum
      this.wrenMaterial = _wr_phong_material_new();
      _wr_phong_material_set_color(this.wrenMaterial, defaultColor);
      _wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.defaultShader());
    } else {
      this.wrenMaterial = _wr_pbr_material_new();
      _wr_pbr_material_set_base_color(this.wrenMaterial, defaultColor);
      _wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.pbrShader());
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

class WbBox extends WbGeometry {
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

class WbSphere extends WbGeometry {
  constructor(id, radius, ico, subdivision) {
    super(id);
    this.radius = radius;
    this.ico = ico;
    this.subdivision = subdivision;
  }

  createWrenObjects() {
    super.createWrenObjects();
    super.computeWrenRenderable();

    let wrenMesh = _wr_static_mesh_unit_sphere_new(this.subdivision, this.ico, false);

    _wr_renderable_set_mesh(this.wrenRenderable, wrenMesh);

    this.updateScale();
  }

  updateScale() {
    let scaledRadius = this.radius;

    _wr_transform_set_scale(this.wrenNode, _wrjs_color_array(scaledRadius, scaledRadius, scaledRadius));
  }

}

class WbCone extends WbGeometry {
  constructor(id, bottomRadius, height, subdivision, side, bottom) {
    super(id);
    this.bottomRadius = bottomRadius;
    this.height = height;
    this.subdivision = subdivision;
    this.side = side;
    this.bottom = bottom;
  }

  createWrenObjects() {
    super.createWrenObjects();

    if (!this.bottom && !this.side)
        return;

    this.computeWrenRenderable();

    let wrenMesh = _wr_static_mesh_unit_cone_new(this.subdivision, this.side , this.bottom);

    _wr_renderable_set_mesh(this.wrenRenderable, wrenMesh);

    let scale = _wrjs_color_array(this.bottomRadius, this.height, this.bottomRadius);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

}

class WbPlane extends WbGeometry{
  constructor(id, size) {
    super(id);
    this.size = size;
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.computeWrenRenderable();

    let wrenMesh = _wr_static_mesh_unit_rectangle_new(false);

    _wr_renderable_set_mesh(this.wrenRenderable, wrenMesh);

    this.updateSize();
  }

  updateSize() {
    // allow the bounding sphere to scale down
    let scaleY = 0.1 * Math.min(this.size.x, this.size.y);

    let scale = _wrjs_color_array(this.size.x, scaleY, this.size.y);
    _wr_transform_set_scale(this.wrenNode, scale);
  }
}

class WbCylinder extends WbGeometry {
  constructor (id, radius, height, subdivision, bottom, side, top){
    super(id);
    this.radius = radius;
    this.height = height;
    this.subdivision = subdivision;
    this.bottom = bottom;
    this.side = side;
    this.top = top;
  }

  createWrenObjects() {
    super.createWrenObjects();

    if(this.subdivision < 3)
      this.subdivision = 3;

    if (!this.bottom && !this.side && !this.top)
      return;

    this.computeWrenRenderable();

    let wrenMesh = _wr_static_mesh_unit_cylinder_new(this.subdivision, this.side, this.top, this.bottom, false);

    _wr_renderable_set_mesh(this.wrenRenderable, wrenMesh);

    let scale = _wrjs_color_array(this.radius, this.height, this.radius);
    _wr_transform_set_scale(this.wrenNode, scale);
  }
}

class WbPointSet extends WbGeometry {

}


class WbAbstractAppearance extends WbBaseNode {
  constructor(id, transform){
    super(id);
    this.textureTransform = transform;
  }

  createWrenObjects(){
    super.createWrenObjects();
    if(typeof this.textureTransform !== 'undefined') {
      this.textureTransform.createWrenObjects();
    }
  }

}

class WbAppearance extends WbAbstractAppearance {
  constructor(id, material, texture, transform){
    super(id, transform);
    this.material = material;
    this.texture = texture;
  }

  createWrenObjects(){
    super.createWrenObjects();
    if(typeof this.material !== 'undefined') {
      this.material.createWrenObjects();
    }

    if(typeof this.texture !== 'undefined') {
      this.texture.createWrenObjects();
    }
  }

  modifyWrenMaterial(wrenMaterial) {
    if(typeof this.material !== 'undefined') {
      _wr_material_set_default_program(wrenMaterial, WbWrenShaders.phongShader());
      _wr_material_set_stencil_ambient_emissive_program(wrenMaterial, WbWrenShaders.phongStencilAmbientEmissiveShader());
      _wr_material_set_stencil_diffuse_specular_program(wrenMaterial, WbWrenShaders.phongStencilDiffuseSpecularShader());

      this.material.modifyWrenMaterial(wrenMaterial, this.texture && this.texture.wrenTexture);
    } else
      wrenMaterial = WbAppearance.fillWrenDefaultMaterial(wrenMaterial);


    if (this.texture)
      this.texture.modifyWrenMaterial(wrenMaterial, 0, 2);
    else
      _wr_material_set_texture(wrenMaterial, null, 0);

    if (this.textureTransform)
      this.textureTransform.modifyWrenMaterial(wrenMaterial);
    else
      _wr_material_set_texture_transform(wrenMaterial, null);

    return wrenMaterial;
  }

  static fillWrenDefaultMaterial(wrenMaterial) {
    console.log("coucou");
    //TODO add suport if not a phong material
    if (!wrenMaterial) {
      _wr_material_delete(wrenMaterial);
      wrenMaterial = _wr_phong_material_new();
    }
    //Replace default by phongShader
    _wr_material_set_default_program(wrenMaterial, WbWrenShaders.phongShader());
    return wrenMaterial;
  }
}

class WbMaterial extends WbBaseNode {
  constructor(id, ambientIntensity, diffuseColor, specularColor, emissiveColor, shininess, transparency) {
    super(id);
    this.ambientIntensity = ambientIntensity;
    this.diffuseColor = diffuseColor;
    this.specularColor = specularColor;
    this.emissiveColor = emissiveColor;
    this.shininess = shininess;
    this.transparency = transparency;
  }

  modifyWrenMaterial(wrenMaterial, textured) {
    let ambient, diffuse, specular, shininess;

    ambient = glm.vec3(this.ambientIntensity, this.ambientIntensity, this.ambientIntensity);

    if (textured) {
      diffuse = glm.vec3(1.0, 1.0, 1.0);
      specular = glm.vec3(1.0, 1.0, 1.0);
      shininess = 0.0;
    } else {
      ambient = glm.vec3(this.ambientIntensity * this.diffuseColor.x,this.ambientIntensity * this.diffuseColor.y,
                       this.ambientIntensity * this.diffuseColor.z);
      diffuse = glm.vec3(this.diffuseColor.x, this.diffuseColor.y, this.diffuseColor.z);
      specular = glm.vec3(this.specularColor.x, this.specularColor.y, this.specularColor.z);
      shininess = this.shininess;
    }

    let ambientColorPointer = array3Pointer(ambient.x, ambient.y, ambient.z);
    let diffuseColorPointer = array3Pointer(diffuse.x, diffuse.y, diffuse.z);
    let specularColorPointer = array3Pointer(specular.x, specular.y, specular.z);
    let emissiveColorPointer = array3Pointer(this.emissiveColor.x, this.emissiveColor.y, this.emissiveColor.z);
    _wr_phong_material_set_all_parameters(wrenMaterial, ambientColorPointer, diffuseColorPointer, specularColorPointer, emissiveColorPointer, shininess, this.transparency);

    _free(ambientColorPointer);
    _free(diffuseColorPointer);
    _free(specularColorPointer);
    _free(emissiveColorPointer);
  }
}

class WbImageTexture extends WbBaseNode {
  constructor(id, url, isTransparent, s, t, anisotropy, image){
    super(id);
    this.url = url;

    this.isTransparent = isTransparent;
    this.repeatS = s;
    this.repeatT = t;

    this.anisotropy = anisotropy;
    this.wrenTextureIndex = 0;
    this.usedFiltering = 0
    this.image = image;
    this.wrenTexture = undefined;
    this.wrenTextureTransform = undefined;
    this.wrenBackgroundTexture = undefined;
    this.externalTexture = false;
    this.externalTextureRatio = glm.vec2(1.0,1.0);

    this.updateWrenTexture();
  }

  modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex) {
    if (!wrenMaterial)
      return;

    this.wrenTextureIndex = mainTextureIndex;
    _wr_material_set_texture(wrenMaterial, this.wrenTexture, this.wrenTextureIndex);
    if (this.wrenTexture) {
      _wr_texture_set_translucent(this.wrenTexture, this.isTransparent);
      _wr_material_set_texture_wrap_s(
        wrenMaterial, this.repeatS ? 0x2901 : 0x812F, this.wrenTextureIndex);//enum 2x
      _wr_material_set_texture_wrap_t(
        wrenMaterial, this.repeatT ? 0x2901 : 0x812F, this.wrenTextureIndex);//enum2x
      _wr_material_set_texture_anisotropy(wrenMaterial, 1 << (this.usedFiltering  - 1), this.wrenTextureIndex);
      _wr_material_set_texture_enable_interpolation(wrenMaterial, this.usedFiltering , this.wrenTextureIndex);
      _wr_material_set_texture_enable_mip_maps(wrenMaterial, this.usedFiltering, this.wrenTextureIndex);

      if (this.externalTexture && ! this.parentNode().textureTransform) {
        _wr_texture_transform_delete(this.wrenTextureTransform);
        this.wrenTextureTransform = _wr_texture_transform_new();
        _wr_texture_transform_set_scale(this.wrenTextureTransform, this.externalTextureRatio.x, this.externalTextureRatio.y);
        _wr_material_set_texture_transform(wrenMaterial, this.wrenTextureTransform);
      }
    }

    _wr_material_set_texture(wrenMaterial, this.wrenBackgroundTexture, backgroundTextureIndex);
    if (typeof this.wrenBackgroundTexture !== 'undefined') {
      // background texture can't be transparent
      _wr_texture_set_translucent(this.wrenBackgroundTexture, false);

      // if there's an opaque background texture, we can't treat the foreground texture as opaque, as we're going to alpha blend
      // them in the shader anyway
      if (typeof this.wrenTexture !== 'undefined')
        _wr_texture_set_translucent(this.wrenTexture, false);

      _wr_material_set_texture_wrap_s(wrenMaterial,
                                     this.repeatS ? 0x2901 : 0x812F,
                                     backgroundTextureIndex); //enum x2
      _wr_material_set_texture_wrap_t(wrenMaterial,
                                     this.repeatT ? 0x2901 : 0x812F,
                                     backgroundTextureIndex); //enum x2
      _wr_material_set_texture_enable_interpolation(wrenMaterial, false, backgroundTextureIndex);
      _wr_material_set_texture_enable_mip_maps(wrenMaterial, false, backgroundTextureIndex);
    }
  }

  updateWrenTexture() {
    this.destroyWrenTexture();
    // Only load the image from disk if the texture isn't already in the cache
    let texture = Module.ccall('wr_texture_2d_copy_from_cache', 'number', ['string'], [this.url]);
    if (texture === 0) {
      texture = _wr_texture_2d_new();
      _wr_texture_set_size(texture, this.image.width, this.image.height);
      _wr_texture_set_translucent(texture, this.isTransparent);
      let bitsPointer = arrayXPointer(this.image.bits);
      _wr_texture_2d_set_data(texture, bitsPointer);
      _free(bitsPointer);
      Module.ccall('wr_texture_2d_set_file_path', null, ['number', 'string'], [texture, this.url]);
      _wr_texture_setup(texture);

    } else
      this.isTransparent = _wr_texture_is_translucent(texture);

      this.wrenTexture = texture;
  }

  destroyWrenTexture() {
    if (typeof this.externalTexture != 'undefined')
      _wr_texture_delete(this.wrenTexture);

    _wr_texture_transform_delete(this.wrenTextureTransform);

    this.wrenTexture = undefined;
    this.wrenTextureTransform = undefined;

    //TODO see how to delete js image
    //delete mImage;
    //this.image = undefined;
  }
}

class WbTextureTransform extends WbBaseNode{
  constructor(id, center, rotation, scale, translation){
    super(id);
    this.center = center;
    this.rotation = rotation;
    this.scale = scale;
    this.translation = translation;
    this.wrenTextureTransform;
  }

  modifyWrenMaterial(wrenMaterial) {
    this.destroyWrenObjects();

    // apply translation before rotation
    this.wrenTextureTransform = _wr_texture_transform_new();
    _wr_texture_transform_set_scale(this.wrenTextureTransform, this.scale.x, this.scale.y);
    _wr_texture_transform_set_position(this.wrenTextureTransform, this.translation.x, this.translation.y);
    _wr_texture_transform_set_center(this.wrenTextureTransform, this.center.x, this.center.y);
    _wr_texture_transform_set_rotation(this.wrenTextureTransform, this.rotation);

    _wr_material_set_texture_transform(wrenMaterial, this.wrenTextureTransform);
  }

  destroyWrenObjects() {
    if (this.wrenTextureTransform)
      _wr_texture_transform_delete(this.wrenTextureTransform);
  }
}

class WbImage {
  constructor() {
    this.bits = undefined;
    this.width = 0;
    this.height = 0;
  }
}

class WbPBRAppearance extends WbAbstractAppearance {
  constructor(id, baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness, metalnessMap,
    IBLStrength, normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor, emissiveColorMap, emissiveIntensity, textureTransform) {
    super(id, textureTransform);
    this.baseColor = baseColor;
    this.baseColorMap = baseColorMap;
    this.transparency = transparency;
    this.roughness = roughness;
    this.roughnessMap = roughnessMap;
    this.metalness = metalness;
    this.metalnessMap = metalnessMap;
    this.IBLStrength = IBLStrength;
    this.normalMap = normalMap;
    this.normalMapFactor = normalMapFactor;
    this.occlusionMap = occlusionMap;
    this.occlusionMapStrength = occlusionMapStrength;
    this.emissiveColor = emissiveColor;
    this.emissiveColorMap = emissiveColorMap;
    this.emissiveIntensity = emissiveIntensity;
    this.textureTransform = textureTransform;
    if (WbPBRAppearance.cInstanceCounter == 0) {
      let quality = 2;//TODO: WbPreferences::instance()->value("OpenGL/textureQuality", 2).toInt();
      let resolution = Math.pow(2, 6 + quality);  // 0: 64, 1: 128, 2: 256
      WbPBRAppearance.cBrdfTexture = _wr_texture_cubemap_bake_brdf(WbWrenShaders.iblBrdfBakingShader(), resolution);
    }
    ++WbPBRAppearance.cInstanceCounter;
  }

  createWrenObjects(){
    super.createWrenObjects();
    if (typeof this.baseColorMap !== 'undefined')
      this.baseColorMap.createWrenObjects();

    if (typeof this.roughnessMap !== 'undefined')
      this.roughnessMap.createWrenObjects();

    if (typeof this.metalnessMap !== 'undefined')
      this.metalnessMap.createWrenObjects();

    if (typeof this.normalMap !== 'undefined')
      this.normalMap.createWrenObjects();

    if (typeof this.occlusionMap !== 'undefined')
      this.occlusionMap.createWrenObjects();

    if (typeof this.emissiveColorMap !== 'undefined')
      this.emissiveColorMap.createWrenObjects();
  }

  modifyWrenMaterial(wrenMaterial) {
    if (!wrenMaterial) {//enum //TODO check if not pbr material
      _wr_material_delete(wrenMaterial);
      wrenMaterial = _wr_pbr_material_new();
    }

    // set up shaders
    _wr_material_set_default_program(wrenMaterial, WbWrenShaders.pbrShader());
    _wr_material_set_stencil_ambient_emissive_program(wrenMaterial, WbWrenShaders.pbrStencilAmbientEmissiveShader());
    _wr_material_set_stencil_diffuse_specular_program(wrenMaterial, WbWrenShaders.pbrStencilDiffuseSpecularShader());

    // apply textures
    if (typeof this.baseColorMap !== 'undefined')
      this.baseColorMap.modifyWrenMaterial(wrenMaterial, 0, 7);

    if (typeof this.roughnessMap !== 'undefined')
      this.roughnessMap.modifyWrenMaterial(wrenMaterial, 1, 7);

    if (typeof this.metalnessMap !== 'undefined')
      this.metalnessMap.modifyWrenMaterial(wrenMaterial, 2, 7);


    //let background = WbBackground::firstInstance();
    let backgroundLuminosity = 1.0;
    /*if (background) {
      backgroundLuminosity = background->luminosity();
      connect(background, &WbBackground::luminosityChanged, this, &WbPbrAppearance::updateCubeMap, Qt::UniqueConnection);

      // irradiance map
      WrTextureCubeMap *irradianceCubeTexture = background->irradianceCubeTexture();
      if (irradianceCubeTexture) {
        wr_material_set_texture_cubemap(wrenMaterial, irradianceCubeTexture, 0);
        wr_material_set_texture_cubemap_wrap_r(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        wr_material_set_texture_cubemap_wrap_s(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        wr_material_set_texture_cubemap_wrap_t(wrenMaterial, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
        wr_material_set_texture_cubemap_anisotropy(wrenMaterial, 8, 0);
        wr_material_set_texture_cubemap_enable_interpolation(wrenMaterial, true, 0);
        wr_material_set_texture_cubemap_enable_mip_maps(wrenMaterial, true, 0);
      } else
        wr_material_set_texture_cubemap(wrenMaterial, NULL, 0);

      connect(background, &WbBackground::cubemapChanged, this, &WbPbrAppearance::updateCubeMap, Qt::UniqueConnection);
    } else*/
      _wr_material_set_texture_cubemap(wrenMaterial, null, 0);

    if (typeof this.normalMap !== 'undefined')
      this.normalMap.modifyWrenMaterial(wrenMaterial, 4, 7);

    if (typeof this.occlusionMap !== 'undefined')
      this.occlusionMap.modifyWrenMaterial(wrenMaterial, 3, 7);

    if (typeof this.emissiveColorMap !== 'undefined'){
      this.emissiveColorMap.modifyWrenMaterial(wrenMaterial, 6, 7);
    }
    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.modifyWrenMaterial(wrenMaterial);
    else
      _wr_material_set_texture_transform(wrenMaterial, null);

    _wr_material_set_texture(wrenMaterial, WbPBRAppearance.cBrdfTexture, 5);
    _wr_material_set_texture_enable_mip_maps(wrenMaterial, false, 5);
    _wr_material_set_texture_enable_interpolation(wrenMaterial, false, 5);

    let baseColorPointer = array3Pointer(this.baseColor.x, this.baseColor.y, this.baseColor.z);
    let emissiveColorPointer = array3Pointer(this.emissiveColor.x, this.emissiveColor.y, this.emissiveColor.z);


    let backgroundColor = glm.vec3(0.0, 0.0, 0.0);

    /*if (background) {
      WbRgb skyColor = background->skyColor();
      backgroundColor[0] = static_cast<float>(skyColor.red());
      backgroundColor[1] = static_cast<float>(skyColor.green());
      backgroundColor[2] = static_cast<float>(skyColor.blue());
    }*/

    let backgroundColorPointer = array3Pointer(backgroundColor.x, backgroundColor.y, backgroundColor.z);

    // set material properties
    _wr_pbr_material_set_all_parameters(wrenMaterial, backgroundColorPointer, baseColorPointer,
      this.transparency, this.roughness, this.metalness, backgroundLuminosity * this.IBLStrength,this.normalMapFactor,
      this.occlusionMapStrength, emissiveColorPointer,this.emissiveIntensity);

    return wrenMaterial;
  }
}

WbPBRAppearance.cBrdfTexture = undefined;
WbPBRAppearance.cInstanceCounter = 0;

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

  static phongShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 0);  // main texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 1);  // pen texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 2);  // background texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 16);//enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 17);//enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 0);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 2);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 4);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], 5);//enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG], "../../../resources/wren/shaders/phong.vert", "../../../resources/wren/shaders/phong.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG];
  }

  static phongStencilAmbientEmissiveShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.HADER_PHONG_STENCIL_AMBIENT_EMISSIVE]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], 0); // main texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], 1); // pen texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], 2); // background texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], 16); //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], 17);//enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], 0);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], 2);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], 4);//enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE], "../../../resources/wren/shaders/phong_stencil_ambient_emissive.vert", "../../../resources/wren/shaders/phong_stencil_ambient_emissive.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_AMBIENT_EMISSIVE];
  }

  static phongStencilDiffuseSpecularShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 0);// main texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 1);// pen texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 2);// background texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 16);//enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 17);//enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 0);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 2);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 3);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], 4);//enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR], "../../../resources/wren/shaders/phong_stencil_diffuse_specular.vert", "../../../resources/wren/shaders/phong_stencil_diffuse_specular.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PHONG_STENCIL_DIFFUSE_SPECULAR];
  }

  static pbrShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 0);  // base color texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 1);  // roughness texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 2);  // metalness texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 3);  // occlusion map //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 4);  // normal map //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 5);  // BRDF LUT //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 6);  // emissive texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 7);  // background texture (for displays) //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 8);  // pen texture //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 13);  // irradiance cubemap //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 16); //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 17); //enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 1); //enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 2);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 4); //enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], 5); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR], "../../../resources/wren/shaders/pbr.vert", "../../../resources/wren/shaders/pbr.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR];
  }

  static pbrStencilAmbientEmissiveShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE] = _wr_shader_program_new();

      // base color texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 0); //enum
      // roughness texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 1); //enum
      // metalness texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 2); //enum
      // occlusion map
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 3); //enum
      // normal map
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 4); //enum
      // BRDF LUT
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 5); //enum
      // emissive texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 6); //enum
      // background texture (for displays)
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 7); //enum
      // pen texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 8); //enum
      // irradiance cubemap
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 13); //enum
      // specular cubemap
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 16); //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 17); //enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 1); //enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], 4); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE], "../../../resources/wren/shaders/pbr_stencil_ambient_emissive.vert", "../../../resources/wren/shaders/pbr_stencil_ambient_emissive.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_AMBIENT_EMISSIVE];
  }

  static pbrStencilDiffuseSpecularShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR] = _wr_shader_program_new();

      // base color texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 0); //enum
      // roughness texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 1); //enum
      // metalness texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 2); //enum
      // occlusion map
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 3); //enum
      // normal map
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 4); //enum
      // background texture (for displays)
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 7); //enum
      // pen texture
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 8); //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 16); //enum
      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 17); //enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 1); //enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 2); //enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 3); //enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], 4); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR], "../../../resources/wren/shaders/pbr_stencil_diffuse_specular.vert", "../../../resources/wren/shaders/pbr_stencil_diffuse_specular.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PBR_STENCIL_DIFFUSE_SPECULAR];
  }

  static encodeDepthShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], 16); //enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], 4); //enum

      let minRange = 0.0;
      Module.ccall('wr_shader_program_create_custom_uniform', null, ['number', 'string', 'number', 'number'], [WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "minRange", 0, _wrjs_pointerOnFloat(minRange)]); //enum

      let maxRange = 1.0;
      Module.ccall('wr_shader_program_create_custom_uniform', null, ['number', 'string', 'number', 'number'], [WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "maxRange", 0, _wrjs_pointerOnFloat(maxRange)]); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "../../../resources/wren/shaders/encode_depth.vert", "../../../resources/wren/shaders/encode_depth.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH];
  }


  static fogShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_FOG]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_FOG] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_FOG], 16); //enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_FOG], 4);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_FOG], 5);//enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_FOG], "../../../resources/wren/shaders/fog.vert", "../../../resources/wren/shaders/fog.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_FOG];
  }

  static iblBrdfBakingShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_IBL_BRDF_BAKE]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_IBL_BRDF_BAKE] = _wr_shader_program_new();

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_IBL_BRDF_BAKE], "../../../resources/wren/shaders/bake_brdf.vert", "../../../resources/wren/shaders/bake_brdf.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_IBL_BRDF_BAKE];
  }

  static shadowVolumeShader() {
    if (! WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME]) {
       WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], 16);//enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], 2);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], 3);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], 4);//enum

       WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], "../../../resources/wren/shaders/shadow_volume.vert", "../../../resources/wren/shaders/shadow_volume.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME];
  }

  static hdrResolveShader() {
    if (! WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_RESOLVE]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_RESOLVE] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_RESOLVE], 0);//enum

      const defaultExposureValue = 1.0;
      Module.ccall('wr_shader_program_create_custom_uniform', null, ['number', 'string', 'number', 'number'], [WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_RESOLVE], "exposure", 0, _wrjs_pointerOnFloat(defaultExposureValue)]); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_RESOLVE], "../../../resources/wren/shaders/pass_through.vert", "../../../resources/wren/shaders/hdr_resolve.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_RESOLVE];
  }

  static hdrClearShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_CLEAR]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_CLEAR] = _wr_shader_program_new();

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_CLEAR], 0); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_CLEAR], "../../../resources/wren/shaders/pass_through.vert", "../../../resources/wren/shaders/hdr_clear.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_HDR_CLEAR];
  }

  static skyboxShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SKYBOX]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SKYBOX] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SKYBOX], 13); //enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SKYBOX], 4); //enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SKYBOX], "../../../resources/wren/shaders/skybox.vert", "../../../resources/wren/shaders/skybox.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SKYBOX];
  }

  static passThroughShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PASS_THROUGH]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PASS_THROUGH] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PASS_THROUGH], 0);//enum

      WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PASS_THROUGH], "../../../resources/wren/shaders/pass_through.vert", "../../../resources/wren/shaders/pass_through.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_PASS_THROUGH];
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

class WbWrenPostProcessingEffects {
  static loadResources(){
    console.log("TODO");
  }

  /*static gtao(width, height, textureFormat, depthTexture, normalTexture, halfRes) {
    let gtaoEffect = _wr_post_processing_effect_new();
    _wr_post_processing_effect_set_drawing_index(gtaoEffect, 0);//enum

    let colorPassThrough = _wr_post_processing_effect_pass_new();
    _wr_post_processing_effect_pass_set_name(colorPassThrough, "colorPassThrough");
    _wr_post_processing_effect_pass_set_program(colorPassThrough, WbWrenShaders::passThroughShader());
    _wr_post_processing_effect_pass_set_output_size(colorPassThrough, width, height);
    _wr_post_processing_effect_pass_set_alpha_blending(colorPassThrough, false);
    _wr_post_processing_effect_pass_set_input_texture_count(colorPassThrough, 1);
    _wr_post_processing_effect_pass_set_input_texture_interpolation(colorPassThrough, 0, false);
    _wr_post_processing_effect_pass_set_output_texture_count(colorPassThrough, 1);
    _wr_post_processing_effect_pass_set_output_texture_format(colorPassThrough, 0, textureFormat);
    _wr_post_processing_effect_append_pass(gtaoEffect, colorPassThrough);

    WrPostProcessingEffectPass *depthDownsamplePassThrough, *normalDownsamplePassThrough = NULL;
    if (halfRes) {
      depthDownsamplePassThrough = wr_post_processing_effect_pass_new();
      wr_post_processing_effect_pass_set_name(depthDownsamplePassThrough, "depthDownsamplePassThrough");
      wr_post_processing_effect_pass_set_program(depthDownsamplePassThrough, WbWrenShaders::passThroughShader());
      wr_post_processing_effect_pass_set_output_size(depthDownsamplePassThrough, width / 2, height / 2);
      wr_post_processing_effect_pass_set_input_texture_count(depthDownsamplePassThrough, 1);
      wr_post_processing_effect_pass_set_alpha_blending(depthDownsamplePassThrough, false);
      wr_post_processing_effect_pass_set_input_texture(depthDownsamplePassThrough, 0, WR_TEXTURE(depthTexture));
      wr_post_processing_effect_pass_set_input_texture_interpolation(depthDownsamplePassThrough, 0, false);
      wr_post_processing_effect_pass_set_output_texture_count(depthDownsamplePassThrough, 1);
      wr_post_processing_effect_pass_set_output_texture_format(depthDownsamplePassThrough, 0, WR_TEXTURE_INTERNAL_FORMAT_R32F);
      wr_post_processing_effect_append_pass(gtaoEffect, depthDownsamplePassThrough);

      normalDownsamplePassThrough = wr_post_processing_effect_pass_new();
      wr_post_processing_effect_pass_set_name(normalDownsamplePassThrough, "normalDownsamplePassThrough");
      wr_post_processing_effect_pass_set_program(normalDownsamplePassThrough, WbWrenShaders::passThroughShader());
      wr_post_processing_effect_pass_set_alpha_blending(normalDownsamplePassThrough, false);
      wr_post_processing_effect_pass_set_output_size(normalDownsamplePassThrough, width / 2, height / 2);
      wr_post_processing_effect_pass_set_input_texture_count(normalDownsamplePassThrough, 1);
      wr_post_processing_effect_pass_set_input_texture(normalDownsamplePassThrough, 0, WR_TEXTURE(normalTexture));
      wr_post_processing_effect_pass_set_input_texture_interpolation(normalDownsamplePassThrough, 0, false);
      wr_post_processing_effect_pass_set_output_texture_count(normalDownsamplePassThrough, 1);
      wr_post_processing_effect_pass_set_output_texture_format(normalDownsamplePassThrough, 0, textureFormat);
      wr_post_processing_effect_append_pass(gtaoEffect, normalDownsamplePassThrough);
    }

    WrPostProcessingEffectPass *gtaoForwardPass = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(gtaoForwardPass, "gtaoForwardPass");
    wr_post_processing_effect_pass_set_program(gtaoForwardPass, WbWrenShaders::gtaoShader());
    wr_post_processing_effect_pass_set_input_texture_count(gtaoForwardPass, 3);

    if (halfRes)
      wr_post_processing_effect_pass_set_output_size(gtaoForwardPass, width / 2, height / 2);
    else {
      wr_post_processing_effect_pass_set_output_size(gtaoForwardPass, width, height);
      wr_post_processing_effect_pass_set_input_texture(gtaoForwardPass, 0, WR_TEXTURE(depthTexture));
      wr_post_processing_effect_pass_set_input_texture(gtaoForwardPass, 1, WR_TEXTURE(normalTexture));
    }

    wr_post_processing_effect_pass_set_input_texture(gtaoForwardPass, 2, WR_TEXTURE(gtaoNoiseTexture));
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(gtaoForwardPass, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(gtaoForwardPass, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(gtaoForwardPass, 2, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_interpolation(gtaoForwardPass, 0, false);
    wr_post_processing_effect_pass_set_input_texture_interpolation(gtaoForwardPass, 1, false);
    wr_post_processing_effect_pass_set_input_texture_interpolation(gtaoForwardPass, 2, false);
    wr_post_processing_effect_pass_set_clear_before_draw(gtaoForwardPass, true);
    wr_post_processing_effect_pass_set_alpha_blending(gtaoForwardPass, false);
    wr_post_processing_effect_pass_set_output_texture_count(gtaoForwardPass, 1);
    wr_post_processing_effect_pass_set_output_texture_format(gtaoForwardPass, 0, WR_TEXTURE_INTERNAL_FORMAT_RED);
    wr_post_processing_effect_append_pass(gtaoEffect, gtaoForwardPass);

    WrPostProcessingEffectPass *spatialDenoise = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(spatialDenoise, "spatialDenoise");
    wr_post_processing_effect_pass_set_program(spatialDenoise, WbWrenShaders::gtaoSpatialDenoiseShader());

    if (halfRes)
      wr_post_processing_effect_pass_set_output_size(spatialDenoise, width / 2, height / 2);
    else
      wr_post_processing_effect_pass_set_output_size(spatialDenoise, width, height);

    wr_post_processing_effect_pass_set_input_texture_count(spatialDenoise, 2);
    wr_post_processing_effect_pass_set_input_texture(spatialDenoise, 1, WR_TEXTURE(depthTexture));
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(spatialDenoise, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(spatialDenoise, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_interpolation(spatialDenoise, 0, true);
    wr_post_processing_effect_pass_set_input_texture_interpolation(spatialDenoise, 1, false);
    wr_post_processing_effect_pass_set_alpha_blending(spatialDenoise, false);
    wr_post_processing_effect_pass_set_clear_before_draw(spatialDenoise, true);
    wr_post_processing_effect_pass_set_output_texture_count(spatialDenoise, 1);
    wr_post_processing_effect_pass_set_output_texture_format(spatialDenoise, 0, WR_TEXTURE_INTERNAL_FORMAT_RED);
    wr_post_processing_effect_append_pass(gtaoEffect, spatialDenoise);

    WrPostProcessingEffectPass *temporalDenoise = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(temporalDenoise, "temporalDenoise");
    wr_post_processing_effect_pass_set_program(temporalDenoise, WbWrenShaders::gtaoTemporalDenoiseShader());
    wr_post_processing_effect_pass_set_output_size(temporalDenoise, width, height);
    wr_post_processing_effect_pass_set_input_texture_count(temporalDenoise, 4);
    wr_post_processing_effect_pass_set_input_texture(temporalDenoise, 3, WR_TEXTURE(depthTexture));
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(temporalDenoise, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(temporalDenoise, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(temporalDenoise, 2, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(temporalDenoise, 3, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_interpolation(temporalDenoise, 0, true);
    wr_post_processing_effect_pass_set_input_texture_interpolation(temporalDenoise, 1, true);
    wr_post_processing_effect_pass_set_input_texture_interpolation(temporalDenoise, 2, false);
    wr_post_processing_effect_pass_set_input_texture_interpolation(temporalDenoise, 3, false);
    wr_post_processing_effect_pass_set_alpha_blending(temporalDenoise, false);
    wr_post_processing_effect_pass_set_clear_before_draw(temporalDenoise, true);
    wr_post_processing_effect_pass_set_output_texture_count(temporalDenoise, 1);
    wr_post_processing_effect_pass_set_output_texture_format(temporalDenoise, 0, WR_TEXTURE_INTERNAL_FORMAT_RED);
    wr_post_processing_effect_append_pass(gtaoEffect, temporalDenoise);

    WrPostProcessingEffectPass *finalBlend = wr_post_processing_effect_pass_new();
    wr_post_processing_effect_pass_set_name(finalBlend, "FinalBlend");
    wr_post_processing_effect_pass_set_program(finalBlend, WbWrenShaders::gtaoCombineShader());
    wr_post_processing_effect_pass_set_output_size(finalBlend, width, height);
    wr_post_processing_effect_pass_set_input_texture_count(finalBlend, 3);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(finalBlend, 0, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(finalBlend, 1, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_wrap_mode(finalBlend, 2, WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE);
    wr_post_processing_effect_pass_set_input_texture_interpolation(finalBlend, 0, false);
    wr_post_processing_effect_pass_set_input_texture_interpolation(finalBlend, 1, false);
    wr_post_processing_effect_pass_set_input_texture_interpolation(finalBlend, 2, false);
    wr_post_processing_effect_pass_set_clear_before_draw(finalBlend, true);
    wr_post_processing_effect_pass_set_input_texture(finalBlend, 2, WR_TEXTURE(depthTexture));
    wr_post_processing_effect_pass_set_output_texture_count(finalBlend, 3);
    wr_post_processing_effect_pass_set_output_texture_format(finalBlend, 0, textureFormat);
    wr_post_processing_effect_pass_set_output_texture_format(finalBlend, 1, WR_TEXTURE_INTERNAL_FORMAT_RED);
    wr_post_processing_effect_pass_set_output_texture_format(finalBlend, 2, WR_TEXTURE_INTERNAL_FORMAT_R32F);
    wr_post_processing_effect_append_pass(gtaoEffect, finalBlend);

    // color texture for blending at the end
    wr_post_processing_effect_connect(gtaoEffect, colorPassThrough, 0, finalBlend, 0);

    // downsampled textures for half-res AO
    if (halfRes) {
      wr_post_processing_effect_connect(gtaoEffect, depthDownsamplePassThrough, 0, gtaoForwardPass, 0);
      wr_post_processing_effect_connect(gtaoEffect, normalDownsamplePassThrough, 0, gtaoForwardPass, 1);
    }

    // denoising
    wr_post_processing_effect_connect(gtaoEffect, gtaoForwardPass, 0, spatialDenoise, 0);
    wr_post_processing_effect_connect(gtaoEffect, spatialDenoise, 0, temporalDenoise, 1);
    wr_post_processing_effect_connect(gtaoEffect, temporalDenoise, 0, finalBlend, 1);

    // loopbacks for temporal
    wr_post_processing_effect_connect(gtaoEffect, finalBlend, 1, temporalDenoise, 0);
    wr_post_processing_effect_connect(gtaoEffect, finalBlend, 2, temporalDenoise, 2);

    wr_post_processing_effect_set_result_program(gtaoEffect, WbWrenShaders::passThroughShader());

    return gtaoEffect;
  }*/
}

//WbWrenPostProcessingEffects static variable
WbWrenPostProcessingEffects.gtaoNoiseTexture = null


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

function array3Pointer(x, y, z) {
  let data = new Float32Array([x, y, z]);
  let nDataBytes = data.length * data.BYTES_PER_ELEMENT;
  let dataPtr = Module._malloc(nDataBytes);
  let dataHeap = new Uint8Array(Module.HEAPU8.buffer, dataPtr, nDataBytes);
  dataHeap.set(new Uint8Array(data.buffer));

  return dataHeap.byteOffset;
}

function arrayXPointer(array) {
  let data = new Uint8ClampedArray(array);
  let nDataBytes = data.length * data.BYTES_PER_ELEMENT;
  let dataPtr = Module._malloc(nDataBytes);
  let dataHeap = new Uint8Array(Module.HEAPU8.buffer, dataPtr, nDataBytes);
  dataHeap.set(new Uint8Array(data.buffer));

  return dataHeap.byteOffset;
}
