class WbScene {
  constructor(id) {
    this.id = id;
    this.wrenMainFrameBuffer = null;
    this.wrenMainFrameBufferTexture = null;
    this.wrenNormalFrameBufferTexture = null;
    this.wrenDepthFrameBufferTexture = null;


    _wrjs_init_context(canvas.clientWidth, canvas.clientHeight);
    _wr_scene_init(_wr_scene_get_instance());

    _wr_gl_state_set_context_active(true);

    this.updateFrameBuffer();

    _wr_scene_set_fog_program(_wr_scene_get_instance(), WbWrenShaders.fogShader());
    _wr_scene_set_shadow_volume_program(_wr_scene_get_instance(), WbWrenShaders.shadowVolumeShader());

    //WbWrenPostProcessingEffects::loadResources();
    this.updateWrenViewportDimensions();
  }

  updateFrameBuffer() {/*
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
    _wr_texture_set_internal_format(this.wrenNormalFrameBufferTexture, 2);//enum

    _wr_frame_buffer_append_output_texture(this.wrenMainFrameBuffer, this.wrenMainFrameBufferTexture);
    _wr_frame_buffer_append_output_texture(this.wrenMainFrameBuffer, this.wrenNormalFrameBufferTexture);
    _wr_frame_buffer_enable_depth_buffer(this.wrenMainFrameBuffer, true);

    this.wrenDepthFrameBufferTexture = _wr_texture_rtt_new();
    _wr_texture_set_internal_format(this.wrenDepthFrameBufferTexture, 11);//enum
    _wr_frame_buffer_set_depth_texture(this.wrenMainFrameBuffer, this.wrenDepthFrameBufferTexture);

    _wr_frame_buffer_setup(this.wrenMainFrameBuffer);
    _wr_viewport_set_frame_buffer(_wr_scene_get_viewport(_wr_scene_get_instance()), this.wrenMainFrameBuffer);*/

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
      _wr_camera_set_far(this.wrenCamera, DEFAULT_FAR);
  }

  applyFieldOfViewToWren() {
    _wr_camera_set_fovy(this.wrenCamera, this.fieldOfViewY);
  }
}

WbViewpoint.DEFAULT_FAR = 1000000.0;

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

    this.children.forEach(child => child.createWrenObjects());

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
      if (this.appearance) {
        if (this.appearance.wrenObjectsCreatedCalled){
          this.wrenMaterial = this.appearance.modifyWrenMaterial(this.wrenMaterial);
        }
        else
        console.error("applyMaterialToGeometry : not implemented yet");
          //this.wrenMaterial = WbAppearance::fillWrenDefaultMaterial(this.wrenMaterial);
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
  //Not use for now, normaly is cone
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

class WbCylinder extends WbGeometry{
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


class WbAbstractAppearance extends WbBaseNode {
  constructor(id){
    super(id);
    this.textureTransform = undefined;
  }

  createWrenObjects(){
    super.createWrenObjects();

    if(typeof this.textureTransform !== 'undefined') {
      this.textureTransform.createWrenObjects();
    }
  }

}

class WbAppearance extends WbBaseNode {
  constructor(id, material, texture){
    super(id);
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
    } else{
      console.error("Appearance without material is not implemented yet");
      //wrenMaterial = fillWrenDefaultMaterial(wrenMaterial);
    }

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

  static encodeDepthShader() {
    if (!WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH]) {
      WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH] = _wr_shader_program_new();

      _wr_shader_program_use_uniform(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], 16); //enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], 4); //enum

      let minRange = 0.0;
      Module.ccall('wr_shader_program_create_custom_uniform', null, ['number, string, number, number'], [WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "minRange", 0, _wrjs_pointerOnFloat(minRange)]); //enum

      let maxRange = 1.0;
      Module.ccall('wr_shader_program_create_custom_uniform', null, ['number, string, number, number'], [WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_ENCODE_DEPTH], "maxRange", 0, _wrjs_pointerOnFloat(maxRange)]); //enum

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

  static shadowVolumeShader() {
    if (! WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME]) {
       WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME] = _wr_shader_program_new();

      _wr_shader_program_use_uniform( WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], 16);//enum

      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], 2);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], 3);//enum
      _wr_shader_program_use_uniform_buffer(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], 4);//enum

       WbWrenShaders.buildShader(WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME], "../../../resources/wren/shaders/shadow_volume.vert", "../../../resources/wren/shaders/shadow_volume.frag");
    }

    return WbWrenShaders.gShaders[WbWrenShaders.SHADER.SHADER_SHADOW_VOLUME];
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
        //console.log("render");
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
