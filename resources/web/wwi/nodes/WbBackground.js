import {arrayXPointer, arrayXPointerFloat} from './utils/utils.js';
import WbBaseNode from './WbBaseNode.js';
import WbVector3 from './utils/WbVector3.js';
import WbWorld from './WbWorld.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbBackground extends WbBaseNode {
  #cubeArray;
  #cubeMapTexture;
  #hdrClearMaterial;
  #hdrClearMesh;
  #hdrClearRenderable;
  #hdrClearTransform;
  #irradianceCubeArray;
  #skyboxMaterial;
  #skyboxMesh;
  #skyboxRenderable;
  #skyboxTransform;
  constructor(id, skyColor, luminosity) {
    super(id);
    this.skyColor = skyColor;
    this.luminosity = luminosity;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_BACKGROUND;
  }

  setCubeArray(cubeArray) {
    this.#cubeArray = cubeArray;
  }
  setIrradianceCubeArray(irradianceCubeArray) {
    this.#irradianceCubeArray = irradianceCubeArray;
  }

  createWrenObjects() {
    super.createWrenObjects();

    let skyboxShaderProgram = WbWrenShaders.skyboxShader();
    this.#skyboxMaterial = _wr_phong_material_new();
    this.#skyboxRenderable = _wr_renderable_new();
    this.#skyboxMesh = _wr_static_mesh_unit_box_new(false);

    _wr_material_set_default_program(this.#skyboxMaterial, skyboxShaderProgram);
    _wr_renderable_set_cast_shadows(this.#skyboxRenderable, false);
    _wr_renderable_set_receive_shadows(this.#skyboxRenderable, false);
    _wr_renderable_set_mesh(this.#skyboxRenderable, this.#skyboxMesh);
    _wr_renderable_set_material(this.#skyboxRenderable, this.#skyboxMaterial, null);
    _wr_renderable_set_drawing_mode(this.#skyboxRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_TRIANGLES);
    _wr_renderable_set_face_culling(this.#skyboxRenderable, false);

    this.#skyboxTransform = _wr_transform_new();
    _wr_transform_attach_child(this.#skyboxTransform, this.#skyboxRenderable);

    let hdrClearShaderProgram = WbWrenShaders.hdrClearShader();
    this.#hdrClearMaterial = _wr_phong_material_new();
    this.#hdrClearRenderable = _wr_renderable_new();
    this.#hdrClearMesh = _wr_static_mesh_quad_new();

    _wr_material_set_default_program(this.#hdrClearMaterial, hdrClearShaderProgram);
    _wr_renderable_set_cast_shadows(this.#hdrClearRenderable, false);
    _wr_renderable_set_receive_shadows(this.#hdrClearRenderable, false);
    _wr_renderable_set_mesh(this.#hdrClearRenderable, this.#hdrClearMesh);
    _wr_renderable_set_material(this.#hdrClearRenderable, this.#hdrClearMaterial, null);
    _wr_renderable_set_drawing_mode(this.#hdrClearRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_TRIANGLES);

    this.#hdrClearTransform = _wr_transform_new();
    _wr_transform_attach_child(this.#hdrClearTransform, this.#hdrClearRenderable);

    this.#applyColourToWren();
  }

  delete() {
    super.delete();

    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.root.children.indexOf(this);
      WbWorld.instance.root.children.splice(index, 1);
    }

    this.#destroySkyBox();

    this.skyColor = new WbVector3(0, 0, 0);
    this.#applyColourToWren();

    _wr_scene_set_hdr_clear_quad(_wr_scene_get_instance(), null);
    // Delete skybox
    // Shader program is not deleted, a singleton instance is kept in WbWrenShaders
    _wr_node_delete(this.#skyboxRenderable);

    if (typeof this.#skyboxMaterial !== 'undefined')
      _wr_material_delete(this.#skyboxMaterial);

    _wr_node_delete(this.#skyboxTransform);
    _wr_static_mesh_delete(this.#skyboxMesh);

    _wr_node_delete(this.#hdrClearRenderable);
    this.#hdrClearRenderable = undefined;
    _wr_scene_set_hdr_clear_quad(_wr_scene_get_instance(), this.#hdrClearRenderable);

    if (typeof this.#hdrClearMaterial !== 'undefined')
      _wr_material_delete(this.#hdrClearMaterial);

    _wr_node_delete(this.#hdrClearTransform);
    _wr_static_mesh_delete(this.#hdrClearMesh);

    WbBackground.instance = undefined;

    this.#updatePBRs();
  }

  postFinalize() {
    super.postFinalize();

    this.#applySkyBoxToWren();
    this.#updatePBRs();
  }

  // Private functions
  #applyColourToWren() {
    const colorPointer = _wrjs_array3(this.skyColor.x, this.skyColor.y, this.skyColor.z);

    _wr_viewport_set_clear_color_rgb(_wr_scene_get_viewport(_wr_scene_get_instance()), colorPointer);
    if (typeof this.wrenObjectsCreatedCalled !== 'undefined') {
      // use wren's set_diffuse to transform to linear color space
      _wr_phong_material_set_diffuse(this.#hdrClearMaterial, colorPointer);

      // de-gamma correct
      const hdrColor = [Math.pow(this.skyColor.x, 2.2), Math.pow(this.skyColor.y, 2.2), Math.pow(this.skyColor.z, 2.2)];
      // reverse tone map
      const exposure = WbWorld.instance.viewpoint.exposure;
      for (let i = 0; i < 3; ++i)
        hdrColor[i] = -Math.log(1.000000001 - hdrColor[i]) / exposure;

      const hdrColorPointer = _wrjs_array3(hdrColor[0], hdrColor[1], hdrColor[2]);
      _wr_phong_material_set_linear_diffuse(this.#hdrClearMaterial, hdrColorPointer);
      _wr_scene_set_hdr_clear_quad(_wr_scene_get_instance(), this.#hdrClearRenderable);
    }
  }

  #applySkyBoxToWren() {
    this.#destroySkyBox();

    // 1. Load the background.
    if (typeof this.#cubeArray !== 'undefined' && this.#cubeArray.length === 6) {
      this.#cubeMapTexture = _wr_texture_cubemap_new();
      _wr_texture_set_internal_format(this.#cubeMapTexture, Enum.WR_TEXTURE_INTERNAL_FORMAT_RGBA8);

      const bitsPointers = [];
      for (let i = 0; i < 6; ++i) {
        bitsPointers[i] = arrayXPointer(this.#cubeArray[i].bits);
        _wr_texture_cubemap_set_data(this.#cubeMapTexture, bitsPointers[i], i);
      }

      _wr_texture_set_size(this.#cubeMapTexture, this.#cubeArray[0].width, this.#cubeArray[0].height);
      _wr_texture_setup(this.#cubeMapTexture);
      _wr_material_set_texture_cubemap(this.#skyboxMaterial, this.#cubeMapTexture, 0);
      _wr_material_set_texture_cubemap_wrap_r(this.#skyboxMaterial, Enum.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      _wr_material_set_texture_cubemap_wrap_s(this.#skyboxMaterial, Enum.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      _wr_material_set_texture_cubemap_wrap_t(this.#skyboxMaterial, Enum.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      _wr_scene_set_skybox(_wr_scene_get_instance(), this.#skyboxRenderable);

      for (let i = 0; i < 6; ++i)
        _free(bitsPointers[i]);
    }

    // 2. Load the irradiance map.
    const cubeMap = _wr_texture_cubemap_new();
    const hdrImageData = [];
    if (typeof this.#cubeArray !== 'undefined' && this.#irradianceCubeArray.length === 6) {
      _wr_texture_set_internal_format(cubeMap, Enum.WR_TEXTURE_INTERNAL_FORMAT_RGB32F);

      for (let i = 0; i < 6; ++i) {
        hdrImageData[i] = arrayXPointerFloat(this.#irradianceCubeArray[i].bits);
        _wr_texture_cubemap_set_data(cubeMap, hdrImageData[i], i);
      }

      _wr_texture_set_size(cubeMap, this.#irradianceCubeArray[0].width, this.#irradianceCubeArray[0].height);
      _wr_texture_set_texture_unit(cubeMap, 13);
      _wr_texture_setup(cubeMap);

      this.irradianceCubeTexture = _wr_texture_cubemap_bake_specular_irradiance(cubeMap,
        WbWrenShaders.iblSpecularIrradianceBakingShader(), this.#irradianceCubeArray[0].width);
      _wr_texture_cubemap_disable_automatic_mip_map_generation(this.irradianceCubeTexture);
    } else {
      if (typeof this.irradianceCubeTexture !== 'undefined') {
        _wr_texture_delete(this.irradianceCubeTexture);
        this.irradianceCubeTexture = undefined;
      }
      // Fallback: a cubemap is found but no irradiance map: bake a small irradiance map to have right colors.
      // Reflections won't be good in such case.
      if (typeof this.#cubeMapTexture !== 'undefined') {
        this.irradianceCubeTexture = _wr_texture_cubemap_bake_specular_irradiance(this.#cubeMapTexture,
          WbWrenShaders.iblSpecularIrradianceBakingShader(), 64);
        _wr_texture_cubemap_disable_automatic_mip_map_generation(this.irradianceCubeTexture);
      }
    }

    _wr_texture_delete(cubeMap);

    for (let i = 0; i < hdrImageData.length; ++i)
      _free(hdrImageData[i]);
  }

  #destroySkyBox() {
    _wr_scene_set_skybox(_wr_scene_get_instance(), null);

    if (typeof this.#skyboxMaterial !== 'undefined')
      _wr_material_set_texture_cubemap(this.#skyboxMaterial, null, 0);

    if (typeof this.#cubeMapTexture !== 'undefined') {
      _wr_texture_delete(this.#cubeMapTexture);
      this.#cubeMapTexture = undefined;
    }

    if (typeof this.irradianceCubeTexture !== 'undefined') {
      _wr_texture_delete(this.irradianceCubeTexture);
      this.irradianceCubeTexture = undefined;
    }
  }

  #updatePBRs() {
    WbWorld.instance.nodes.forEach((value, key, map) => {
      if (value.nodeType === WbNodeType.WB_NODE_PBR_APPEARANCE && typeof value.parent !== 'undefined') {
        const parent = WbWorld.instance.nodes.get(value.parent);
        parent?.applyMaterialToGeometry();
      }
    });
  }
}
