// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import {WbBaseNode} from "./WbBaseNode.js";
import {WbViewpoint} from "./WbViewpoint.js";
import {WbWrenShaders} from "./WbWrenShaders.js";
import {arrayXPointer} from "./WbUtils.js";

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

  delete(){
    super.delete();
    this.destroySkyBox();

    this.skyColor = new glm.vec3(0, 0, 0);
    this.applyColourToWren();

    _wr_scene_set_hdr_clear_quad(_wr_scene_get_instance(), null);
    // Delete skybox
    // Shader program is not deleted, a singleton instance is kept in WbWrenShaders
    _wr_node_delete(this.skyboxRenderable);

    if (typeof this.skyboxMaterial !== 'undefined')
      _wr_material_delete(this.skyboxMaterial);

    _wr_node_delete(this.skyboxTransform);
    _wr_static_mesh_delete(this.skyboxMesh);

    _wr_node_delete(this.hdrClearRenderable);
    this.hdrClearRenderable = null;
    _wr_scene_set_hdr_clear_quad(_wr_scene_get_instance(), this.hdrClearRenderable);

    if (typeof this.hdrClearMaterial !== 'undefined')
      _wr_material_delete(this.hdrClearMaterial);

    _wr_node_delete(this.hdrClearTransform);
    _wr_static_mesh_delete(this.hdrClearMesh);
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
    _wr_renderable_set_drawing_mode(this.skyboxRenderable, ENUM.WR_RENDERABLE_DRAWING_MODE_TRIANGLES);
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

    let hdrImageData = [];
    // 1. Load the background.
    if(this.cubeArray.length === 6) {
      this.cubeMapTexture = _wr_texture_cubemap_new();
      _wr_texture_set_internal_format(this.cubeMapTexture, ENUM.WR_TEXTURE_INTERNAL_FORMAT_RGBA8);

      let bitsPointers = []
      for (let i = 0; i < 6; ++i) {
        // TODO Check if some rotations are needed for ENU
        bitsPointers[i] = arrayXPointer(this.cubeArray[i].bits);
        _wr_texture_cubemap_set_data(this.cubeMapTexture, bitsPointers[i], i);
      }

      _wr_texture_set_size(this.cubeMapTexture, this.cubeArray[0].width, this.cubeArray[0].height);
      _wr_texture_setup(this.cubeMapTexture);
      _wr_material_set_texture_cubemap(this.skyboxMaterial, this.cubeMapTexture, 0);
      _wr_material_set_texture_cubemap_wrap_r(this.skyboxMaterial, ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      _wr_material_set_texture_cubemap_wrap_s(this.skyboxMaterial, ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      _wr_material_set_texture_cubemap_wrap_t(this.skyboxMaterial, ENUM.WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE, 0);
      _wr_scene_set_skybox(_wr_scene_get_instance(), this.skyboxRenderable);

      for(let i = 0; i < 6; ++i) {
        _free(bitsPointers[i]);
      }
    }

    // 2. Load the irradiance map.
    let cm = _wr_texture_cubemap_new();

    if(this.irradianceCubeArray.length === 6) {
      let w = 0;
      let data = new Uint32Array([w]);
      let nDataBytes = data.length * data.BYTES_PER_ELEMENT;
      let dataPtr = Module._malloc(nDataBytes);
      let dataHeap = new Uint8Array(Module.HEAPU8.buffer, dataPtr, nDataBytes);
      dataHeap.set(new Uint8Array(data.buffer));

      for (let i = 0; i < 6; ++i) {
        _wr_texture_set_internal_format(cm, ENUM.WR_TEXTURE_INTERNAL_FORMAT_RGB32F);
        let data = Module.ccall('wrjs_load_hdr_file', null, ['number','string'], [dataHeap.byteOffset, this.irradianceCubeArray[i]]);
        w = Module.getValue(dataHeap.byteOffset,'i32');
        // TODO Check if some rotations are needed for ENU
        _wr_texture_cubemap_set_data(cm, data, i);
        hdrImageData[i] = data;
      }

      //hdr must be square? if not change also in wrjs_load_hdr_file
      _wr_texture_set_size(cm, w, w);
      _wr_texture_set_texture_unit(cm, 13);
      _wr_texture_setup(cm);

      this.irradianceCubeTexture = _wr_texture_cubemap_bake_specular_irradiance(cm, WbWrenShaders.iblSpecularIrradianceBakingShader(), w);
      _wr_texture_cubemap_disable_automatic_mip_map_generation(this.irradianceCubeTexture);
    } else {
      if (this.irradianceCubeTexture) {
        _wr_texture_delete(this.irradianceCubeTexture);
        this.irradianceCubeTexture = null;
      }
      // Fallback: a cubemap is found but no irradiance map: bake a small irradiance map to have right colors.
      // Reflections won't be good in such case.
      if (this.cubeMapTexture) {
        this.irradianceCubeTexture = _wr_texture_cubemap_bake_specular_irradiance(this.cubeMapTexture, WbWrenShaders.iblSpecularIrradianceBakingShader(), 64);
        _wr_texture_cubemap_disable_automatic_mip_map_generation(this.irradianceCubeTexture);
      }
    }

    _wr_texture_delete(cm);


    for(let i = 0; i < hdrImageData.length; ++i)
      _wrjs_free_hdr_file(hdrImageData[i]);
  }

  preFinalize() {
    super.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.applySkyBoxToWren();
  }
}

export{WbBackground}
