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


import {WbAbstractAppearance} from "./WbAbstractAppearance.js"
import {WbWrenShaders} from "./WbWrenShaders.js"

class WbAppearance extends WbAbstractAppearance {
  constructor(id, material, texture, transform){
    super(id, transform);
    this.material = material;
    this.texture = texture;
  }

  delete(){
    if (typeof this.material !== 'undefined')
      this.material.delete();

    if (typeof this.texture !== 'undefined')
      this.texture.delete();

    super.delete();
  }

  createWrenObjects(){
    super.createWrenObjects();
    if (typeof this.material !== 'undefined') {
      this.material.createWrenObjects();
    }

    if (typeof this.texture !== 'undefined') {
      this.texture.createWrenObjects();
    }
  }

  modifyWrenMaterial(wrenMaterial) {
    if (typeof this.material !== 'undefined') {
      _wr_material_set_default_program(wrenMaterial, WbWrenShaders.phongShader());
      _wr_material_set_stencil_ambient_emissive_program(wrenMaterial, WbWrenShaders.phongStencilAmbientEmissiveShader());
      _wr_material_set_stencil_diffuse_specular_program(wrenMaterial, WbWrenShaders.phongStencilDiffuseSpecularShader());

      this.material.modifyWrenMaterial(wrenMaterial, this.texture && this.texture.wrenTexture);
    } else {
      wrenMaterial = WbAppearance.fillWrenDefaultMaterial(wrenMaterial);
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

  static fillWrenDefaultMaterial(wrenMaterial) {
    //TODO add suport if not a phong material
    if (!wrenMaterial) {
      _wr_material_delete(wrenMaterial);
      wrenMaterial = _wr_phong_material_new();
    }

    _wr_material_set_default_program(wrenMaterial, WbWrenShaders.defaultShader());
    return wrenMaterial;
  }

  preFinalize() {
    super.preFinalize();

    if (typeof this.material !== 'undefined')
      this.material.preFinalize();

    if (typeof this.texture !== 'undefined')
      this.texture.preFinalize();

  }

  postFinalize() {
    super.postFinalize();

    if (typeof this.material !== 'undefined')
      this.material.postFinalize();
    if (typeof this.texture !== 'undefined')
      this.texture.postFinalize();
  }
}

export {WbAppearance}
