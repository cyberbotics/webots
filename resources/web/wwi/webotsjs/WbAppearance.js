import {WbAbstractAppearance} from "./WbAbstractAppearance.js"
import {WbWrenShaders} from "./WbWrenShaders.js"

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

export {WbAppearance}
