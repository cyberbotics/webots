import {WbBaseNode} from "./WbBaseNode.js";
import {array3Pointer} from "./WbUtils.js";

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

  preFinalize() {
    super.preFinalize();

    /*updateAmbientIntensity();
    updateDiffuseColor();
    updateEmissiveColor();
    updateShininess();
    updateSpecularColor();
    updateTransparency();*/
  }

  postFinalize() {
    super.postFinalize();
  }
}

export {WbMaterial}
