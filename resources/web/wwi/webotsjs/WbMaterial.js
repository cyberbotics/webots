// Copyright 1996-2021 Cyberbotics Ltd.
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
import {World} from "./World.js"

import {array3Pointer} from "./WbUtils.js";
import {WbVector3} from "./utils/WbVector3.js";

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

  delete(){
    let parent = World.instance.nodes.get(this.parent);

    if (typeof parent !== 'undefined') {
      let shape = World.instance.nodes.get(parent.parent);
      if (typeof shape !== 'undefined') {
        parent.material = undefined;
        shape.updateAppearance();
      }
    }
    super.delete
  }

  modifyWrenMaterial(wrenMaterial, textured) {
    let ambient, diffuse, specular, shininess;

    ambient = new WbVector3(this.ambientIntensity, this.ambientIntensity, this.ambientIntensity);

    if (textured) {
      diffuse = new WbVector3(1.0, 1.0, 1.0);
      specular = new WbVector3(1.0, 1.0, 1.0);
      shininess = 0.0;
    } else {
      ambient = new WbVector3(this.ambientIntensity * this.diffuseColor.x,this.ambientIntensity * this.diffuseColor.y,
                       this.ambientIntensity * this.diffuseColor.z);
      diffuse = new WbVector3(this.diffuseColor.x, this.diffuseColor.y, this.diffuseColor.z);
      specular = new WbVector3(this.specularColor.x, this.specularColor.y, this.specularColor.z);
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
