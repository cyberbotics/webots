import {WbBaseNode} from "./WbBaseNode.js";
import {WbMaterial} from "./WbMaterial.js";

class Use extends WbBaseNode {
  constructor(id, def, parent){
    super(id);
    this.def = def;

    this.parent = parent;
    this.wrenObjectsCreatedCalled = false

    this.wrenRenderable;
    this.wrenTextureTransform;
    this.material = new WbMaterial(35, this.def.material.ambientIntensity, this.def.material.diffuseColor, this.def.material.specularColor, this.def.material.emissiveColor, this.def.material.shininess, this.def.material.transparency);
    this.material.parent = this.id;
  }

  createWrenObjects() {
    console.log(this.def);

    console.log(this);

    super.createWrenObjects();
    let tempList = [];

    let temp = this.def.parent;
    this.def.parent = this.parent;

    let temp2 = this.def.wrenRenderable;
    this.def.wrenRenderable = undefined;

    let temp3 = this.def.material;
    this.def.material = this.material;

    this.def.createWrenObjects();

    this.def.parent = temp;

    this.wrenRenderable = this.def.wrenRenderable;
    this.def.wrenRenderable = temp2;

    this.material = this.def.material;
    this.def.material = temp3;

    this.wrenObjectsCreatedCalled = true;
  }

  modifyWrenMaterial1(wrenMaterial) {
    let temp3 = this.def.material;
    this.def.material = this.material;

    let temp = this.def.wrenTextureTransform;
    this.def.wrenTextureTransform = undefined;

    this.def.modifyWrenMaterial(wrenMaterial);

    this.wrenRenderable = this.def.wrenTextureTransform;
    this.def.wrenTextureTransform = temp;

    this.material = this.def.material;
    this.def.material = temp3;

  }

  modifyWrenMaterial2(wrenMaterial, textured) {
    this.def.modifyWrenMaterial(wrenMaterial, textured);
  }

  modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex) {
    if (typeof backgroundTextureIndex === 'undefined') {
      if (typeof mainTextureIndex === 'undefined')
        this.modifyWrenMaterial1(wrenMaterial);
      else
        this.modifyWrenMaterial2(wrenMaterial, mainTextureIndex);
    }else
      this.def.modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex);
  }

  setWrenMaterial(wrenMaterial, castShadow) {
    let temp2 = this.def.wrenRenderable;
    this.def.wrenRenderable = this.wrenRenderable;

    this.def.setWrenMaterial(wrenMaterial, castShadow);

    this.wrenRenderable = this.def.wrenRenderable;
    this.def.wrenRenderable = temp2;
  }

  preFinalize(){
    this.isPreFinalizeCalled = true;
  }

  postFinalize(){
    this.isPreFinalizeCalled = false;
  }
}

export{Use}
