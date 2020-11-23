class Use {
  constructor(id, def, parent){
    this.id = id;
    this.def = def;

    this.parent = parent;
    this.wrenRenderable;

    this.wrenTextureTransform;
  }

  createWrenObjects() {
    let temp = this.def.parent;
    this.def.parent = this.parent;

    let temp2 = this.def.wrenRenderable;
    this.def.wrenRenderable = undefined;

    this.def.createWrenObjects();

    this.def.parent = temp;

    this.wrenRenderable = this.def.wrenRenderable;
    this.def.wrenRenderable = temp2;
  }

  modifyWrenMaterial1(wrenMaterial) {
    let temp2 = this.def.wrenTextureTransform;
    this.def.wrenTextureTransform = undefined;
    
    this.def.modifyWrenMaterial(wrenMaterial);

    this.wrenRenderable = this.def.wrenTextureTransform;
    this.def.wrenTextureTransform = temp2;
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
  }

  postFinalize(){
  }
}

export{Use}
