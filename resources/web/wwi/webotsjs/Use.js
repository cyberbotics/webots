class Use {
  constructor(id, def, parent){
    this.id = id;
    this.def = def;

    this.parent = parent;
    this.wrenRenderable;
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

  modifyWrenMaterial(wrenMaterial, textured) {
    this.def.modifyWrenMaterial(wrenMaterial, textured);
  }

  modifyWrenMaterial(wrenMaterial, mainTextureIndex, backgroundTextureIndex) {
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
