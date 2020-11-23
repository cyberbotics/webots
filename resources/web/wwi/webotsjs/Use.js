class Use {
  constructor(id, def, parent){
    this.id = id;
    this.def = def;
    this.parent = parent;
  }

  createWrenObjects() {
  }

  modifyWrenMaterial(wrenMaterial, textured) {
    this.def.modifyWrenMaterial(wrenMaterial, textured);
  }

  setWrenMaterial() {
  }

  preFinalize(){
  }

  postFinalize(){
  }
}

export{Use}
