class Use {
  constructor(id, def, parent){
    this.id = id;
    this.def = def;
    this.parent = parent;
  }

  createWrenObjects() {
    let wrenObjTemp3 = this.def.parent;
    this.def.parent = this.parent;
    this.def.createWrenObjects();
    this.parent = this.def.parent;
    this.def.parent = wrenObjTemp3;
  }

  modifyWrenMaterial() {
    this.def.modifyWrenMaterial();
  }

  setWrenMaterial() {
  }

  preFinalize(){
    this.def.preFinalize();
  }

  postFinalize(){
    this.def.postFinalize();
  }
}

export{Use}
