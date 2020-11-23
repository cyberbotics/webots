class Use {
  constructor(id, def, parent){
    this.id = id;
    this.def = def;
    this.parent = parent;
  }

  createWrenObjects() {
  }

  modifyWrenMaterial() {
    this.def.modifyWrenMaterial();
  }

  setWrenMaterial() {
    let wrenObjTemp3 = this.def.parent;
    this.def.parent = this.parent;
    this.def.setWrenMaterial();
    this.parent = this.def.parent;
    this.def.parent = wrenObjTemp3;
  }
}

export{Use}
