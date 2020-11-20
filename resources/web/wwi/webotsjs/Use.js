class Use {
  constructor(id, def, parent){
    this.id = id;
    this.def = def;
    this.parent = parent;

    this.wrenNode;

  }

  createWrenObjects() {
    console.log(this.def);
    let wrenObjTemp = this.def.wrenNode;
    this.def.wrenNode = undefined;
    this.def.createWrenObjects();
    this.wrenNode = this.def.wrenNode;
    this.def.wrenNode = wrenObjTemp;
  }

  modifyWrenMaterial() {
    this.def.modifyWrenMaterial();
  }
}

export{Use}
