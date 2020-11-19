import {WbBaseNode} from "./WbBaseNode.js"
import {WbAppearance} from "./WbAppearance.js"
import {WbPBRAppearance} from "./WbPBRAppearance.js"
import {WbWrenShaders} from "./WbWrenShaders.js"


class WbShape extends WbBaseNode {
  constructor(id, castShadow, geometry, appearance) {
    super(id);
    this.castShadow = castShadow;
    this.appearance = appearance;
    this.geometry = geometry;

    this.wrenMaterial = undefined;
  }


  createWrenObjects() {
    super.createWrenObjects();
    if (this.appearance)
      this.appearance.createWrenObjects();

    if (this.geometry){
      this.geometry.createWrenObjects();
      //not sure of the place
      this.applyMaterialToGeometry()
    }
  }

  applyMaterialToGeometry() {
    if (!this.wrenMaterial)
      this.createWrenMaterial(ENUM.WR_MATERIAL_PHONG);
    if (this.geometry) {
      if (this.appearance instanceof WbAppearance) {
        if (this.appearance.wrenObjectsCreatedCalled)
          this.wrenMaterial = this.appearance.modifyWrenMaterial(this.wrenMaterial);
        else
          this.wrenMaterial = WbAppearance.fillWrenDefaultMaterial(this.wrenMaterial);
      } else if (this.appearance instanceof WbPBRAppearance && !(this.geometry instanceof WbPointSet)) {
        this.createWrenMaterial(2);
        if (this.appearance.wrenObjectsCreatedCalled){
          this.wrenMaterial = this.appearance.modifyWrenMaterial(this.wrenMaterial);
        }
      } else {
        this.wrenMaterial = WbAppearance.fillWrenDefaultMaterial(this.wrenMaterial);
      }
    }

    this.geometry.setWrenMaterial(this.wrenMaterial, this.castShadow);
  }

  createWrenMaterial(type) {
    let defaultColor = _wrjs_color_array(1.0, 1.0, 1.0);
    if (this.wrenMaterial)
      _wr_material_delete(this.wrenMaterial);

    if (type === ENUM.WR_MATERIAL_PHONG) {
      this.wrenMaterial = _wr_phong_material_new();
      _wr_phong_material_set_color(this.wrenMaterial, defaultColor);
      _wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.defaultShader());
    } else {
      this.wrenMaterial = _wr_pbr_material_new();
      _wr_pbr_material_set_base_color(this.wrenMaterial, defaultColor);
      _wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.pbrShader());
    }
  }
}

export {WbShape}
