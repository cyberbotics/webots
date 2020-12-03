import {WbBaseNode} from "./WbBaseNode.js"

class WbFog extends WbBaseNode {
  constructor(id, color, visibilityRange, fogType) {
    super(id);
    this.color = color;
    this.visibilityRange = visibilityRange;
    this.fogType = fogType;

    this.wrenFogType = undefined;
  }

  createWrenObjects(){
    super.createWrenObjects();

    this.applyChangesToWren();
  }

  applyChangesToWren() {
    let density = 0.0;
    if (this.visibilityRange > 0.0)
      density = 1.0 / this.visibilityRange;
    else
      this.wrenFogType = ENUM.WR_SCENE_FOG_TYPE_NONE;

    console.log(this);
    let colorPointer = _wrjs_color_array(this.color.x, this.color.y, this.color.z);
    _wr_scene_set_fog(_wr_scene_get_instance(), this.wrenFogType, ENUM.WR_SCENE_FOG_DEPTH_TYPE_POINT, colorPointer, density, 0.0, this.visibilityRange);
  }

  preFinalize() {
    super.preFinalize();
    this.updateFogType();
  }

  updateFogType() {
    if (this.fogType === "EXPONENTIAL")
      this.wrenFogType = ENUM.WR_SCENE_FOG_TYPE_EXPONENTIAL;
    else if (this.fogType == "EXPONENTIAL2")
      this.wrenFogType = ENUM.WR_SCENE_FOG_TYPE_EXPONENTIAL2;
    else
      this.wrenFogType = ENUM.WR_SCENE_FOG_TYPE_LINEAR;

    if (this.wrenFogType === ENUM.WR_SCENE_FOG_TYPE_LINEAR && this.fogType !== "LINEAR")
      console.warn("Unknown 'fogType': " + this.fogType + " Set to \"LINEAR\"");

    if (this.wrenObjectsCreatedCalled)
      this.applyChangesToWren();

  }
}

export {WbFog}
