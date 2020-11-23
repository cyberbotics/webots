import {WbGeometry} from "./WbGeometry.js"

class WbCone extends WbGeometry {
  constructor(id, bottomRadius, height, subdivision, side, bottom) {
    super(id);
    this.bottomRadius = bottomRadius;
    this.height = height;
    this.subdivision = subdivision;
    this.side = side;
    this.bottom = bottom;
  }

  createWrenObjects() {
    super.createWrenObjects();

    if (!this.bottom && !this.side)
        return;

    this.computeWrenRenderable();

    let wrenMesh = _wr_static_mesh_unit_cone_new(this.subdivision, this.side , this.bottom);

    _wr_renderable_set_mesh(this.wrenRenderable, wrenMesh);

    let scale = _wrjs_color_array(this.bottomRadius, this.height, this.bottomRadius);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  postFinalize() {
    super.postFinalize();
  }

}

export {WbCone}
