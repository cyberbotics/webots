import {WbGeometry} from './wbGeometry.js';

class WbCylinder extends WbGeometry {
  constructor(id, radius, height, subdivision, bottom, side, top) {
    super(id);
    this.radius = radius;
    this.height = height;
    this.subdivision = subdivision;
    this.bottom = bottom;
    this.side = side;
    this.top = top;
  }

  delete() {
    _wr_static_mesh_delete(this.wrenMesh);

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();

    if (this.subdivision < 3)
      this.subdivision = 3;

    if (!this.bottom && !this.side && !this.top)
      return;

    this.computeWrenRenderable();

    this.wrenMesh = _wr_static_mesh_unit_cylinder_new(this.subdivision, this.side, this.top, this.bottom, false);

    _wr_renderable_set_mesh(this.wrenRenderable, this.wrenMesh);

    const scale = _wrjs_array3(this.radius, this.height, this.radius);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbCylinder(customID, this.radius, this.height, this.subdivision, this.bottom, this.side, this.top);
  }
}

export {WbCylinder};
