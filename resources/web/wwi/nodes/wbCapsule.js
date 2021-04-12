import {WbGeometry} from './wbGeometry.js';

class WbCapsule extends WbGeometry {
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

    this.buildWrenMesh();
  }

  buildWrenMesh() {
    super.deleteWrenRenderable();

    if (typeof this.wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this.wrenMesh);
      this.wrenMesh = undefined;
    }

    if (!this.bottom && !this.side && !this.top)
      return;

    super.computeWrenRenderable();

    // Restore pickable state
    super.setPickable(this.isPickable);

    this.wrenMesh = _wr_static_mesh_capsule_new(this.subdivision, this.radius, this.height, this.side, this.top, this.bottom, false);

    _wr_renderable_set_mesh(this.wrenRenderable, this.wrenMesh);
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbCapsule(customID, this.radius, this.height, this.subdivision, this.bottom, this.side, this.top);
  }
}

export {WbCapsule};
