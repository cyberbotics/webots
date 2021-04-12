import {WbGeometry} from './wbGeometry.js';

class WbPlane extends WbGeometry {
  constructor(id, size) {
    super(id);
    this.size = size;
  }

  delete() {
    _wr_static_mesh_delete(this.wrenMesh);

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.computeWrenRenderable();

    const createOutlineMesh = super.isInBoundingObject();
    const wrenMesh = _wr_static_mesh_unit_rectangle_new(createOutlineMesh);

    _wr_renderable_set_mesh(this.wrenRenderable, wrenMesh);

    this.updateSize();
  }

  updateSize() {
    if (super.isInBoundingObject())
      updateLineScale();
    else
      this.updateScale();
  }

  updateLineScale() {
    if (!this.isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;

    // allow the bounding sphere to scale down
    const scaleY = 0.1 * Math.min(this.size.x, this.size.y);
    wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.size.x * (1.0 + offset), scaleY, this.size.y * (1.0 + offset)));
  }

  updateScale() {
    // allow the bounding sphere to scale down
    const scaleY = 0.1 * Math.min(this.size.x, this.size.y);

    const scale = _wrjs_array3(this.size.x, scaleY, this.size.y);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  isSuitableForInsertionInBoundingObject() {
    return super.isSuitableForInsertionInBoundingObject() && !(this.size.x <= 0.0 || this.size.y <= 0.0);
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbPlane(customID, this.size);
  }
}

export {WbPlane};
