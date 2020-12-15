import {WbGeometry} from "./WbGeometry.js"
import {arrayXPointerFloat} from "./WbUtils.js"

class WbElevationGrid extends WbGeometry {
  constructor(id, height, xDimension, xSpacing, zDimension, zSpacing, thickness) {
    super(id);
    this.height = height;
    this.xDimension = xDimension;
    this.xSpacing = xSpacing;
    this.zDimension = zDimension;
    this.zSpacing = zSpacing;
    this.thickness = thickness;
  }

  createWrenObjects() {
    super.createWrenObjects();
    this.buildWrenMesh();
  }

  buildWrenMesh() {
    super.deleteWrenRenderable();

    _wr_static_mesh_delete(this.wrenMesh);
    this.wrenMesh = undefined;

    if (this.xDimension < 2 || this.zDimension < 2)
      return;

    if (this.xSpacing == 0.0 || this.zSpacing == 0.0)
      return;

    super.computeWrenRenderable();


    // Restore pickable state
    //setPickable(isPickable());

    // convert height values to float, pad with zeroes if necessary
    let numValues = this.xDimension * this.zDimension;
    let heightData = [];

    let availableValues = Math.min(numValues, this.height.length);
    for (let i = 0; i < availableValues; ++i)
      heightData[i] = this.height[i];

    const createOutlineMesh = this.isInBoundingObject;

    let heightDataPointer = arrayXPointerFloat(heightData);
    this.wrenMesh = _wr_static_mesh_unit_elevation_grid_new(this.xDimension, this.zDimension, heightDataPointer, this.thickness, createOutlineMesh);

    _free(heightDataPointer);

    // This must be done after WbGeometry::computeWrenRenderable() otherwise
    // the outline scaling is applied to the wrong WREN transform
    if (createOutlineMesh)
      this.updateLineScale();
    else
      this.updateScale();

    _wr_renderable_set_mesh(this.wrenRenderable, this.wrenMesh);
  }

  updateScale() {
    let scalePointer = _wrjs_color_array(this.xSpacing, 1.0, this.zSpacing);
    _wr_transform_set_scale(this.wrenNode, scalePointer);
  }
}

export {WbElevationGrid}
