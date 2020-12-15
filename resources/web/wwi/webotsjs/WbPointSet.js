import {WbGeometry} from "./WbGeometry.js"
import {arrayXPointerFloat} from "./WbUtils.js"

class WbPointSet extends WbGeometry {
  constructor(id, coord, color){
    super(id);
    this.coord = coord;
    this.color = color;
  }

  createWrenObjects() {
    super.createWrenObjects();
    //_wr_config_enable_point_size(true);
    this.updateCoord();
    this.buildWrenMesh();
  }

  updateCoord() {
    if (!this.sanitizeFields())
      return;

    //if (this.wrenObjectsCreatedCalled)
      //this.buildWrenMesh();

    if (this.boundingSphere)
      this.boundingSphere.setOwnerSizeChanged();
  }

  sanitizeFields() {
    if (typeof this.coord === 'undefined' || this.coord.length === 0) {
      console.warn("A non-empty 'Coordinate' node should be present in the 'coord' field.");
      return false;
    }

    if (typeof this.color !== 'undefined' && this.color.length != this.coord.length) {
      console.warn("If a 'Color' node is present in the 'color' field, it should have the same number of component as the 'Coordinate' node in the 'coord' field.");
      if (this.color.length === 0)
        return false;
      else
        console.warn("Only the first " + Math.min(this.color.length, this.coord.length) + " points will be drawn.");
    }

    return true;
  }

  buildWrenMesh() {
    super.deleteWrenRenderable();

    _wr_static_mesh_delete(this.wrenMesh);
    this.wrenMesh = undefined;

    if (typeof this.coord === 'undefined' || this.coord.length === 0)
      return;

    super.computeWrenRenderable();

    let coordsData = [];
    let colorData = undefined
    if(typeof this.color !== 'undefined');
      colorData = [];

    let coordsCount = this.computeCoordsAndColorData(coordsData, colorData);

    let coordsDataPointer = arrayXPointerFloat(coordsData);
    let colorDataPointer = arrayXPointerFloat(colorData);
    this.wrenMesh = _wr_static_mesh_point_set_new(coordsCount, coordsDataPointer, colorDataPointer);

    _free(coordsDataPointer);
    _free(colorDataPointer);

    _wr_renderable_set_cast_shadows(this.wrenRenderable, false);
    _wr_renderable_set_receive_shadows(this.wrenRenderable, false);
    _wr_renderable_set_drawing_mode(this.wrenRenderable, ENUM.WR_RENDERABLE_DRAWING_MODE_POINTS);
    _wr_renderable_set_point_size(this.wrenRenderable, 4.0);
    _wr_renderable_set_mesh(this.wrenRenderable, this.wrenMesh);
  }

  computeCoordsAndColorData(coordsData, colorData) {
    if (typeof this.coord === 'undefined')
      return 0;

    let count = 0;
    if (typeof colorData !== 'undefined') {
      let size = Math.min(this.coord.length, this.color.length);
      for(let i = 0; i < size; i++){
        coordsData[3 * count] = this.coord[i].x;
        coordsData[3 * count + 1] = this.coord[i].y;
        coordsData[3 * count + 2] = this.coord[i].z;
        colorData[3 * count] = this.color[i].x;
        colorData[3 * count + 1] = this.color[i].y;
        colorData[3 * count + 2] = this.color[i].z;
        count++;
      }
    } else {
      for(let i = 0; i < this.coord.length; i++){
        coordsData[3 * count] = this.coord[i].x;
        coordsData[3 * count + 1] = this.coord[i].y;
        coordsData[3 * count + 2] = this.coord[i].z;
        count++;
      }
    }
    return count;
  }

}

export {WbPointSet}
