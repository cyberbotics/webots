import WbGeometry from './WbGeometry.js';

export default class WbCylinder extends WbGeometry {
  constructor(id, radius, height, subdivision, bottom, side, top) {
    super(id);
    this.radius = radius;
    this.height = height;
    this.subdivision = subdivision;
    this.bottom = bottom;
    this.side = side;
    this.top = top;
  }

  setParameter(parameterName, parameterValue) {
    switch (parameterName) {
      case 'radius':
        this.radius = parameterValue;
        this.updateRadius();
        break;
      case 'height':
        this.height = parameterValue;
        this.updateHeight();
        break;
      case 'subdivision':
        this.subdivision = parameterValue;
        this.updateSubdivision();
        break;
      case 'top':
        this.top = parameterValue;
        this._buildWrenMesh();
        break;
      case 'bottom':
        this.bottom = parameterValue;
        this._buildWrenMesh();
        break;
      case 'side':
        this.side = parameterValue;
        this._buildWrenMesh();
        break;
      default:
        throw new Error('Unknown parameter ' + parameterName + ' for node WbCylinder.');
    }
  }

  clone(customID) {
    this.useList.push(customID);
    return new WbCylinder(customID, this.radius, this.height, this.subdivision, this.bottom, this.side, this.top);
  }

  createWrenObjects() {
    super.createWrenObjects();

    if (this.subdivision < 3)
      this.subdivision = 3;

    if (!this.bottom && !this.side && !this.top)
      return;

    this._computeWrenRenderable();

    this._wrenMesh = _wr_static_mesh_unit_cylinder_new(this.subdivision, this.side, this.top, this.bottom, false);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);

    const scale = _wrjs_array3(this.radius, this.height, this.radius);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;
    const scale =  _wrjs_array3(this.radius * (1 + offset), this.height * (1 + offset), this.radius * (1 + offset))

    wr_transform_set_scale(wrenNode(), scale);
  }

  updateRadius() {
    if (super._isAValidBoundingObject())
      this.updateLineScale();
    else
      _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.radius, this.height, this.radius));
  }

  updateHeight() {
    if (super._isAValidBoundingObject())
      this.updateLineScale();
    else
      _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.radius, this.height, this.radius));
  }

  updateFaces() {
    this._buildWrenMesh();
  }

  updateSubdivision() {
    this._buildWrenMesh();
  }

  _buildWrenMesh() {
    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    this._wrenMesh = _wr_static_mesh_unit_cylinder_new(this.subdivision, this.side, this.top, this.bottom, false);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);

    const scale = _wrjs_array3(this.radius, this.height, this.radius);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  delete() {
    _wr_static_mesh_delete(this._wrenMesh);

    super.delete();
  }
}
