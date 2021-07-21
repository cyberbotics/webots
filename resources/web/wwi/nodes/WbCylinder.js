import WbGeometry from './WbGeometry.js';
import WbWorld from './WbWorld.js';

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
        this.updateMesh();
        break;
      case 'top':
        this.top = parameterValue;
        this.updateMesh();
        break;
      case 'bottom':
        this.bottom = parameterValue;
        this.updateMesh();
        break;
      case 'side':
        this.side = parameterValue;
        this.updateMesh();
        break;
      default:
        throw new Error('Unknown parameter ' + parameterName + ' for node WbCylinder.');
    }
  };

  clone(customID) {
    this.useList.push(customID);
    return new WbCylinder(customID, this.radius, this.height, this.subdivision, this.bottom, this.side, this.top);
  };

  createWrenObjects() {
    super.createWrenObjects();
    this._sanitizeFields();
    this._buildWrenMesh();
  };

  updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;
    const scale =  _wrjs_array3(this.radius * (1 + offset), this.height * (1 + offset), this.radius * (1 + offset))

    wr_transform_set_scale(wrenNode(), scale);
  };

  updateRadius() {
    this._sanitizeFields();

    if (super._isAValidBoundingObject())
      this.updateLineScale();
    else
      _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.radius, this.height, this.radius));
  };

  updateHeight() {
    this._sanitizeFields();

    if (super._isAValidBoundingObject())
      this.updateLineScale();
    else
      _wr_transform_set_scale(this.wrenNode, _wrjs_array3(this.radius, this.height, this.radius));
  };

  updateMesh() {
    this._sanitizeFields();

    this._buildWrenMesh();
    const parent = WbWorld.instance.nodes.get(this.parent); // needed otherwise it is no longer visible
    parent.updateAppearance();
  };

  _sanitizeFields() {
    if (this.subdivision < 3)
      this.subdivision = 3;

    if (this.radius < 0)
      this.radius = 1;

    if (this.height < 0)
      this.height = 2;
  };

  _buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    if (!this.bottom && !this.side && !this.top)
      return;

    super._computeWrenRenderable();

    this._wrenMesh = _wr_static_mesh_unit_cylinder_new(this.subdivision, this.side, this.top, this.bottom, false);

    const scale = _wrjs_array3(this.radius, this.height, this.radius);
    _wr_transform_set_scale(this.wrenNode, scale);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  };

  delete() {
    _wr_static_mesh_delete(this._wrenMesh);

    super.delete();
  };
}
