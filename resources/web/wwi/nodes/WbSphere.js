import WbGeometry from './WbGeometry.js';
import WbWorld from './WbWorld.js';

export default class WbSphere extends WbGeometry {
  constructor(id, radius = 0.1, ico = false, subdivision = 1) {
    super(id);
    this.radius = radius;
    this.ico = ico;
    this.subdivision = subdivision;
  };

  setParameter(parameterName, parameterValue) {
    switch (parameterName) {
      case 'radius':
        this.radius = parameterValue;
        this.updateRadius();
        break;
      case 'ico':
        this.ico = parameterValue;
        this.updateMesh();
        break;
      case 'subdivision':
        this.subdivision = parameterValue;
        this.updateMesh();
        break;
      default:
        throw new Error('Unknown parameter ' + parameterName + ' for node WbSphere.');
    }
  };

  clone(customID) {
    this.useList.push(customID);
    return new WbSphere(customID, this.radius, this.ico, this.subdivision);
  };

  createWrenObjects() {
    super.createWrenObjects();
    this._sanitizeFields();
    this._buildWrenMesh();
  };

  delete() {
    _wr_static_mesh_delete(this._wrenMesh);

    super.delete();
  };

  updateLineScale() {
    if (!this._isAValidBoundingObject())
      return;

    const offset = _wr_config_get_line_scale() / WbGeometry.LINE_SCALE_FACTOR;
    const scaledRadius = this.radius * (1.0 + offset);
    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(scaledRadius, scaledRadius, scaledRadius));
  };

  updateScale() {
    const scaledRadius = this.radius;

    _wr_transform_set_scale(this.wrenNode, _wrjs_array3(scaledRadius, scaledRadius, scaledRadius));
  };

  updateRadius() {
    this._sanitizeFields();

    if (super.isInBoundingObject())
      this.updateLineScale();
    else
      this.updateScale();
  };

  updateMesh() {
    this._sanitizeFields();

    this._buildWrenMesh();
    const parent = WbWorld.instance.nodes.get(this.parent); // needed otherwise it is no longer visible
    parent.updateAppearance();
    parent.updateIsPickable();
  };

  // Private functions

  _sanitizeFields() {
    if (this.ico) {
      this.subdivision = this.subdivision > 5 ? 5 : this.subdivision;
      this.subdivision = this.subdivision < 1 ? 1 : this.subdivision;
    } else {
      this.subdivision = this.subdivision > 32 ? 32 : this.subdivision;
      this.subdivision = this.subdivision < 3 ? 3 : this.subdivision;
    }

    if (this.radius < 0)
      this.radius = 1
  };

  _isAValidBoundingObject() {
    return super._isAValidBoundingObject() && this.radius > 0;
  };

  _buildWrenMesh() {
    super._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    super._computeWrenRenderable();

    const createOutlineMesh = super.isInBoundingObject();
    this._wrenMesh = _wr_static_mesh_unit_sphere_new(this.subdivision, this.ico, false);

    // Restore pickable state
    super.setPickable(this.isPickable);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);

    if (createOutlineMesh)
      this.updateLineScale();
    else
      this.updateScale();
  };
}
