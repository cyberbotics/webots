import WbGeometry from './WbGeometry.js';
import WbWorld from './WbWorld.js';

export default class WbCapsule extends WbGeometry {
  constructor(id, radius, height, subdivision, bottom, side, top) {
    super(id);
    this.radius = radius;
    this.height = height;
    this.subdivision = subdivision;
    this.bottom = bottom;
    this.side = side;
    this.top = top;
  };

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
        throw new Error('Unknown parameter ' + parameterName + ' for node WbCapsule.');
    }
  };

  clone(customID) {
    this.useList.push(customID);
    return new WbCapsule(customID, this.radius, this.height, this.subdivision, this.bottom, this.side, this.top);
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

  // Private functions

  _sanitizeFields() {
    if (this.subdivision < 4)
      this.subdivision = 4;

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

    // Restore pickable state
    super.setPickable(this.isPickable);

    this._wrenMesh = _wr_static_mesh_capsule_new(this.subdivision, this.radius, this.height, this.side, this.top, this.bottom, false);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  };
}
