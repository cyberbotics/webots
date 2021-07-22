import WbGeometry from './WbGeometry.js';
import WbWorld from './WbWorld.js';

export default class WbCone extends WbGeometry {
  constructor(id, bottomRadius, height, subdivision, side, bottom) {
    super(id);
    this.bottomRadius = bottomRadius;
    this.height = height;
    this.subdivision = subdivision;
    this.side = side;
    this.bottom = bottom;
  };

  setParameter(parameterName, parameterValue) {
    switch (parameterName) {
      case 'bottomRadius':
        this.bottomRadius = parameterValue;
        this.updateBottomRadius();
        break;
      case 'height':
        this.height = parameterValue;
        this.updateHeight();
        break;
      case 'subdivision':
        this.subdivision = parameterValue;
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
    return new WbCone(customID, this.bottomRadius, this.height, this.subdivision, this.side, this.bottom);
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

  updateBottomRadius() {
    this._sanitizeFields();

    this.updateScale();
  };

  updateHeight() {
    this._sanitizeFields();

    this.updateScale();
  };

  updateScale() {
    const scale = _wrjs_array3(this.bottomRadius, this.height, this.bottomRadius);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  updateMesh() {
    this._sanitizeFields();

    this._buildWrenMesh();
    const parent = WbWorld.instance.nodes.get(this.parent); // needed otherwise it is no longer visible
    parent.updateAppearance();
    parent.updateIsPickable();
  };

  _sanitizeFields() {
    if (this.subdivision < 3)
      this.subdivision = 3;

    if (this.bottomRadius < 0)
      this.bottomRadius = 1;

    if (this.height < 0)
      this.height = 2;
  };

  _buildWrenMesh() {
    this._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    if (!this.bottom && !this.side)
      return;

    this._computeWrenRenderable();

    this._wrenMesh = _wr_static_mesh_unit_cone_new(this.subdivision, this.side, this.bottom);

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);

    this.updateScale();
  }
}
