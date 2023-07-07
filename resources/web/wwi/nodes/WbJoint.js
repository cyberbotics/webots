import WbBaseNode from './WbBaseNode.js';
import WbSolid from './WbSolid.js';
import WbWorld from './WbWorld.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbJoint extends WbBaseNode {
  #endPoint;
  #jointParameters;
  #position;

  get endPoint() {
    return this.#endPoint;
  }

  set endPoint(endPoint) {
    this._endPointZeroRotation = undefined;
    this._endPointZeroTranslation = undefined;
    this.#endPoint = endPoint;
  }

  get jointParameters() {
    return this.#jointParameters;
  }

  set jointParameters(jointParameters) {
    this.#jointParameters = jointParameters;

    if (typeof this.#jointParameters !== 'undefined')
      this.#jointParameters.onChange = () => this._updatePosition();
  }

  get position() {
    return this.#position;
  }

  set position(newPosition) {
    if (this.#position === newPosition)
      return;

    this.#position = newPosition;
    if (typeof this.jointParameters !== 'undefined')
      this._updatePosition();
  }

  boundingSphere() {
    const solid = this.solidEndPoint();
    return solid?.boundingSphere();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.#endPoint?.createWrenObjects();
  }

  delete() {
    const parent = WbWorld.instance.nodes.get(this.parent);
    if (typeof parent !== 'undefined') {
      if (typeof parent.endPoint !== 'undefined')
        parent.endPoint = undefined;
      else {
        const index = parent.children.indexOf(this);
        parent.children.splice(index, 1);
      }
    }

    this.#jointParameters?.delete();
    this.#endPoint?.delete();

    super.delete();
  }

  preFinalize() {
    super.preFinalize();

    this.#position = typeof this.jointParameters === 'undefined' ? 0 : this.jointParameters.position;
    this._updateEndPointZeroTranslationAndRotation();

    this.#jointParameters?.preFinalize();
    this.#endPoint?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#jointParameters?.postFinalize();
    this.#endPoint?.postFinalize();
  }

  solidEndPoint() {
    if (typeof this.endPoint === 'undefined')
      return;

    if (this.endPoint.nodeType === WbNodeType.WB_NODE_SLOT) {
      const childrenSlot = this.endPoint.slotEndPoint();
      return childrenSlot?.solidEndPoint();
    } else if (this.endPoint instanceof WbSolid)
      return this.endPoint;
  }

  updateBoundingObjectVisibility() {
    this.#endPoint?.updateBoundingObjectVisibility();
  }

  _updatePosition() {}
  _updateEndPointZeroTranslationAndRotation() {}
}
