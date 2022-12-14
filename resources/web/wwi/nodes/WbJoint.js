import WbBaseNode from './WbBaseNode.js';
import WbWorld from './WbWorld.js';

export default class WbJoint extends WbBaseNode {
  #endPoint;
  #jointParameters;

  get endPoint() {
    return this.#endPoint;
  }

  set endPoint(endPoint) {
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

  updateBoundingObjectVisibility() {
    this.#endPoint?.updateBoundingObjectVisibility();
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
    this.#jointParameters?.preFinalize();
    this.#endPoint?.preFinalize();

    this.position = typeof this.jointParameters === 'undefined' ? 0 : this.jointParameters.position;
    this._updateEndPointZeroTranslationAndRotation();
  }

  postFinalize() {
    super.postFinalize();

    this.#jointParameters?.postFinalize();
    this.#endPoint?.postFinalize();
  }

  _updatePosition() {}
  _updateEndPointZeroTranslationAndRotation() {}
}
