import WbBaseNode from './WbBaseNode.js';

// Also used to represent a solid
export default class WbHingeJoint extends WbBaseNode {
  constructor(id, endPoint = null) {
    super(id);
    this.endPoint = endPoint;
  }

  createWrenObjects() {
    super.createWrenObjects();

    if (typeof this.endPoint !== 'undefined' && this.endPoint !== null)
      this.endPoint.createWrenObjects();
  }

  preFinalize() {
    super.preFinalize();

    if (typeof this.endPoint !== 'undefined' && this.endPoint !== null)
      this.endPoint.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    if (typeof this.endPoint !== 'undefined' && this.endPoint !== null)
      this.endPoint.postFinalize();
  }
}
