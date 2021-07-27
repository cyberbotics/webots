import WbBaseNode from './WbBaseNode.js';

import {getAnId} from './utils/utils.js';

// Also used to represent a solid
export default class WbSlot extends WbBaseNode {
  constructor(id, type = '', endPoint = null) {
    super(id);
    this.type = type;
    this.endPoint = endPoint;
  }

  createWrenObjects(isTransform) {
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
