import WbBaseNode from './WbBaseNode.js';

import {getAnId} from './utils/utils.js';

// Also used to represent a solid
export default class WbSlot extends WbBaseNode {
  constructor(id, type = '', endPoint) {
    super(id);
    this.type = type;
    this.endPoint = endPoint;
  }
}
