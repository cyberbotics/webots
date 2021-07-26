import WbTransform from './WbTransform.js';

import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';

import {getAnId} from './utils/utils.js';

// Also used to represent a solid
export default class WbSolid extends WbTransform {
  constructor(id, isSolid = false, translation, scale, rotation) {
    super(id, false, translation, scale, rotation);
  }
}
