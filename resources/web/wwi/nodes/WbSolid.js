import WbTransform from './WbTransform.js';

import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbSFBool from './utils/WbSFBool.js';

import {getAnId} from './utils/utils.js';

// Also used to represent a solid
export default class WbSolid extends WbTransform {
  constructor(id, isSolid = WbSFBool(false), translation = new WbVector3(0, 0, 0), scale = new WbVector3(1, 1, 1), rotation = new WbVector4(0, 1, 0, 0)) {
    super(id, false, translation, scale, rotation);
  }
}
