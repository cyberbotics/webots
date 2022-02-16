import WbTransform from './WbTransform.js';
import WbWorld from './WbWorld.js';

export default class WbTrack extends WbTransform {
  constructor(id, translation, scale, rotation) {
    super(id, true, translation, scale, rotation);
    this.animatedObjectList = [];
    this.pathList = [];
  }

  postFinalize() {
    super.postFinalize();
    WbWorld.instance.tracks.push(this);
  }
}
