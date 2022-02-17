import WbTransform from './WbTransform.js';
import WbWorld from './WbWorld.js';

export default class WbTrackWheel extends WbTransform {
  constructor(id, translation, scale, rotation) {
    super(id, false, translation, scale, rotation);
    this.surfaceVelocity = 0;
  }

  updateRotation(newRotation) {
    this.rotation = newRotation;
    if (WbWorld.instance.readyForUpdates)
      this.applyRotationToWren();
    WbWorld.instance.nodes.get(this.parent).animateMesh();
  }
}
