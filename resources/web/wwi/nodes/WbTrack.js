import WbTransform from './WbTransform.js';
import WbWorld from './WbWorld.js';

import WbBeltPosition from './utils/WbBeltPosition.js';

export default class WbTrack extends WbTransform {
  constructor(id, translation, scale, rotation) {
    super(id, true, translation, scale, rotation);
    this.animatedObjectList = [];
    this.pathList = [];
    this.animationStepSize = 0;
  }

  postFinalize() {
    super.postFinalize();
    WbWorld.instance.tracks.push(this);
    this.initAnimatedGeometriesBeltPosition();
  }

  initAnimatedGeometriesBeltPosition() {
    const numGeometries = this.animatedObjectList.length;
    this.pathStepSize = this.pathLength / numGeometries;
    const beltPosition = new WbBeltPosition(this.pathList[0].startPoint, this.pathList[0].initialRotation, 0);
    this.firstGeometryPosition = beltPosition;
  }

  /*animateMesh() {
    if (this.animatedObjectList.length === 0)
      return;

    let stepSize = this.animationStepSize;
    this.animationStepSize = 0;

    let beltPosition = this.firstGeometryPosition;
    for (let i = 0; i < this.animatedObjectList.length; ++i) {
      beltPosition = computeNextGeometryPosition(beltPosition, stepSize);
      mBeltPositions[i] = beltPosition;
      if (beltPosition.segmentIndex < 0) {
        // abort
        clearAnimatedGeometries();
        return;
      }

      float position[3];
      float rotation[4];
      WbVector3(beltPosition.position.x(), 0.0, beltPosition.position.y()).toFloatArray(position);
      WbRotation(0.0, 1.0, 0.0, beltPosition.rotation).toFloatArray(rotation);

      wr_transform_set_position(mBeltElements[i], position);
      wr_transform_set_orientation(mBeltElements[i], rotation);

      if (i == 0) {
        mFirstGeometryPosition = beltPosition;
        stepSize = mPathStepSize;
      }
    }
  }*/
}
