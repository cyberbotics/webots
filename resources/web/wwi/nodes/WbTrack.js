import WbTrackWheel from './WbTrackWheel.js';
import WbTransform from './WbTransform.js';
import WbVector2 from './utils/WbVector2.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbWorld from './WbWorld.js';

import WbBeltPosition from './utils/WbBeltPosition.js';

export default class WbTrack extends WbTransform {
  constructor(id, translation, scale, rotation) {
    super(id, true, translation, scale, rotation);
    this.animatedObjectList = [];
    this.pathList = [];
    this.animationStepSize = 0;
    this.beltPosition = [];
    this.linearSpeed = 0.2;
  }

  postFinalize() {
    super.postFinalize();
    WbWorld.instance.tracks.push(this);
    this.initAnimatedGeometriesBeltPosition();
  }

  initAnimatedGeometriesBeltPosition() {
    const numGeometries = this.animatedObjectList.length;
    if (typeof this.pathLength !== 'undefined') {
      this.pathStepSize = this.pathLength / numGeometries;
      const beltPosition = new WbBeltPosition(this.pathList[0].startPoint, this.pathList[0].initialRotation, 0);
      this.firstGeometryPosition = beltPosition;
    }
  }

  animateMesh() {
    if (this.animatedObjectList.length === 0)
      return;

    // Retrieve the first TrackWheel children to deduce the speed of the track
    for (let i = 0; i < this.children.length; i++) {
      if (this.children[i] instanceof WbTrackWheel) {
        let velocity = this.children[i].angularVelocity;
        if (Math.abs(velocity.x) > Math.abs(velocity.y) && Math.abs(velocity.x) > Math.abs(velocity.y))
          this.linearSpeed = velocity.x;
        else if (Math.abs(velocity.y) > Math.abs(velocity.z))
          this.linearSpeed = velocity.y;
        else
          this.linearSpeed = velocity.z;

        this.linearSpeed *= this.children[i].radius;
        this.children[i].angularVelocity = new WbVector3();
        break;
      }
    }

    let stepSize = WbWorld.instance.basicTimeStep / 1000 * this.linearSpeed;
    this.animationStepSize = 0;
    let beltPosition = this.firstGeometryPosition;
    for (let i = 0; i < this.animatedObjectList.length; ++i) {
      let beltElement = WbWorld.instance.nodes.get(this.animatedObjectList[i]);
      if (typeof beltElement === 'undefined') {
        console.error('BeltElement not defined');
        return;
      }

      while (stepSize > this.pathStepSize)
        stepSize -= this.pathStepSize;
      while (stepSize < -this.pathStepSize)
        stepSize += this.pathStepSize;

      beltPosition = this.computeNextGeometryPosition(beltPosition, stepSize);

      const position = new WbVector3(beltPosition.position.x, 0.0, beltPosition.position.y);
      const rotation = new WbVector4(0.0, 1.0, 0.0, beltPosition.rotation);

      beltElement.translation = position;
      beltElement.rotation = rotation;

      beltElement.applyTranslationToWren();
      beltElement.applyRotationToWren();

      if (i === 0) {
        this.firstGeometryPosition = beltPosition;
        stepSize = this.pathStepSize;
      }
    }
  }

  computeNextGeometryPosition(currentBeltPosition, stepSize, segmentChanged) {
    if (stepSize === 0)
      return currentBeltPosition;

    const isPositiveStep = stepSize >= 0;
    const singleWheelCase = this.numberOfTrackWheel === 1;
    const segment = this.pathList[currentBeltPosition.segmentIndex];

    const endPoint = stepSize < 0 ? segment.startPoint : segment.endPoint;
    const maxDistanceVector = endPoint.sub(currentBeltPosition.position);

    let newStepSize = stepSize;
    console.log(endPoint)
    if (singleWheelCase || (!maxDistanceVector.isNull() && maxDistanceVector.length() > 1e-10)) {
      let maxStepSize = 0.0;
      if (segment.radius < 0) {
        // straight
        maxStepSize = maxDistanceVector.length();
        if (singleWheelCase || Math.abs(stepSize) <= maxStepSize) {
          const nextPosition = currentBeltPosition.position.add(segment.increment.mul(stepSize));
          return new WbBeltPosition(nextPosition, segment.initialRotation, currentBeltPosition.segmentIndex);
        }
      } else {
        // round
        let relativePosition = currentBeltPosition.position.sub(segment.center);
        if (singleWheelCase)
          maxStepSize = Math.abs(stepSize);
        else {
          let relativeEndPosition = endPoint.sub(segment.center);
          let c1, c2;
          if (isPositiveStep * segment.increment.x > 0) {
            c1 = relativePosition;
            c2 = relativeEndPosition;
          } else {
            c1 = relativeEndPosition;
            c2 = relativePosition;
          }
          let angle = Math.atan2(c2.x * c1.y - c2.y * c1.x, c2.x * c1.x + c2.y * c1.y);
          if (angle < 0)
            angle += 2 * Math.M_PI;

          maxStepSize = angle * segment.radius;
          if (maxStepSize < 0)
            maxStepSize += 2 * Math.PI;
        }

        if (Math.abs(stepSize) <= maxStepSize) {
          const angle = -(stepSize * segment.increment.x) / segment.radius;
          const newPositionX = relativePosition.x * Math.cos(angle) - relativePosition.y * Math.sin(angle);
          const newPositionY = relativePosition.y * Math.cos(angle) + relativePosition.x * Math.sin(angle);
          const nextPosition = segment.center.add(new WbVector2(newPositionX, newPositionY));
          let rotation = currentBeltPosition.rotation - angle;
          if (segmentChanged) {
            if (!isPositiveStep) {
              let previousStep = currentBeltPosition.segmentIndex + 1;
              if (previousStep === this.pathList.length)
                previousStep = 0;
              rotation = this.pathList[previousStep].initialRotation;
            } else
              rotation = segment.initialRotation;
            rotation -= angle;
          }
          return new WbBeltPosition(nextPosition, rotation, currentBeltPosition.segmentIndex);
        }
      }

      currentBeltPosition.position = endPoint;
      currentBeltPosition.rotation = 0.0;
      if (newStepSize < 0)
        newStepSize += maxStepSize;
      else
        newStepSize -= maxStepSize;
    }

    let nextSegmentIndex = currentBeltPosition.segmentIndex;
    if (isPositiveStep) {
      ++nextSegmentIndex;
      if (nextSegmentIndex === this.pathList.length)
        nextSegmentIndex = 0;
    } else {
      --nextSegmentIndex;
      if (nextSegmentIndex === -1)
        nextSegmentIndex = this.pathList.length - 1;
    }
    currentBeltPosition.segmentIndex = nextSegmentIndex;
    return this.computeNextGeometryPosition(currentBeltPosition, newStepSize, true);
  }
}
