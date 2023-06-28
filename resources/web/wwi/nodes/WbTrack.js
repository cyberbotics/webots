import WbSolid from './WbSolid.js';
import WbVector2 from './utils/WbVector2.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbWorld from './WbWorld.js';

import WbBeltPosition from './utils/WbBeltPosition.js';
import WbPathSegment from './utils/WbPathSegment.js';
import { getAnId } from './utils/id_provider.js';
import { clampedAcos } from './utils/math_utilities.js';
import { WbNodeType } from './wb_node_type.js';

export default class WbTrack extends WbSolid {
  #device;
  constructor(id, translation, rotation, geometriesCount) {
    super(id, translation, rotation);
    this.geometriesCount = geometriesCount;
    this.pathList = [];
    this.wheelsList = [];
    this.beltElements = [];
    this.beltPositions = [];
    this.#device = [];
    this.linearSpeed = 0;
    this.animationStepSize = 0;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_TRACK;
  }

  get device() {
    return this.#device;
  }

  set device(device) {
    this.#device = device;
  }

  delete() {
    this.clearAnimatedGeometries();
    super.delete();
  }

  updateAnimatedGeometries() {
    if (typeof this.geometryField === 'undefined')
      return;

    this.initAnimatedGeometriesBeltPosition();

    let stepSize = 0;
    let beltPosition = this.firstGeometryPosition;

    for (let i = 0; i < this.geometriesCount; ++i) {
      beltPosition = this.computeNextGeometryPosition(beltPosition, stepSize);
      this.beltPositions.push(beltPosition);
      if (typeof beltPosition === 'undefined' || beltPosition.segmentIndex < 0) {
        // abort
        this.clearAnimatedGeometries();
        return;
      }

      const newElement = this.geometryField.clone(getAnId());
      newElement.parent = this.id;
      WbWorld.instance.nodes.set(newElement.id, newElement);

      newElement.translation = new WbVector3(beltPosition.position.x, 0.0, beltPosition.position.y);
      newElement.rotation = new WbVector4(0.0, 1.0, 0.0, beltPosition.rotation);

      if (i === 0)
        stepSize = this.pathStepSize;

      this.beltElements.push(newElement);
    }
  }

  clearAnimatedGeometries() {
    for (let i = 0; i < this.beltElements.length; i++)
      this.beltElements[i].delete();

    this.beltElements = undefined;
  }

  computeBeltPath() {
    this.pathLength = 0.0;
    const wheelsCount = this.wheelsList.length;

    if (wheelsCount <= 0)
      return;

    let center = new WbVector2(this.wheelsList[0].translation.x, this.wheelsList[0].translation.z);
    let radius = this.wheelsList[0].radius;
    if (wheelsCount === 1) {
      // round path
      const startPoint = new WbVector2(center.x, center.y + radius);
      this.pathList.push(new WbPathSegment(startPoint, startPoint, 0.0, radius, center, new WbVector2(1, 1)));
      this.pathLength = 2 * Math.PI * radius;
      return;
    }

    let firstPoint, previousPoint, distanceVector;
    let pointA, pointB;
    let previousRotation = 0.0;
    let nextIndex = 1;

    let wheelsPositionError = false;
    for (let w = 0; w < wheelsCount; ++w) {
      if (w === wheelsCount - 1)
        nextIndex = 0;

      let nextCenter = new WbVector2(this.wheelsList[nextIndex].translation.x, this.wheelsList[nextIndex].translation.z);
      let nextRadius = this.wheelsList[nextIndex].radius;
      distanceVector = nextCenter.sub(center);
      if (!wheelsPositionError && distanceVector.length() < 0.0000001) {
        wheelsPositionError = true;
        continue;
      }
      let wheelsAngle = Math.atan2(distanceVector.y, distanceVector.x);
      let absAngle = 0.0;
      let isWheelInner = this.wheelsList[w].inner;
      let isOuterTangent = isWheelInner === this.wheelsList[nextIndex].inner;
      if (isOuterTangent) {
        // outer tangent
        let relAngle = clampedAcos((radius - nextRadius) / distanceVector.length());

        if (!isWheelInner)
          relAngle = -relAngle;
        absAngle = relAngle + wheelsAngle;
        pointA = new WbVector2(Math.cos(absAngle), Math.sin(absAngle)).mul(radius).add(center);
        pointB = new WbVector2(Math.cos(absAngle), Math.sin(absAngle)).mul(nextRadius).add(nextCenter);
      } else {
        // inner tangent
        let relAngle = clampedAcos((radius + nextRadius) / distanceVector.length());
        if (!isWheelInner)
          relAngle = -relAngle;
        absAngle = relAngle + wheelsAngle;
        pointA = new WbVector2(radius * Math.cos(absAngle) + center.x, radius * Math.sin(absAngle) + center.y);
        pointB = new WbVector2(nextRadius * Math.cos(absAngle + Math.PI) + nextCenter.x,
          nextRadius * Math.sin(absAngle + Math.PI) + nextCenter.y);
      }

      if (w === 0)
        firstPoint = pointA;
      else {
        let c1, c2, inc;
        if (isWheelInner) {
          c1 = previousPoint.sub(center);
          c2 = pointA.sub(center);
          inc = new WbVector2(1, 1);
        } else {
          c2 = previousPoint.sub(center);
          c1 = pointA.sub(center);
          inc = new WbVector2(-1, -1);
        }
        let angle = Math.atan2(c2.x * c1.y - c2.y * c1.x, c2.x * c1.x + c2.y * c1.y);
        if (angle < 0)
          angle += 2 * Math.PI;
        this.pathLength += radius * angle;
        // round path
        this.pathList.push(new WbPathSegment(previousPoint, pointA, previousRotation, radius, center, inc));
      }

      // straight path
      distanceVector = pointB.sub(pointA);
      this.pathLength += distanceVector.length();
      if (Math.abs(distanceVector.y) < 1e-10 && Math.abs(distanceVector.x) < 1e-10) {
        const centerDistanceVector = nextCenter.sub(center);
        previousRotation = -Math.atan2(centerDistanceVector.y, centerDistanceVector.x) - Math.PI / 2;
      } else
        previousRotation = -Math.atan2(distanceVector.y, distanceVector.x);
      this.pathList.push(new WbPathSegment(pointA, pointB, previousRotation, -1, new WbVector2(), distanceVector.normalized()));

      previousPoint = pointB;
      center = nextCenter;
      radius = nextRadius;
      ++nextIndex;
    }

    // add last round path
    radius = this.wheelsList[0].radius;
    const firstWheelCenter = new WbVector2(this.wheelsList[0].translation.x, this.wheelsList[0].translation.z);
    const c1 = previousPoint.sub(firstWheelCenter);
    const c2 = firstPoint.sub(firstWheelCenter);
    let angle = Math.atan2(c2.x * c1.y - c2.y * c1.x, c2.x * c1.x + c2.y * c1.y);
    if (angle < 0)
      angle += 2 * Math.PI;
    this.pathLength += radius * angle;
    this.pathList.push(new WbPathSegment(previousPoint, firstPoint, previousRotation, radius, firstWheelCenter,
      new WbVector2(1, 1)));

    if (wheelsPositionError)
      // multiple wheels at the same location
      console.error('Two or more consecutive TrackWheel nodes are located at the same position. Only the first node is used.');
  }

  preFinalize() {
    super.preFinalize();
    this.computeBeltPath();
    this.updateAnimatedGeometries();
    this.beltElements?.forEach(beltElement => beltElement.preFinalize());
  }

  createWrenObjects(isTransform) {
    super.createWrenObjects();
    this.beltElements?.forEach(beltElement => beltElement.createWrenObjects());
  }

  postFinalize() {
    super.postFinalize();
    WbWorld.instance.tracks.add(this);
    this.beltElements?.forEach(beltElement => beltElement.postFinalize());
  }

  initAnimatedGeometriesBeltPosition() {
    if (typeof this.pathLength !== 'undefined' && this.pathLength > 0) {
      this.pathStepSize = this.pathLength / this.geometriesCount;
      const beltPosition = new WbBeltPosition(this.pathList[0].startPoint, this.pathList[0].initialRotation, 0);
      this.firstGeometryPosition = beltPosition;
    }
  }

  animateMesh() {
    if (this.geometriesCount === 0 || typeof this.geometryField === 'undefined')
      return;

    let stepSize = WbWorld.instance.basicTimeStep / 1000 * this.linearSpeed;
    this.animationStepSize = 0;
    let beltPosition = this.firstGeometryPosition;
    for (let i = 0; i < this.geometriesCount; ++i) {
      const beltElement = this.beltElements[i];
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
        this.firstGeometryPosition = beltPosition.clone();
        stepSize = this.pathStepSize;
      }
    }
  }

  computeNextGeometryPosition(currentBeltPosition, stepSize, segmentChanged) {
    if (stepSize === 0)
      return currentBeltPosition;
    const isPositiveStep = stepSize >= 0;
    const singleWheelCase = this.wheelsList.length === 1;
    const segment = this.pathList[currentBeltPosition.segmentIndex];
    const endPoint = stepSize < 0 ? segment.startPoint : segment.endPoint;
    const maxDistanceVector = endPoint.sub(currentBeltPosition.position);
    let newStepSize = stepSize;

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
        const relativePosition = currentBeltPosition.position.sub(segment.center);
        if (singleWheelCase)
          maxStepSize = Math.abs(stepSize);
        else {
          const relativeEndPosition = endPoint.sub(segment.center);
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
            angle += 2 * Math.PI;

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
