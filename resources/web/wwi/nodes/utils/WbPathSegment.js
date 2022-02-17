export default class WbPathSegment {
  constructor(startPoint, endPoint, initialRotation, radius, center, increment) {
    this.startPoint = startPoint;
    this.endPoint = endPoint;
    this.initialRotation = initialRotation;
    this.radius = radius;
    this.center = center;
    this.increment = increment;
  }
}
