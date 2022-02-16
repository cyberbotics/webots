export default class WbPathSegment {
  constructor(startPoint, endPoint, rotation, initialRotation, radius, center, increment) {
    this.startPoint = startPoint;
    this.endPoint = endPoint;
    this.rotation = rotation;
    this.initialRotation = initialRotation;
    this.radius = radius;
    this.center = center;
    this.increment = increment;
  }
}
