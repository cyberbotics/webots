export default class WbPathSegment {
  constructor(startPoint, endPoint, initialRotation, radius, center, increment) {
    this.startPoint = startPoint; // WbVector2
    this.endPoint = endPoint; // WbVector2
    this.initialRotation = initialRotation; // Double
    this.radius = radius; // Double
    this.center = center; // WbVector2
    this.increment = increment; // Wbvector2
  }
}
