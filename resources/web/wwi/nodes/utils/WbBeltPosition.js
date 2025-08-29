export default class WbBeltPosition {
  constructor(position, angle, index) {
    this.position = position; // WbVector2
    this.rotation = angle; // Double
    this.segmentIndex = index; // Int
  }

  clone() {
    return new WbBeltPosition(this.position, this.rotation, this.segmentIndex);
  }
}
