export default class WbWrenCamera {
  static computeFieldOfViewY(fovX, aspectRatio) {
    return 2 * Math.atan(Math.tan(fovX * 0.5) / aspectRatio);
  }
}
