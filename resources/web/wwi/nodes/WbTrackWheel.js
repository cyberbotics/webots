import WbTransform from './WbTransform.js';
import WbWorld from './WbWorld.js';

export default class WbTrackWheel extends WbTransform {
  constructor(id, translation, scale, rotation, radius) {
    super(id, false, translation, scale, rotation);
    this.radius = radius;
    this.angularVelocity = 0;
  }

  updateRotation(newRotation) {
    let currQuat = Quaternion.fromAxisAngle([this.rotation.x, this.rotation.y, this.rotation.z], this.rotation.w);
    let nextQuat = Quaternion.fromAxisAngle([newRotation.x, newRotation.y, newRotation.z], newRotation.w);
    this.rotation = newRotation;
    this.angularVelocity = quatDifferentiateAngularVelocity(nextQuat, currQuat, WbWorld.instance.basicTimeStep / 1000);
    if (WbWorld.instance.readyForUpdates)
      this.applyRotationToWren();
    WbWorld.instance.nodes.get(this.parent).animateMesh();
  }
}

function quatDifferentiateAngularVelocity(nextQuat, currQuat, dt) {
  return quatToScaledAxisAngle(quatAbs(nextQuat.mul(currQuat.inverse()))).div(dt);
}

function quatAbs(x) {
  return x.w < 0.0 ? x.neg() : x;
}

function quatToScaledAxisAngle(q) {
  return q.log().mul(2);
}
