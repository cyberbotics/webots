import WbPose from './WbPose.js';
import WbWorld from './WbWorld.js';
import { WbNodeType } from './wb_node_type.js';

export default class WbTrackWheel extends WbPose {
  constructor(id, translation, rotation, radius, inner) {
    super(id, translation, rotation);
    this.radius = radius;
    this.inner = inner;
    this.angularVelocity = 0;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_TRACK_WHEEL;
  }

  updateRotation(newRotation) {
    const currQuat = Quaternion.fromAxisAngle([this.rotation.x, this.rotation.y, this.rotation.z], this.rotation.w);
    const nextQuat = Quaternion.fromAxisAngle([newRotation.x, newRotation.y, newRotation.z], newRotation.w);

    this.rotation = newRotation;
    let angle = nextQuat.mul(currQuat.inverse());
    angle = angle.w < 0.0 ? angle.neg() : angle; // Take absolute value
    angle = angle.log().mul(2); // Convert to scaled axis angle
    this.angularVelocity = angle.div(WbWorld.instance.basicTimeStep / 1000);

    if (WbWorld.instance.readyForUpdates)
      this.applyRotationToWren();
    const track = WbWorld.instance.nodes.get(this.parent);
    if (typeof track !== 'undefined' && track.linearSpeed === 0) {
      const velocity = this.angularVelocity;
      if (Math.abs(velocity.x) > Math.abs(velocity.y) && Math.abs(velocity.x) > Math.abs(velocity.y))
        track.linearSpeed = velocity.x;
      else if (Math.abs(velocity.y) > Math.abs(velocity.z))
        track.linearSpeed = velocity.y;
      else
        track.linearSpeed = velocity.z;

      track.linearSpeed *= this.radius;
    }
  }
}
