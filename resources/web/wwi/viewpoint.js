/* global THREE */
'use strict';

class Viewpoint { // eslint-disable-line no-unused-vars
  constructor() {
    this.onCameraParametersChanged = null;
    // After initialization 'followedObjectId' contains the id ('n<id>') of the followed node
    // or 'none' if no object is followed.
    this.followedObjectId = null;
    // If the followed object has moved since the last time we updated the viewpoint position, this field will contain a
    // vector with the translation applied to the object.
    this.followedObjectDeltaPosition = null;
    this.viewpointMass = 1.0; // Mass of the viewpoint used during the object following algorithm.
    this.viewpointFriction = 0.05; // Friction applied to the viewpoint whenever it is going faster than the followed object.
    this.viewpointForce = null; // Vector with the force that will be applied to the viewpoint for the next delta T.
    this.viewpointVelocity = null; // Current velocity of the viewpoint.
    this.viewpointLastUpdate = undefined; // Last time we updated the position of the viewpoint.

    // Initialize default camera.
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.001, 400);
    this.camera.position.x = 10;
    this.camera.position.y = 10;
    this.camera.position.z = 10;
  }

  reset(time) {
    this.camera.position.copy(this.initialViewpointPosition);
    this.camera.quaternion.copy(this.initialViewpointOrientation);
    this.updateViewpointPosition(true, time);
    this.notifyCameraParametersChanged();
  }

  isFollowedObject(object) {
    return this.followedObjectId &&
           (object.name === this.followedObjectId || object.userData.name === this.followedObjectId);
  }

  resetFollow() {
    this.followedObjectId = null;
  }

  initFollowParameters() {
    this.initialViewpointPosition = this.camera.position.clone();
    this.initialViewpointOrientation = this.camera.quaternion.clone();
    if (this.camera.userData.followSmoothness != null)
      this.setViewpointMass(this.camera.userData.followSmoothness);
    if (this.camera.userData.followedId != null)
      this.follow(this.camera.userData.followedId);
    else
      this.follow.followedObjectId = 'none';
  }

  follow(objectId) {
    this.followedObjectId = objectId;
    this.viewpointForce = new THREE.Vector3(0.0, 0.0, 0.0);
    this.viewpointVelocity = new THREE.Vector3(0.0, 0.0, 0.0);
  }

  setViewpointMass(mass) {
    this.viewpointMass = mass;
    if (this.viewpointMass <= 0.05)
      this.viewpointMass = 0.0;
    else {
      if (this.viewpointMass > 1.0)
        this.viewpointMass = 1.0;
      this.friction = 0.05 / this.viewpointMass;
    }
  }

  setFollowedObjectDeltaPosition(newPosition, previousPosition) {
    this.followedObjectDeltaPosition = new THREE.Vector3();
    this.followedObjectDeltaPosition.subVectors(newPosition, previousPosition);
  }

  updateViewpointPosition(forcePosition, time) {
    if (this.followedObjectId == null || this.followedObjectId === 'none' || typeof time === 'undefined')
      return false;
    if (typeof this.viewpointLastUpdate === 'undefined')
      this.viewpointLastUpdate = time;

    var timeInterval = Math.abs(time - this.viewpointLastUpdate) / 1000;

    if (timeInterval > 0 && this.camera) {
      this.viewpointLastUpdate = time;
      var viewpointDeltaPosition = null;
      if (this.followedObjectDeltaPosition != null)
        this.viewpointForce.add(this.followedObjectDeltaPosition);

      // Special case: if the mass is 0 we simply move the viewpoint to its equilibrium position.
      // If timeInterval is too large (longer than 1/10 of a second), the progression won't be smooth either way,
      // so in this case we simply move the viewpoint to the equilibrium position as well.
      if (forcePosition || this.viewpointMass === 0 || (timeInterval > 0.1 && this.animation == null)) {
        viewpointDeltaPosition = this.viewpointForce.clone();
        this.viewpointVelocity = new THREE.Vector3(0.0, 0.0, 0.0);
      } else {
        var acceleration = this.viewpointForce.clone();
        acceleration.multiplyScalar(timeInterval / this.viewpointMass);
        this.viewpointVelocity.add(acceleration);
        var scalarVelocity = this.viewpointVelocity.length();

        // Velocity of the object projected onto the velocity of the viewpoint.
        var scalarObjectVelocityProjection;
        if (this.followedObjectDeltaPosition != null) {
          var objectVelocity = this.followedObjectDeltaPosition.clone();
          objectVelocity.divideScalar(timeInterval);
          scalarObjectVelocityProjection = objectVelocity.dot(this.viewpointVelocity) / scalarVelocity;
        } else
          scalarObjectVelocityProjection = 0;

        // The viewpoint is going "faster" than the object, to prevent oscillations we apply a slowing force.
        if (this.viewpointFriction > 0 && scalarVelocity > scalarObjectVelocityProjection) {
          // We apply a friction based on the extra velocity.
          var velocityFactor = (scalarVelocity - (scalarVelocity - scalarObjectVelocityProjection) * this.viewpointFriction) / scalarVelocity;
          this.viewpointVelocity.multiplyScalar(velocityFactor);
        }
        viewpointDeltaPosition = this.viewpointVelocity.clone();
        viewpointDeltaPosition.multiplyScalar(timeInterval);
      }
      this.viewpointForce.sub(viewpointDeltaPosition);
      this.camera.position.add(viewpointDeltaPosition);
      this.followedObjectDeltaPosition = null;
      return true;
    }

    return false;
  }

  rotate(params) {
    var yawAngle = -0.005 * params.dx;
    var pitchAngle = -0.005 * params.dy;
    if (params.pickPosition == null) {
      yawAngle /= -8;
      pitchAngle /= -8;
    }
    var voMatrix = new THREE.Matrix4();
    var pitch = new THREE.Vector3();
    var yaw = new THREE.Vector3();
    voMatrix.makeRotationFromQuaternion(this.camera.quaternion).extractBasis(pitch, yaw, new THREE.Vector3());
    var pitchRotation = new THREE.Quaternion();
    pitchRotation.setFromAxisAngle(pitch, pitchAngle * 2);
    var worldYawRotation = new THREE.Quaternion();
    worldYawRotation.setFromAxisAngle(new THREE.Vector3(0, 1, 0), yawAngle * 2); // axis: world up
    var deltaRotation = worldYawRotation.multiply(pitchRotation);
    if (params.pickPosition)
      this.camera.position.sub(params.pickPosition).applyQuaternion(deltaRotation).add(params.pickPosition);
    this.camera.quaternion.premultiply(deltaRotation);

    this.notifyCameraParametersChanged();
  }

  translate(params) {
    var voMatrix = new THREE.Matrix4();
    var pitch = new THREE.Vector3();
    var yaw = new THREE.Vector3();
    voMatrix.makeRotationFromQuaternion(this.camera.quaternion).extractBasis(pitch, yaw, new THREE.Vector3());
    var targetRight = -params.scaleFactor * params.dx;
    var targetUp = params.scaleFactor * params.dy;
    this.camera.position.addVectors(params.initialCameraPosition, pitch.multiplyScalar(targetRight).add(yaw.multiplyScalar(targetUp)));

    this.notifyCameraParametersChanged();
  }

  zoomAndTilt(params) {
    var voMatrix = new THREE.Matrix4();
    var roll = new THREE.Vector3();
    voMatrix.makeRotationFromQuaternion(this.camera.quaternion).extractBasis(new THREE.Vector3(), new THREE.Vector3(), roll);

    this.camera.position.add(roll.clone().multiplyScalar(params.zoomScale));

    var zRotation = new THREE.Quaternion();
    zRotation.setFromAxisAngle(roll, params.tiltAngle);
    this.camera.quaternion.premultiply(zRotation);

    this.notifyCameraParametersChanged();
  }

  zoom(distance, deltaY) {
    var scaleFactor = 0.02 * distance * ((deltaY < 0) ? -1 : 1);
    var voMatrix = new THREE.Matrix4();
    var roll = new THREE.Vector3();
    voMatrix.makeRotationFromQuaternion(this.camera.quaternion).extractBasis(new THREE.Vector3(), new THREE.Vector3(), roll);

    this.camera.position.add(roll.multiplyScalar(scaleFactor));
    this.notifyCameraParametersChanged();
  }

  notifyCameraParametersChanged(updateScene = true) {
    if (typeof this.onCameraParametersChanged === 'function')
      this.onCameraParametersChanged(updateScene);
  }
}
