/* global THREE */
'use strict';

function Viewpoint(camera) {
  this.camera = camera;
  this.onCameraPositionChanged = null;
  // after initialization 'followedObjectId' contains the id ('n<id>') of the followed node or 'none' if no object is followed
  this.followedObjectId = null;
  // If the followed object has moved since the last time we updated the viewpoint position, this field will contain a
  // vector with the translation applied to the object.
  this.followedObjectDeltaPosition = null;
  this.viewpointMass = 1.0; // Mass of the viewpoint used during the object following algorithm.
  this.viewpointFriction = 0.05; // Friction applied to the viewpoint whenever it is going faster than the followed object.
  this.viewpointForce = null; // Vector with the force that will be applied to the viewpoint for the next delta T.
  this.viewpointVelocity = null; // Current velocity of the viewpoint.
  this.viewpointLastUpdate = undefined; // Last time we updated the position of the viewpoint.
};

Viewpoint.prototype = {
  constructor: Viewpoint,

  reset: function(time) {
    this.camaera.position.copy(this.initialViewpointPosition);
    this.camaera.quaternion.copy(this.view.initialViewpointOrientation);
    this.updateViewpointPosition(true, time);
  },

  isFollowedObject: function(object) {
    return this.followedObjectId &&
           (object.name === this.followedObjectId || object.userData.name === this.followedObjectId);
  },

  resetFollow: function() {
    this.followedObjectId = null;
  },

  initFollowParameters: function() {
    this.initialViewpointPosition = this.camera.position;
    this.initialViewpointOrientation = this.camera.quaternion;
    if (this.camera.userData.followSmoothness !== null)
      this.setViewpointMass(this.camera.userData.followSmoothness);
    if (this.camera.userData.followedId != null) {
      this.followedObjectId = this.camera.userData.followedId;
      this.follow(this.camera.userData.followedId);
    } else
      this.follow.followedObjectId = 'none';
  },

  follow: function(objectId) {
    this.followedObjectId = objectId;
    this.viewpointForce = new THREE.Vector3(0.0, 0.0, 0.0);
    this.viewpointVelocity = new THREE.Vector3(0.0, 0.0, 0.0);
  },

  setViewpointMass: function(mass) {
    this.viewpointMass = mass;
    if (this.viewpointMass <= 0.05)
      this.viewpointMass = 0.0;
    else {
      if (this.viewpointMass > 1.0)
        this.viewpointMass = 1.0;
      this.friction = 0.05 / this.viewpointMass;
    }
  },

  setFollowedObjectDeltaPosition: function(newPosition, previousPosition) {
    this.followedObjectDeltaPosition = new THREE.Vector3();
    this.followedObjectDeltaPosition.subVectors(newPosition, previousPosition);
  },

  updateViewpointPosition: function(forcePosition, time) {
    if (this.followedObjectId == null || this.followedObjectId === 'none' || time === undefined)
      return;
    if (this.viewpointLastUpdate === undefined)
      this.viewpointLastUpdate = time;

    var timeInterval = Math.abs(time - this.viewpointLastUpdate) / 1000;

    if (timeInterval > 0 && this.camera) {
      this.viewpointLastUpdate = time;
      var viewpointPosition = this.camera.position;
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
      var viewpointNewPosition = new THREE.Vector3();
      viewpointNewPosition.addVectors(viewpointPosition, viewpointDeltaPosition);
      this.viewpointForce.sub(viewpointDeltaPosition);
      this.camera.position.copy(viewpointNewPosition);
      if (this.onCameraPositionChanged)
        this.onCameraPositionChanged();
      this.followedObjectDeltaPosition = null;
    }
  },

  startRotateViewpoint: function() {

  },

  rotate: function(params) {
    var vo = this.camera.quaternion;
    var vp = this.camera.position.clone();
    var cosW = Math.cos(vo.w);
    var sinW = Math.sin(vo.w);
    var halfYawAngle = -0.005 * params.dx;
    var halfPitchAngle = -0.005 * params.dy;
    if (params.pickPosition == null) {
      halfYawAngle /= -8;
      halfPitchAngle /= -8;
    }
    var sinusYaw = Math.sin(halfYawAngle);
    var sinusPitch = Math.sin(halfPitchAngle);
    var tx = (1 - cosW) * vo.x;
    var pitch = new THREE.Vector3(tx * vo.x + cosW, tx * vo.y + sinW * vo.z, tx * vo.z - sinW * vo.y);
    var pitchRotation = new THREE.Quaternion(sinusPitch * pitch.x, sinusPitch * pitch.y, sinusPitch * pitch.z, Math.cos(halfPitchAngle));
    var worldUp = new THREE.Vector3(0, 1, 0);
    var yawRotation = new THREE.Quaternion(sinusYaw * worldUp.x, sinusYaw * worldUp.y, sinusYaw * worldUp.z, Math.cos(halfYawAngle));
    var deltaRotation = yawRotation.multiply(pitchRotation);
    if (params.pickPosition) {
      var rotationMatrix = new THREE.Matrix4();
      rotationMatrix.makeRotationFromQuaternion(deltaRotation);
      var currentPosition = vp.sub(params.pickPosition).applyMatrix(rotationMatrix).add(params.pickPosition);
      this.camera.position.copy(currentPosition);
    }
    var voq = new THREE.Quaternion();
    voq.setFromAxisAngle(new THREE.Vector3(vo.x, vo.y, vo.z), vo.w);
    var currentOrientation = deltaRotation.multiply(voq);
    this.camera.quaternion.copy(currentOrientation);

    if (this.onCameraPositionChanged)
      this.onCameraPositionChanged();
  },

  translate: function(params) {
    var vo = this.camera.quaternion;
    var vp = this.camera.position.clone();
    var cosW = Math.cos(vo.w);
    var sinW = Math.sin(vo.w);
    var targetRight = -params.distanceToPickPosition * params.scaleFactor * params.dx;
    var targetUp = params.distanceToPickPosition * params.scaleFactor * params.dy;
    var tx = (1 - cosW) * vo.x;
    var pitch = new THREE.Vector3(tx * vo.x + cosW, tx * vo.y + sinW * vo.z, tx * vo.z - sinW * vo.y);
    var ty = (1 - cosW) * vo.y;
    var yaw = new THREE.Vector3(ty * vo.x - sinW * vo.z, ty * vo.y + cosW, ty * vo.z + sinW * vo.x);
    var target = vp.add(pitch.multiplyScalar(targetRight).add(yaw.multiplyScalar(targetUp)));
    this.camera.position.copy(target);
  },

  zoomAndTilt: function(params) {
    var vo = this.camera.quaternion;
    var vp = this.camera.position.clone();
    var cosW = Math.cos(vo.w);
    var sinW = Math.sin(vo.w);
    var tz = (1 - cosW) * vo.z;
    var roll = new THREE.Vector3(tz * vo.x + sinW * vo.y, tz * vo.y - sinW * vo.x, tz * vo.z + cosW);
    var target = vp.add(roll.multiplyScalar(params.zoomScale));
    this.camera.position.copy(target);

    var zRotation = new THREE.Quaternion();
    zRotation.setFromAxisAngle(roll, params.tiltAngle);
    var voq = new THREE.Quaternion();
    voq.setFromAxisAngle(new THREE.Vector3(vo.x, vo.y, vo.z), vo.w);
    var orientation = zRotation.multiply(voq);
    this.camera.quaternion.copy(orientation);
  },

  zoom: function(distance, deltaY) {
    var vo = this.camera.quaternion;
    var vp = this.camera.position.clone();
    var scaleFactor = 0.02 * distance * ((deltaY < 0) ? -1 : 1);
    var c = Math.cos(vo.w);
    var s = Math.sin(vo.w);
    var tz = (1 - c) * vo.z;
    var roll = new THREE.Vector3(tz * vo.x + s * vo.y, tz * vo.y - s * vo.x, tz * vo.z + c);
    var target = vp.add(roll.multiplyScalar(scaleFactor));
    this.camera.position.copy(target);
  }
};
