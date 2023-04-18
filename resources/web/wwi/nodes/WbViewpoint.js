import {M_PI_4, TAN_M_PI_8} from './utils/constants.js';
import {direction, up} from './utils/utils.js';
import {GtaoLevel, disableAntiAliasing} from './wb_preferences.js';
import WbBaseNode from './WbBaseNode.js';
import WbMatrix3 from './utils/WbMatrix3.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbWorld from './WbWorld.js';
import WbWrenHdr from '../wren/WbWrenHdr.js';
import WbWrenGtao from '../wren/WbWrenGtao.js';
import WbWrenBloom from '../wren/WbWrenBloom.js';
import WbWrenSmaa from '../wren/WbWrenSmaa.js';
import {webots} from '../webots.js';

export default class WbViewpoint extends WbBaseNode {
  #defaultOrientation;
  #defaultPosition;
  #fieldOfViewY;
  #followedObjectDeltaPosition;
  #initialPosition;
  #inverseViewMatrix;
  #tanHalfFieldOfViewY;
  #viewpointForce;
  #viewpointLastUpdate;
  #viewpointVelocity;
  #wrenBloom;
  #wrenCamera;
  #wrenGtao;
  #wrenHdr;
  #wrenSmaa;
  #wrenViewport;
  constructor(id, fieldOfView, orientation, position, exposure, bloomThreshold, near, far, followSmoothness, followedId,
    ambientOcclusionRadius) {
    super(id);

    // the defaultOrientation and defaultPosition record the initial viewpoint and the modifications due to the follow
    // of an object to allow a smooth reset of the viewpoint.
    // the initialOrientation and initalPosition keep the value of the initial viewpoint.
    // it is used to reset the viewpoint, when an animation, with the "viewpoint follow" feature enabled, restarts.
    this.orientation = this.#defaultOrientation = orientation;
    this.position = this.#defaultPosition = this.#initialPosition = position;
    this.exposure = exposure;
    this.bloomThreshold = bloomThreshold;
    this.near = near;
    this.far = far;
    this.aspectRatio = canvas.width / canvas.height;
    this.fieldOfView = fieldOfView;
    this.#fieldOfViewY = M_PI_4;
    this.#tanHalfFieldOfViewY = TAN_M_PI_8;
    this.ambientOcclusionRadius = ambientOcclusionRadius;

    this.followSmoothness = followSmoothness;
    this.followedId = followedId;
    this.#viewpointForce = new WbVector3();
    this.#viewpointVelocity = new WbVector3();

    this.#wrenHdr = new WbWrenHdr();
    this.#wrenGtao = new WbWrenGtao();
    this.#wrenBloom = new WbWrenBloom();
    this.#wrenSmaa = new WbWrenSmaa();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.#wrenViewport = _wr_scene_get_viewport(_wr_scene_get_instance());

    _wr_viewport_set_clear_color_rgb(this.#wrenViewport, _wrjs_array3(0.0, 0.0, 0.0));
    this.#wrenCamera = _wr_viewport_get_camera(this.#wrenViewport);
    this.#applyPositionToWren();
    this.#applyOrientationToWren();
    this.#applyNearToWren();
    this.#applyFarToWren();
    this.#applyFieldOfViewToWren();
    this.updatePostProcessingEffects();
    this.#inverseViewMatrix = _wr_transform_get_matrix(this.#wrenCamera);
  }

  delete() {
    if (typeof this.#wrenSmaa !== 'undefined')
      this.#wrenSmaa.delete();

    if (typeof this.#wrenHdr !== 'undefined')
      this.#wrenHdr.delete();

    if (typeof this.#wrenGtao !== 'undefined')
      this.#wrenGtao.delete();

    if (typeof this.#wrenBloom !== 'undefined')
      this.#wrenBloom.delete();
  }

  preFinalize() {
    super.preFinalize();

    this.updateFieldOfView();
    this.updateNear();
    this.updateFar();
  }

  postFinalize() {
    super.postFinalize();

    this.updatePostProcessingEffects();
  }

  resetViewpoint() {
    this.position = this.#defaultPosition;
    this.orientation = this.#defaultOrientation;
    this.updatePosition();
    this.updateOrientation();
  }

  // Converts screen coordinates to world coordinates
  toWorld(pos) {
    let zFar = this.far;
    if (zFar === 0)
      zFar = WbViewpoint.DEFAULT_FAR;

    const projection = new WbMatrix4();
    projection.set(1.0 / (this.aspectRatio * this.#tanHalfFieldOfViewY), 0, 0, 0, 0, 1.0 / this.#tanHalfFieldOfViewY, 0, 0, 0,
      0, zFar / (this.near - zFar), -(zFar * this.near) / (zFar - this.near), 0, 0, -1, 0);
    const eye = new WbVector3(this.position.x, this.position.y, this.position.z);
    const center = eye.sub(direction(this.orientation));
    const upVec = up(this.orientation);

    const f = (center.sub(eye)).normalized();
    const s = f.cross(upVec).normalized();
    const u = s.cross(f);

    const view = new WbMatrix4();
    view.set(-s.x, -s.y, -s.z, s.dot(eye), u.x, u.y, u.z, -u.dot(eye), f.x, f.y, f.z, -f.dot(eye), 0, 0, 0, 1);

    const inverse = projection.mul(view);
    if (!inverse.inverse())
      return;

    let screenCoord = new WbVector4(pos.x, pos.y, pos.z, 1.0);
    screenCoord = inverse.mulByVec4(screenCoord);
    return screenCoord.div(screenCoord.w);
  }

  updateAspectRatio(renderWindowAspectRatio) {
    if (!this.wrenObjectsCreatedCalled)
      return;

    this.aspectRatio = renderWindowAspectRatio;
    _wr_camera_set_aspect_ratio(this.#wrenCamera, this.aspectRatio);

    this.#updateFieldOfViewY();

    this.#applyFieldOfViewToWren();
  }

  updateExposure() {
    if (this.wrenObjectsCreatedCalled && this.#wrenHdr)
      this.#wrenHdr.setExposure(this.exposure);
  }

  updateFar() {
    if (this.far > 0.0 && this.far < this.near)
      this.far = this.near + 1.0;

    if (this.wrenObjectsCreatedCalled)
      this.#applyFarToWren();
  }

  updateFieldOfView() {
    this.#updateFieldOfViewY();

    if (this.wrenObjectsCreatedCalled)
      this.#applyFieldOfViewToWren();
  }

  updateFollowUp(time, forcePosition) {
    if (typeof this.followedId === 'undefined' || typeof WbWorld.instance.nodes.get(this.followedId) === 'undefined')
      return;

    // reset the viewpoint position and the variables when the animation restarts
    if (time === 0) {
      this.#viewpointLastUpdate = time;
      this.position = this.position.sub(this.#defaultPosition.sub(this.#initialPosition));
      this.#defaultPosition = this.#initialPosition;
      this.#followedObjectDeltaPosition = new WbVector3();
      this.#viewpointForce = new WbVector3();
      this.#viewpointVelocity = new WbVector3();
    }

    if (typeof this.#viewpointLastUpdate === 'undefined')
      this.#viewpointLastUpdate = time;

    const timeInterval = Math.abs(time - this.#viewpointLastUpdate) / 1000;
    const mass = ((this.followSmoothness < 0.05) ? 0.0 : ((this.followSmoothness > 1.0) ? 1.0 : this.followSmoothness));

    if (timeInterval > 0) {
      this.#viewpointLastUpdate = time;
      let deltaPosition;

      if (typeof this.#followedObjectDeltaPosition !== 'undefined')
        this.#viewpointForce = this.#viewpointForce.add(this.#followedObjectDeltaPosition);

      if (forcePosition || mass === 0 || (timeInterval > 0.1 && typeof webots.animation === 'undefined')) {
        deltaPosition = new WbVector3(this.#viewpointForce.x, this.#viewpointForce.y, this.#viewpointForce.z);
        this.#viewpointVelocity = new WbVector3();
      } else {
        let acceleration = new WbVector3(this.#viewpointForce.x, this.#viewpointForce.y, this.#viewpointForce.z);
        acceleration = acceleration.mul(timeInterval / mass);
        this.#viewpointVelocity = this.#viewpointVelocity.add(acceleration);
        const scalarVelocity = this.#viewpointVelocity.length();

        let scalarObjectVelocityProjection;
        if (typeof this.#followedObjectDeltaPosition !== 'undefined') {
          let objectVelocity = new WbVector3(this.#followedObjectDeltaPosition.x, this.#followedObjectDeltaPosition.y,
            this.#followedObjectDeltaPosition.z);
          objectVelocity = objectVelocity.div(timeInterval);
          scalarObjectVelocityProjection = objectVelocity.dot(this.#viewpointVelocity) / scalarVelocity;
        } else
          scalarObjectVelocityProjection = 0;

        let viewpointFriction = 0.05 / mass;
        if (viewpointFriction > 0 && scalarVelocity > scalarObjectVelocityProjection) {
          const velocityFactor = (scalarVelocity - (scalarVelocity - scalarObjectVelocityProjection) * viewpointFriction) /
            scalarVelocity;
          this.#viewpointVelocity = this.#viewpointVelocity.mul(velocityFactor);
        }

        deltaPosition = this.#viewpointVelocity.mul(timeInterval);
      }
      this.#viewpointForce = this.#viewpointForce.sub(deltaPosition);
      this.position = this.position.add(deltaPosition);
      this.#defaultPosition = this.#defaultPosition.add(deltaPosition);
      this.#followedObjectDeltaPosition = undefined;
      this.updatePosition();
    }
  }

  setFollowedObjectDeltaPosition(newPosition, previousPosition) {
    this.#followedObjectDeltaPosition = newPosition.sub(previousPosition);
  }

  updateNear() {
    if (this.far > 0.0 && this.far < this.near)
      this.near = this.far;

    if (this.wrenObjectsCreatedCalled)
      this.#applyNearToWren();
  }

  updatePosition() {
    if (this.wrenObjectsCreatedCalled)
      this.#applyPositionToWren();

    WbWorld.instance.billboards.forEach(id => {
      let billboard = WbWorld.instance.nodes.get(id);
      if (typeof billboard !== 'undefined')
        billboard.updatePosition();
    });
  }

  updatePostProcessingEffects() {
    if (!this.wrenObjectsCreatedCalled)
      return;

    if (this.#wrenSmaa) {
      if (disableAntiAliasing)
        this.#wrenSmaa.detachFromViewport();
      else
        this.#wrenSmaa.setup(this.#wrenViewport);
    }

    if (this.#wrenHdr) {
      this.#wrenHdr.setup(this.#wrenViewport);
      this.updateExposure();
    }

    if (this.#wrenGtao) {
      const qualityLevel = GtaoLevel;
      if (qualityLevel === 0)
        this.#wrenGtao.detachFromViewport();
      else {
        this.#wrenGtao.setHalfResolution(qualityLevel <= 2);
        this.#wrenGtao.setup(this.#wrenViewport);
        this.updateNear();
        this.updateFar();
        this.#updateFieldOfViewY();
      }
    }

    if (this.#wrenBloom) {
      if (this.bloomThreshold === -1.0)
        this.#wrenBloom.detachFromViewport();
      else
        this.#wrenBloom.setup(this.#wrenViewport);

      this.#wrenBloom.setThreshold(this.bloomThreshold);
    }

    this.updateAspectRatio(canvas.width / canvas.height);
  }

  updatePostProcessingParameters() {
    if (!this.wrenObjectsCreatedCalled)
      return;

    if (this.#wrenHdr)
      this.updateExposure();

    if (this.#wrenGtao) {
      if (this.ambientOcclusionRadius === 0.0 || GtaoLevel === 0) {
        this.#wrenGtao.detachFromViewport();
        return;
      } else if (!this.#wrenGtao.hasBeenSetup)
        this.#wrenGtao.setup(this.#wrenViewport);

      const qualityLevel = GtaoLevel;
      this.updateNear();
      this.#wrenGtao.setRadius(this.ambientOcclusionRadius);
      this.#wrenGtao.setQualityLevel(qualityLevel);
      this.#wrenGtao.applyOldInverseViewMatrixToWren();
      this.#wrenGtao.copyNewInverseViewMatrix(this.#inverseViewMatrix);
    }

    if (this.#wrenBloom) {
      if (this.bloomThreshold === -1.0) {
        this.#wrenBloom.detachFromViewport();
        return;
      } else if (!this.#wrenBloom.hasBeenSetup)
        this.#wrenBloom.setup(this.#wrenViewport);

      this.#wrenBloom.setThreshold(this.bloomThreshold);
    }
  }

  updateOrientation() {
    if (this.wrenObjectsCreatedCalled)
      this.#applyOrientationToWren();
  }

  // Private functions

  #applyFarToWren() {
    if (this.far > 0.0)
      _wr_camera_set_far(this.#wrenCamera, this.far);
    else
      _wr_camera_set_far(this.#wrenCamera, WbViewpoint.DEFAULT_FAR);
  }

  #applyFieldOfViewToWren() {
    _wr_camera_set_fovy(this.#wrenCamera, this.#fieldOfViewY);
    if (this.#wrenGtao)
      this.#wrenGtao.setFov(this.#fieldOfViewY);
  }

  #applyNearToWren() {
    _wr_camera_set_near(this.#wrenCamera, this.near);
  }

  #applyOrientationToWren() {
    const fluRotation = this.orientation.toMatrix3().mulByMat3(WbMatrix3.fromEulerAngles(Math.PI / 2, -Math.PI / 2, 0))
      .toRotation();
    _wr_camera_set_orientation(this.#wrenCamera, _wrjs_array4(fluRotation.w, fluRotation.x, fluRotation.y, fluRotation.z));
  }

  #applyPositionToWren() {
    _wr_camera_set_position(this.#wrenCamera, _wrjs_array3(this.position.x, this.position.y, this.position.z));
  }

  #updateFieldOfViewY() {
    this.#tanHalfFieldOfViewY = Math.tan(0.5 * this.fieldOfView);

    // According to VRML standards, the meaning of fieldOfView depends on the aspect ratio:
    // the view angle is taken with respect to the largest dimension
    if (this.aspectRatio < 1.0)
      this.#fieldOfViewY = this.fieldOfView;
    else {
      this.#tanHalfFieldOfViewY /= this.aspectRatio;
      this.#fieldOfViewY = 2.0 * Math.atan(this.#tanHalfFieldOfViewY);
    }
  }
}

WbViewpoint.DEFAULT_FAR = 1000000.0;
