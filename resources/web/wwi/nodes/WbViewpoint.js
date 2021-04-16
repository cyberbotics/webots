import {M_PI_4, TAN_M_PI_8} from './WbConstants.js';
import {direction, up} from './utils/WbUtils.js';
import {GTAO_LEVEL, disableAntiAliasing} from './WbPreferences.js';
import WbBaseNode from './WbBaseNode.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbWorld from './WbWorld.js';
import WbWrenHdr from './../wren/WbWrenHdr.js';
import WbWrenGtao from './../wren/WbWrenGtao.js';
import WbWrenBloom from './../wren/WbWrenBloom.js';
import WbWrenSmaa from './../wren/WbWrenSmaa.js';

export default class WbViewpoint extends WbBaseNode {
  constructor(id, fieldOfView, orientation, position, exposure, bloomThreshold, near, far, followSmoothness, followedId, ambientOcclusionRadius) {
    super(id);
    this.orientation = this.initialOrientation = orientation;
    this.position = this.initialPosition = position;

    this.exposure = exposure;
    this.bloomThreshold = bloomThreshold;
    this.near = near;
    this.far = far;
    this.aspectRatio = canvas.width / canvas.height;
    this.fieldOfView = fieldOfView;
    this.fieldOfViewY = M_PI_4;
    this.tanHalfFieldOfViewY = TAN_M_PI_8;
    this.ambientOcclusionRadius = ambientOcclusionRadius;

    this.followSmoothness = followSmoothness;
    this.followedId = followedId;
    this.followedSolidPreviousPosition = new WbVector3();
    this.equilibriumVector = new WbVector3();
    this.velocity = new WbVector3();

    this.wrenHdr = new WbWrenHdr();
    this.wrenGtao = new WbWrenGtao();
    this.wrenBloom = new WbWrenBloom();
    this.wrenSmaa = new WbWrenSmaa();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.wrenViewport = _wr_scene_get_viewport(_wr_scene_get_instance());

    _wr_viewport_set_clear_color_rgb(this.wrenViewport, _wrjs_array3(0.0, 0.0, 0.0));
    this.wrenCamera = _wr_viewport_get_camera(this.wrenViewport);
    this._applyPositionToWren();
    this._applyOrientationToWren();
    this._applyNearToWren();
    this._applyFarToWren();
    this._applyFieldOfViewToWren();
    this.updatePostProcessingEffects();
    this.inverseViewMatrix = _wr_transform_get_matrix(this.wrenCamera);
  }

  delete() {
    if (typeof this.wrenSmaa !== 'undefined')
      this.wrenSmaa.delete();

    if (typeof this.wrenHdr !== 'undefined')
      this.wrenHdr.delete();

    if (typeof this.wrenGtao !== 'undefined')
      this.wrenGtao.delete();

    if (typeof this.wrenBloom !== 'undefined')
      this.wrenBloom.delete();
  }

  preFinalize() {
    super.preFinalize();

    this.updateFieldOfView();
    this.updateNear();
    this.updateFar();
  }

  postFinalize() {
    super.postFinalize();
    if (typeof WbWorld.instance.nodes.get(this.followedId) !== 'undefined' && typeof WbWorld.instance.nodes.get(this.followedId).translation !== 'undefined')
      this.followedSolidPreviousPosition = WbWorld.instance.nodes.get(this.followedId).translation;

    this.updatePostProcessingEffects();
  }

  resetViewpoint() {
    this.position = this.initialPosition;
    this.orientation = this.initialOrientation;
    this.updatePosition();
    this.updateOrientation();
  }

  // Converts screen coordinates to world coordinates
  toWorld(pos) {
    let zFar = this.far;
    if (zFar === 0)
      zFar = WbViewpoint.DEFAULT_FAR;

    const projection = new WbMatrix4();
    projection.set(1.0 / (this.aspectRatio * this.tanHalfFieldOfViewY), 0, 0, 0, 0, 1.0 / this.tanHalfFieldOfViewY, 0, 0, 0, 0, zFar / (this.near - zFar), -(zFar * this.near) / (zFar - this.near), 0, 0, -1, 0);
    const eye = new WbVector3(this.position.x, this.position.y, this.position.z);
    const center = eye.add(direction(this.orientation));
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
    _wr_camera_set_aspect_ratio(this.wrenCamera, this.aspectRatio);

    this._updateFieldOfViewY();

    this._applyFieldOfViewToWren();
  }

  updateExposure() {
    if (this.wrenObjectsCreatedCalled && this.wrenHdr)
      this.wrenHdr.setExposure(this.exposure);
  }

  updateFar() {
    if (this.far > 0.0 && this.far < this.near)
      this.far = this.near + 1.0;

    if (this.wrenObjectsCreatedCalled)
      this._applyFarToWren();
  }

  updateFieldOfView() {
    this._updateFieldOfViewY();

    if (this.wrenObjectsCreatedCalled)
      this._applyFieldOfViewToWren();
  }

  updateFollowUp(time) {
    if (typeof this.followedId === 'undefined' || typeof WbWorld.instance.nodes.get(this.followedId) === 'undefined')
      return;

    const followedSolid = WbWorld.instance.nodes.get(this.followedId);
    const followedSolidPosition = followedSolid.translation;
    const delta = followedSolidPosition.sub(this.followedSolidPreviousPosition);

    this.followedSolidPreviousPosition = followedSolidPosition;
    this.equilibriumVector = this.equilibriumVector.add(delta);
    const mass = ((this.followSmoothness < 0.05) ? 0.0 : ((this.followSmoothness > 1.0) ? 1.0 : this.followSmoothness));

    // If mass is 0, we instantly move the viewpoint to its equilibrium position.
    if (mass === 0.0) {
      this.position = this.position.add(this.equilibriumVector);
      this.velocity.setXyz(0.0, 0.0, 0.0);
      this.equilibriumVector.setXyz(0.0, 0.0, 0.0);
    } else { // Otherwise we apply a force and let physics do the rest.
      const timeStep = WbWorld.instance.basicTimeStep / 1000.0;
      const acceleration = this.equilibriumVector.div(mass);
      this.velocity = this.velocity.add(acceleration.mul(timeStep));

      const viewPointScalarVelocity = this.velocity.length();
      let followedObjectScalarVelocity;
      let followedObjectVelocity = new WbVector3();
      if (delta.length() > 0.0) {
        followedObjectVelocity = (delta.div(timeStep));
        followedObjectScalarVelocity = followedObjectVelocity.dot(this.velocity) / viewPointScalarVelocity;
      } else
        followedObjectScalarVelocity = 0.0;

      // If the viewpoint is going faster than the followed object, we slow it down to avoid oscillations
      if (viewPointScalarVelocity > followedObjectScalarVelocity) {
        const relativeSpeed = viewPointScalarVelocity - followedObjectScalarVelocity;
        if (relativeSpeed < 0.0001)
          this.velocity = this.velocity.mul(followedObjectScalarVelocity / viewPointScalarVelocity);
        else {
          let friction = 0.05 / mass;
          if (friction > 1.0)
            friction = 1.0;
          this.velocity = this.velocity.mul((viewPointScalarVelocity - relativeSpeed * friction) / viewPointScalarVelocity);
        }
      }

      const deltaPosition = this.velocity.mul(timeStep);

      this.position = this.position.add(deltaPosition);

      this.equilibriumVector = this.equilibriumVector.sub(deltaPosition);
    }

    this.updatePosition();
  }

  updateNear() {
    if (this.far > 0.0 && this.far < this.near)
      this.near = this.far;

    if (this.wrenObjectsCreatedCalled)
      this._applyNearToWren();
  }

  updatePosition() {
    if (this.wrenObjectsCreatedCalled)
      this._applyPositionToWren();
  }

  updatePostProcessingEffects() {
    if (!this.wrenObjectsCreatedCalled)
      return;

    if (this.wrenSmaa) {
      if (disableAntiAliasing)
        this.wrenSmaa.detachFromViewport();
      else
        this.wrenSmaa.setup(this.wrenViewport);
    }

    if (this.wrenHdr) {
      this.wrenHdr.setup(this.wrenViewport);
      this.updateExposure();
    }

    if (this.wrenGtao) {
      const qualityLevel = GTAO_LEVEL;
      if (qualityLevel === 0)
        this.wrenGtao.detachFromViewport();
      else {
        this.wrenGtao.setHalfResolution(qualityLevel <= 2);
        this.wrenGtao.setup(this.wrenViewport);
        this.updateNear();
        this.updateFar();
        this._updateFieldOfViewY();
      }
    }

    if (this.wrenBloom) {
      if (this.bloomThreshold === -1.0)
        this.wrenBloom.detachFromViewport();
      else
        this.wrenBloom.setup(this.wrenViewport);

      this.wrenBloom.setThreshold(this.bloomThreshold);
    }

    this.updateAspectRatio(canvas.width / canvas.height);
  }

  updatePostProcessingParameters() {
    if (!this.wrenObjectsCreatedCalled)
      return;

    if (this.wrenHdr)
      this.updateExposure();

    if (this.wrenGtao) {
      if (this.ambientOcclusionRadius === 0.0 || GTAO_LEVEL === 0) {
        this.wrenGtao.detachFromViewport();
        return;
      } else if (!this.wrenGtao.hasBeenSetup)
        this.wrenGtao.setup(this.wrenViewport);

      const qualityLevel = GTAO_LEVEL;
      this.updateNear();
      this.wrenGtao.setRadius(this.ambientOcclusionRadius);
      this.wrenGtao.setQualityLevel(qualityLevel);
      this.wrenGtao.applyOldInverseViewMatrixToWren();
      this.wrenGtao.copyNewInverseViewMatrix(this.inverseViewMatrix);
    }

    if (this.wrenBloom) {
      if (this.bloomThreshold === -1.0) {
        this.wrenBloom.detachFromViewport();
        return;
      } else if (!this.wrenBloom.hasBeenSetup)
        this.wrenBloom.setup(this.wrenViewport);

      this.wrenBloom.setThreshold(this.bloomThreshold);
    }
  }

  updateOrientation() {
    if (this.wrenObjectsCreatedCalled)
      this._applyOrientationToWren();
  }

  // Private functions

  _applyFarToWren() {
    if (this.far > 0.0)
      _wr_camera_set_far(this.wrenCamera, this.far);
    else
      _wr_camera_set_far(this.wrenCamera, WbViewpoint.DEFAULT_FAR);
  }

  _applyFieldOfViewToWren() {
    _wr_camera_set_fovy(this.wrenCamera, this.fieldOfViewY);
    if (this.wrenGtao)
      this.wrenGtao.setFov(this.fieldOfViewY);
  }

  _applyNearToWren() {
    _wr_camera_set_near(this.wrenCamera, this.near);
  }

  _applyOrientationToWren() {
    _wr_camera_set_orientation(this.wrenCamera, _wrjs_array4(this.orientation.w, this.orientation.x, this.orientation.y, this.orientation.z));
  }

  _applyPositionToWren() {
    _wr_camera_set_position(this.wrenCamera, _wrjs_array3(this.position.x, this.position.y, this.position.z));
  }

  _updateFieldOfViewY() {
    this.tanHalfFieldOfViewY = Math.tan(0.5 * this.fieldOfView);

    // According to VRML standards, the meaning of fieldOfView depends on the aspect ratio:
    // the view angle is taken with respect to the largest dimension
    if (this.aspectRatio < 1.0)
      this.fieldOfViewY = this.fieldOfView;
    else {
      this.tanHalfFieldOfViewY /= this.aspectRatio;
      this.fieldOfViewY = 2.0 * Math.atan(this.tanHalfFieldOfViewY);
    }
  }
}

WbViewpoint.DEFAULT_FAR = 1000000.0;
