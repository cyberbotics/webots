import {WbBaseNode} from "./WbBaseNode.js";
import {WbWrenHdr} from "./WbWrenHdr.js";
import {WbWrenGtao} from "./WbWrenGtao.js";
import {WbWrenBloom} from "./WbWrenBloom.js";
import {GTAO_LEVEL} from "./WbPreferences.js";


import {M_PI_4, TAN_M_PI_8} from "./WbConstants.js";


class WbViewpoint extends WbBaseNode {
  constructor(id, orientation, position, exposure, bloomThreshold, zNear, far, followsmoothness, ambientOcclusionRadius) {
    super(id);
    this.orientation = orientation;
    this.position = position;

    WbViewpoint.exposure = exposure;
    this.bloomThreshold = bloomThreshold;
    this.zNear = zNear;
    this.far = far;
    this.aspectRatio = 800/600; //TODO do not hardcode
    this.fieldOfView = M_PI_4;
    this.fieldOfViewY = M_PI_4;
    this.tanHalfFieldOfViewY = TAN_M_PI_8;
    this.followsmoothness = followsmoothness;
    this.ambientOcclusionRadius = ambientOcclusionRadius;

    this.inverseViewMatrix;

    this.wrenHdr = new WbWrenHdr();
    this.wrenGtao = new WbWrenGtao();
    this.wrenBloom = new WbWrenBloom();
    this.wrenViewport = undefined;
    this.wrenCamera = undefined;
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.wrenViewport = _wr_scene_get_viewport(_wr_scene_get_instance());
    //wr_viewport_set_visibility_mask(wrenViewport, WbWrenRenderingContext::instance()->optionalRenderingsMask());

    _wr_viewport_set_clear_color_rgb(this.wrenViewport, _wrjs_color_array(0.0, 0.0, 0.0));
    this.wrenCamera = _wr_viewport_get_camera(this.wrenViewport);
    this.applyPositionToWren();
    this.applyOrientationToWren();
    this.applyNearToWren();
    this.applyFarToWren();
    this.applyFieldOfViewToWren();
    //applyOrthographicViewHeightToWren();
    //updateLensFlare();
    this.updatePostProcessingEffects();
    this.inverseViewMatrix = _wr_transform_get_matrix(this.wrenCamera);


    // once the camera and viewport are created, update everything in the world instance
    //WbWorld::instance()->setViewpoint(this);
  }

  applyPositionToWren() {
    _wr_camera_set_position(this.wrenCamera, _wrjs_color_array(this.position.x, this.position.y, this.position.z)) ;
  }

  applyOrientationToWren() {
    _wr_camera_set_orientation(this.wrenCamera, _wrjs_array4(this.orientation.w, this.orientation.x, this.orientation.y, this.orientation.z));
  }

  applyNearToWren() {
    _wr_camera_set_near(this.wrenCamera, this.zNear);
  }

  applyFarToWren() {
    if (this.far > 0.0)
      _wr_camera_set_far(this.wrenCamera, this.far);
    else
      _wr_camera_set_far(this.wrenCamera, WbViewpoint.DEFAULT_FAR);
  }

  applyFieldOfViewToWren() {
    _wr_camera_set_fovy(this.wrenCamera, this.fieldOfViewY);
    if (this.wrenGtao)
      this.wrenGtao.setFov(this.fieldOfViewY);
  }

  updateNear() {
    //TODO
    //if (WbFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.05))
      //return;

    if (this.far > 0.0 && this.far < this.near) {
      this.near = this.far;
    }

    if (this.wrenObjectsCreatedCalled)
      this.applyNearToWren();
  }

  updateFar() {
    //TODO
    //if (WbFieldChecker::resetDoubleIfNegative(this, mFar, 0.0))
      //return;

    if (this.far > 0.0 && this.far < this.near) {
      this.far = this.near + 1.0;
      //TOCHECK it is strange to return here but not in update near
      //return;
    }

    if (this.wrenObjectsCreatedCalled)
      this.applyFarToWren();
  }

  updateFieldOfView() {
    //if (WbFieldChecker::resetDoubleIfNotInRangeWithExcludedBounds(this, mFieldOfView, 0.0, M_PI, M_PI_2))
      //return;

    this.updateFieldOfViewY();

    if (this.wrenObjectsCreatedCalled)
      this.applyFieldOfViewToWren();

    //emit cameraParametersChanged();
  }

  updateFieldOfViewY() {
    this.tanHalfFieldOfViewY = Math.tan(0.5 * this.fieldOfView);  // stored for reuse in viewpointRay()

    // According to VRML standards, the meaning of mFieldOfView depends on the aspect ratio:
    // the view angle is taken with respect to the largest dimension
    if (this.aspectRatio < 1.0)
      this.fieldOfViewY = this.fieldOfView;
    else {
      this.tanHalfFieldOfViewY /= this.aspectRatio;
      this.fieldOfViewY = 2.0 * Math.atan(this.tanHalfFieldOfViewY);
    }
  }

  updateAspectRatio(renderWindowAspectRatio) {
    if (!this.wrenObjectsCreatedCalled())
      return;

    this.aspectRatio = renderWindowAspectRatio;
    _wr_camera_set_aspect_ratio(this.wrenCamera, this.aspectRatio);

    this.updateFieldOfViewY();

    this.applyFieldOfViewToWren();

    //emit cameraParametersChanged();
  }

  updatePostProcessingEffects(){
    if(!this.wrenObjectsCreatedCalled)
      return;

    if (typeof this.lensFlare !== 'undefined')
     this.lensFlare.setup(this.wrenViewport);

    if (this.wrenSmaa) {
     /*if (WbPreferences::instance()->value("OpenGL/disableAntiAliasing", true).toBool())
       this.wrenSmaa.detachFromViewport();
     else
       this.wrenSmaa.setup(mWrenViewport);*/
    }

    if (this.wrenHdr) {
      this.wrenHdr.setup(this.wrenViewport);
      this.updateExposure();
    }

    if (this.wrenGtao) {
      let qualityLevel = GTAO_LEVEL;
      if (qualityLevel === 0)
        this.wrenGtao.detachFromViewport();
      else {
        this.wrenGtao.setHalfResolution(qualityLevel <= 2);
        this.wrenGtao.setup(this.wrenViewport);
        this.updateNear();
        this.updateFar();
        this.updateFieldOfViewY();
      }
    }

    if (this.wrenBloom) {
      if (this.bloomThreshold === -1.0)
        this.wrenBloom.detachFromViewport();
      else
        this.wrenBloom.setup(this.wrenViewport);

      this.wrenBloom.setThreshold(this.bloomThreshold);
    }

    //emit refreshRequired();
  }

  updatePostProcessingParameters(){
    if (!this.wrenObjectsCreatedCalled)
      return;

    if (this.wrenHdr)
      this.updateExposure();

    if (this.wrenGtao) {
      if (this.ambientOcclusionRadius == 0.0 || GTAO_LEVEL === 0){
        this.wrenGtao.detachFromViewport();
        return;
      } else if (!this.wrenGtao.hasBeenSetup)
        this.wrenGtao.setup(this.wrenViewport);

      let qualityLevel = GTAO_LEVEL;
      this.updateNear();
      this.wrenGtao.setRadius(this.ambientOcclusionRadius);
      this.wrenGtao.setQualityLevel(qualityLevel);
      this.wrenGtao.applyOldInverseViewMatrixToWren();
      this.wrenGtao.copyNewInverseViewMatrix(this.inverseViewMatrix);
    }

    if (this.wrenBloom) {
      if (this.bloomThreshold == -1.0) {
        this.wrenBloom.detachFromViewport();
        return;
      } else if (!this.wrenBloom.hasBeenSetup)
        this.wrenBloom.setup(this.wrenViewport);

      this.wrenBloom.setThreshold(this.bloomThreshold);
    }
  }

  updateExposure() {
    //TODO
    //if (WbFieldChecker::resetDoubleIfNegative(this, this.exposure, 1.0))
      //return;
    if (this.wrenObjectsCreatedCalled && this.wrenHdr)
      this.wrenHdr.setExposure(WbViewpoint.exposure);
  }

  updatePosition() {
    if (this.wrenObjectsCreatedCalled) {
      this.applyPositionToWren();
    }
  }

  updateOrientation() {
    if (this.wrenObjectsCreatedCalled) {
      this.applyOrientationToWren();
    }
  }

  preFinalize() {
    super.preFinalize();

    this.updateFieldOfView();
    this.updateNear();
    this.updateFar();

    if (typeof this.lensFlare !== 'undefined')
      this.lensFlare.preFinalize();
  }

  postFinalize() {
    super.postFinalize();
    this.updatePostProcessingEffects();
    if (typeof this.lensFlare !== 'undefined')
      this.lensFlare.postFinalize();

    //startFollowUpFromField();
  }

}

WbViewpoint.DEFAULT_FAR = 1000000.0;
//TODO: remove this global value once we have a scenetree;
WbViewpoint.exposure = 1;

export{WbViewpoint}
