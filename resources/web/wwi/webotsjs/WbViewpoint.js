// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import {WbBaseNode} from "./WbBaseNode.js";
import {WbWrenHdr} from "./WbWrenHdr.js";
import {WbWrenGtao} from "./WbWrenGtao.js";
import {WbWrenBloom} from "./WbWrenBloom.js";
import {WbWrenSmaa} from "./WbWrenSmaa.js";
import {GTAO_LEVEL, disableAntiAliasing} from "./WbPreferences.js";
import {WbVector3} from "./utils/WbVector3.js"
import {WbMatrix4} from "./utils/WbMatrix4.js"
import {direction, up} from "./WbUtils.js"
import {World} from "./World.js"


import {M_PI_4, TAN_M_PI_8} from "./WbConstants.js";


class WbViewpoint extends WbBaseNode {
  constructor(id, orientation, position, exposure, bloomThreshold, zNear, far, followSmoothness, followedId, ambientOcclusionRadius) {
    super(id);
    this.orientation = orientation;
    this.position = position;

    this.exposure = exposure;
    this.bloomThreshold = bloomThreshold;
    this.zNear = zNear;
    this.far = far;
    this.aspectRatio = 800/600; //TODO do not hardcode
    this.fieldOfView = M_PI_4;
    this.fieldOfViewY = M_PI_4;
    this.tanHalfFieldOfViewY = TAN_M_PI_8;
    this.ambientOcclusionRadius = ambientOcclusionRadius;

    this.followSmoothness = followSmoothness;
    this.followedId = followedId;
    this.followedSolidPreviousPosition = new WbVector3();
    this.equilibriumVector = new WbVector3();
    this.velocity = new WbVector3();

    this.inverseViewMatrix;

    this.wrenHdr = new WbWrenHdr();
    this.wrenGtao = new WbWrenGtao();
    this.wrenBloom = new WbWrenBloom();
    this.wrenSmaa = new WbWrenSmaa();

    this.wrenViewport = undefined;
    this.wrenCamera = undefined;
  }

  delete () {
    if (typeof this.wrenSmaa !== 'undefined')
     this.wrenSmaa.delete();

    if (typeof this.wrenHdr !== 'undefined')
      this.wrenHdr.delete();

    if (typeof this.wrenGtao !== 'undefined')
      this.wrenGtao.delete();

    if (typeof this.wrenBloom !== 'undefined')
      this.wrenBloom.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.wrenViewport = _wr_scene_get_viewport(_wr_scene_get_instance());

    _wr_viewport_set_clear_color_rgb(this.wrenViewport, _wrjs_color_array(0.0, 0.0, 0.0));
    this.wrenCamera = _wr_viewport_get_camera(this.wrenViewport);
    this.applyPositionToWren();
    this.applyOrientationToWren();
    this.applyNearToWren();
    this.applyFarToWren();
    this.applyFieldOfViewToWren();
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
  }

  updatePostProcessingEffects(){
    if(!this.wrenObjectsCreatedCalled)
      return;

    if (typeof this.lensFlare !== 'undefined')
     this.lensFlare.setup(this.wrenViewport);

    if (this.wrenSmaa) {
     if (disableAntiAliasing)
       this.wrenSmaa.detachFromViewport();
     else{
       this.wrenSmaa.setup(this.wrenViewport);
     }
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
      this.wrenHdr.setExposure(this.exposure);
  }

  updatePosition() {
    if (this.wrenObjectsCreatedCalled)
      this.applyPositionToWren();
  }

  updateOrientation() {
    if (this.wrenObjectsCreatedCalled)
      this.applyOrientationToWren();
  }

  updateFollowUp() {
    if (typeof this.followedId === 'undefined' || typeof World.instance.nodes.get(this.followedId) === 'undefined')
      return;

    let followedSolid = World.instance.nodes.get(this.followedId)
    let followedSolidPosition = followedSolid.translation;
    let delta = followedSolidPosition.sub(this.followedSolidPreviousPosition);
    this.followedSolidPreviousPosition = followedSolidPosition;

    this.equilibriumVector = this.equilibriumVector.add(delta);
    const mass = ((this.followSmoothness < 0.05) ? 0.0 : ((this.followSmoothness > 1.0)  ? 1.0 : this.followSmoothness));

    // If mass is 0, we instantly move the viewpoint to its equilibrium position.
    if (mass === 0.0) {
      this.position = this.position.add(this.equilibriumVector);
      this.velocity.setXyz(0.0, 0.0, 0.0);
      this.equilibriumVector.setXyz(0.0, 0.0, 0.0);
    } else {  // Otherwise we apply a force and let physics do the rest.
      const timeStep = 8 / 1000.0; //TODO get the real timeStep
      const acceleration = this.equilibriumVector.div(mass);
      this.velocity = this.velocity.add(acceleration.mul(timeStep));

      const viewPointScalarVelocity = this.velocity.length();
      let followedObjectScalarVelocity;
      let followedObjectVelocity = new WbVector3;
      if (delta.length() > 0.0) {
        followedObjectVelocity = (delta.div(timeStep));
        followedObjectScalarVelocity = followedObjectVelocity.dot(this.velocity) / viewPointScalarVelocity;
      } else {
        followedObjectVelocity.setXyz(0.0, 0.0, 0.0);
        followedObjectScalarVelocity = 0.0;
      }

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

      const deltaPosition = (this.velocity.mul(timeStep));
      this.position = this.position.add(deltaPosition);
      // Moves the rotation point if a drag rotating the viewpoint is active

      this.equilibriumVector = this.equilibriumVector.sub(deltaPosition);
    }
      this.updatePosition();
  }

  // Converts screen coordinates to world coordinates
  toWorld(pos) {
  let worldCoord;

  if (this.far == 0)
    this.far = WbViewpoint.DEFAULT_FAR;

  let projection = new WbMatrix4();
  projection.set(1.0 / (this.aspectRatio * this.tanHalfFieldOfViewY), 0, 0, 0, 0, 1.0 / this.tanHalfFieldOfViewY, 0, 0, 0, 0, this.far / (this.zNear - this.far), -(this.far * this.zNear) / (this.far - this.zNear), 0, 0, -1, 0);

  let eye = new WbVector3(this.position)
  let center = eye.add(direction(this.orientation))
  let up = up(this.orientation);

  let f = (center.sub(eye)).normalized();
  let s = f.cross(up).normalized();
  let u = s.cross(f);

  let view = WbMatrix4();
  view.set(-s.x, -s.y -s.z, s.dot(eye), u.x u.y, u.z, -u.dot(eye), f.x, f.y, f.z, -f.dot(eye), 0, 0, 0, 1);

  let inverse = projection.mul(view);
  //TODO
  if (!inverse.inverse())
    return;

  WbVector4 screen(pos.x(), pos.y(), pos.z(), 1.0);
  screen = inverse * screen;
  screen /= screen.w();
  P.setXyz(screen.ptr());
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
  }

}

WbViewpoint.DEFAULT_FAR = 1000000.0;

export{WbViewpoint}
