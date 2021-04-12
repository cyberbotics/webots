// Copyright 1996-2021 Cyberbotics Ltd.
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

import {direction, up, right, length, vec4ToQuaternion, quaternionToVec4, fromAxisAngle} from './nodes/wbUtils.js';
import {Selector} from './selector.js';
import {SystemInfo} from './system_info.js';
import {webots} from './webots.js';
import {WbVector3} from './nodes/utils/wbVector3.js';
import {WbWorld} from './nodes/wbWorld.js';
import {WbWrenPicker} from './wren/wbWrenPicker.js';

class MouseEvents {
  constructor(scene, domElement, mobileDevice) {
    this.scene = scene;
    this.domElement = domElement;
    this.mobileDevice = mobileDevice;

    this.state = {
      'initialized': false,
      'mouseDown': 0,
      'moved': false,
      'wheelFocus': false,
      'wheelTimeout': null
    };
    this.moveParams = {};
    this.enableNavigation = true;

    this.onmousemove = (event) => { this._onMouseMove(event); };
    this.onmouseup = (event) => { this._onMouseUp(event); };
    this.ontouchmove = (event) => { this._onTouchMove(event); };
    this.ontouchend = (event) => { this._onTouchEnd(event); };
    domElement.addEventListener('mousedown', (event) => { this._onMouseDown(event); }, false);
    domElement.addEventListener('mouseover', (event) => { this._onMouseOver(event); }, false);
    domElement.addEventListener('mouseleave', (event) => { this._onMouseLeave(event); }, false);
    domElement.addEventListener('wheel', (event) => { this._onMouseWheel(event); }, false);
    domElement.addEventListener('touchstart', (event) => { this._onTouchStart(event); }, true);
    domElement.addEventListener('contextmenu', (event) => { event.preventDefault(); }, false);

    // Prevent '#playerDiv' to raise the context menu of the browser.
    // This bug has been seen on Windows 10 / Firefox only.
    domElement.parentNode.addEventListener('contextmenu', (event) => { event.preventDefault(); }, false);
  }

  init() {
    if (typeof this.picker === 'undefined')
      this.picker = new WbWrenPicker();
  }

  _onMouseDown(event) {
    this.init();

    this.state.wheelFocus = true;
    this._initMouseMove(event);
    switch (event.button) {
      case MouseEvents.Click.RIGHT_CLICK:
        this.state.mouseDown |= 1;
        break;
      case MouseEvents.Click.LEFT_CLICK:
        this.state.mouseDown |= 4;
        break;
      case MouseEvents.Click.WHEEL_CLICK:
        this.state.mouseDown |= 2;
        break;
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this.state.mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this.state.mouseDown = 2;

    if (this.state.mouseDown !== 0) {
      this._setupMoveParameters(event);
      this.state.initialX = event.clientX;
      this.state.initialY = event.clientY;
      document.addEventListener('mousemove', this.onmousemove, false);
      document.addEventListener('mouseup', this.onmouseup, false);
    }

    if (typeof webots.currentView.onmousedown === 'function')
      webots.currentView.onmousedown(event);

    let pos = MouseEvents.convertMouseEventPositionToRelativePosition(canvas, this.state.x, this.state.y);
    this.picker.pick(pos.x, pos.y);
  }

  _onMouseMove(event) {
    if (!this.enableNavigation && event.button === 0) {
      if (typeof webots.currentView.onmousemove === 'function')
        webots.currentView.onmousemove(event);
      return;
    }

    if (typeof this.state.x === 'undefined')
      // mousedown event has not been called yet.
      // This could happen for example when another application has focus while loading the scene.
      return;
    if ('buttons' in event)
      this.state.mouseDown = event.buttons;
    else if ('which' in event) { // Safari only
      switch (event.which) {
        case 0: this.state.mouseDown = 0; break;
        case 1: this.state.mouseDown = 1; break;
        case 2: this.state.pressedButton = 4; break;
        case 3: this.state.pressedButton = 2; break;
        default: this.state.pressedButton = 0; break;
      }
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this.state.mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this.state.mouseDown = 2;

    if (this.state.mouseDown === 0)
      return;

    if (this.state.initialTimeStamp === null)
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;

    this.moveParams.dx = event.clientX - this.state.x;
    this.moveParams.dy = event.clientY - this.state.y;

    let orientation = WbWorld.instance.viewpoint.orientation;
    let position = WbWorld.instance.viewpoint.position;

    let rotationCenter = new WbVector3((this.picker.coordinates.x / canvas.width) * 2 - 1, (this.picker.coordinates.y / canvas.height) * 2 - 1, this.picker.coordinates.z);
    rotationCenter = WbWorld.instance.viewpoint.toWorld(rotationCenter);
    rotationCenter = glm.vec3(rotationCenter.x, rotationCenter.y, rotationCenter.z);

    if (this.state.mouseDown === 1) { // left mouse button to rotate viewpoint
      let halfPitchAngle = -0.005 * this.moveParams.dy;
      let halfYawAngle = -0.005 * this.moveParams.dx;
      if (this.picker.selectedId === -1) {
        halfPitchAngle /= -8;
        halfYawAngle /= -8;
      }
      let sinusYaw = Math.sin(halfYawAngle);
      let sinusPitch = Math.sin(halfPitchAngle);
      let pitch = right(orientation);
      let pitchRotation = glm.quat(Math.cos(halfPitchAngle), sinusPitch * pitch.x, sinusPitch * pitch.y, sinusPitch * pitch.z);
      let worldUpVector = WbWorld.instance.upVector;
      let yawRotation = glm.quat(Math.cos(halfYawAngle), sinusYaw * worldUpVector.x, sinusYaw * worldUpVector.y, sinusYaw * worldUpVector.z);

      // Updates camera's position and orientation
      let deltaRotation = yawRotation.mul(pitchRotation);
      let currentPosition = deltaRotation.mul(glm.vec3(position.x, position.y, position.z).sub(rotationCenter)).add(rotationCenter);
      let currentOrientation = deltaRotation.mul(vec4ToQuaternion(orientation));
      WbWorld.instance.viewpoint.position = new WbVector3(currentPosition.x, currentPosition.y, currentPosition.z);
      WbWorld.instance.viewpoint.orientation = quaternionToVec4(currentOrientation);
      WbWorld.instance.viewpoint.updatePosition();
      WbWorld.instance.viewpoint.updateOrientation();
      this.scene.render();
    } else {
      let distanceToPickPosition = 0.001;
      if (this.picker.selectedId !== -1)
        distanceToPickPosition = length(position.sub(rotationCenter));
      else
        distanceToPickPosition = length(position);

      if (distanceToPickPosition < 0.001)
        distanceToPickPosition = 0.001;

      let scaleFactor = distanceToPickPosition * 2 * Math.tan(WbWorld.instance.viewpoint.fieldOfView / 2) / Math.max(canvas.width, canvas.height);

      if (this.state.mouseDown === 2) { // right mouse button to translate viewpoint
        let targetRight = -scaleFactor * this.moveParams.dx;
        let targetUp = scaleFactor * this.moveParams.dy;
        let upVec = up(orientation);
        let rightVec = right(orientation);
        let targetR = rightVec.mul(targetRight);
        let targetU = upVec.mul(targetUp);
        let target = targetR.add(targetU);
        WbWorld.instance.viewpoint.position = position.add(target);
        WbWorld.instance.viewpoint.updatePosition();
        this.scene.render();
      } else if (this.state.mouseDown === 3 || this.state.mouseDown === 4) { // both left and right button or middle button to zoom
        let rollVector = direction(orientation);
        let zDisplacement = rollVector.mul(scaleFactor * 5 * this.moveParams.dy);
        let roll2 = fromAxisAngle(rollVector.x, rollVector.y, rollVector.z, 0.01 * this.moveParams.dx);
        let roll3 = glm.quat();
        roll3.w = roll2.w;
        roll3.x = roll2.x;
        roll3.y = roll2.y;
        roll3.z = roll2.z;

        WbWorld.instance.viewpoint.position = position.add(zDisplacement);
        WbWorld.instance.viewpoint.orientation = quaternionToVec4(roll3.mul(vec4ToQuaternion(orientation)));
        WbWorld.instance.viewpoint.updatePosition();
        WbWorld.instance.viewpoint.updateOrientation();

        this.scene.render();
      }
    }
    this.state.moved = event.clientX !== this.state.x || event.clientY !== this.state.y;
    this.state.x = event.clientX;
    this.state.y = event.clientY;

    if (typeof webots.currentView.onmousemove === 'function')
      webots.currentView.onmousemove(event);
    if (typeof webots.currentView.onmousedrag === 'function')
      webots.currentView.onmousedrag(event);
  }

  _onMouseUp(event) {
    this._clearMouseMove();
    this._selectAndHandleClick();

    document.removeEventListener('mousemove', this.onmousemove, false);
    document.removeEventListener('mouseup', this.onmouseup, false);

    if (typeof webots.currentView.onmouseup === 'function')
      webots.currentView.onmouseup(event);
  }

  _onMouseWheel(event) {
    this.init();

    event.preventDefault(); // do not scroll page
    if (!('initialCameraPosition' in this.moveParams))
      this._setupMoveParameters(event);
    // else another drag event is already active

    if (!this.enableNavigation || this.state.wheelFocus === false) {
      let offset = event.deltaY;
      if (event.deltaMode === 1)
        offset *= 40; // standard line height in pixel
      window.scroll(0, window.pageYOffset + offset);
      if (this.state.wheelTimeout) { // you have to rest at least 1.5 seconds over the x3d canvas
        clearTimeout(this.state.wheelTimeout); // so that the wheel focus will get enabled and
        this.state.wheelTimeout = setTimeout((event) => { this._wheelTimeoutCallback(event); }, 1500); // allow you to zoom in/out.
      }
      return;
    }

    let distanceToPickPosition;
    let position = WbWorld.instance.viewpoint.position;

    let rotationCenter = new WbVector3((this.picker.coordinates.x / canvas.width) * 2 - 1, (this.picker.coordinates.y / canvas.height) * 2 - 1, this.picker.coordinates.z);
    rotationCenter = WbWorld.instance.viewpoint.toWorld(rotationCenter);
    rotationCenter = glm.vec3(rotationCenter.x, rotationCenter.y, rotationCenter.z);

    if (this.picker.selectedId !== -1)
      distanceToPickPosition = length(position.sub(rotationCenter));
    else
      distanceToPickPosition = length(position);

    if (distanceToPickPosition < 0.001)
      distanceToPickPosition = 0.001;

    let direct = direction(WbWorld.instance.viewpoint.orientation);
    const scaleFactor = 0.02 * (event.deltaY < 0.0 ? -1 : 1) * distanceToPickPosition;
    const zDisplacement = direct.mul(scaleFactor);
    WbWorld.instance.viewpoint.position = position.add(zDisplacement);
    WbWorld.instance.viewpoint.updatePosition();

    this.scene.render();
  }

  _wheelTimeoutCallback(event) {
    this.state.wheelTimeout = null;
    this.state.wheelFocus = true;
  }

  _onMouseOver(event) {
    this.state.wheelTimeout = setTimeout((event) => { this._wheelTimeoutCallback(event); }, 1500);
  }

  _onMouseLeave(event) {
    if (this.state.wheelTimeout != null) {
      clearTimeout(this.state.wheelTimeout);
      this.state.wheelTimeout = null;
    }
    this.state.wheelFocus = false;

    if (typeof webots.currentView.onmouseleave === 'function')
      webots.currentView.onmouseleave(event);
  }

  _onTouchMove(event) {
    if (!this.enableNavigation || event.targetTouches.length === 0 || event.targetTouches.length > 2)
      return;
    if (this.state.initialTimeStamp === null)
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;
    if ((this.state.mouseDown !== 2) !== (event.targetTouches.length > 1))
      // Gesture single/multi touch changed after initialization.
      return;

    const touch = event.targetTouches['0'];
    const x = Math.round(touch.clientX); // discard decimal values returned on android
    const y = Math.round(touch.clientY);
    let orientation = WbWorld.instance.viewpoint.orientation;
    let position = WbWorld.instance.viewpoint.position;

    let rotationCenter = new WbVector3((this.picker.coordinates.x / canvas.width) * 2 - 1, (this.picker.coordinates.y / canvas.height) * 2 - 1, this.picker.coordinates.z);
    rotationCenter = WbWorld.instance.viewpoint.toWorld(rotationCenter);
    rotationCenter = glm.vec3(rotationCenter.x, rotationCenter.y, rotationCenter.z);
    let distanceToPickPosition = 0.001;
    if (this.picker.selectedId !== -1)
      distanceToPickPosition = length(position.sub(rotationCenter));
    else
      distanceToPickPosition = length(position);

    if (distanceToPickPosition < 0.001)
      distanceToPickPosition = 0.001;

    let scaleFactor = distanceToPickPosition * 2 * Math.tan(WbWorld.instance.viewpoint.fieldOfView / 2) / Math.max(canvas.width, canvas.height);

    if (this.state.mouseDown === 2) { // translation
      this.moveParams.dx = x - this.state.x;
      this.moveParams.dy = y - this.state.y;

      // On small phone screens (Android) this is needed to correctly detect clicks and longClicks.
      if (this.state.initialX == null && this.state.initialY == null) {
        this.state.initialX = Math.round(this.state.x);
        this.state.initialY = Math.round(this.state.y);
      }
      if (Math.abs(this.moveParams.dx) < 2 && Math.abs(this.moveParams.dy) < 2 &&
        Math.abs(this.state.initialX - x) < 5 && Math.abs(this.state.initialY - y) < 5)
        this.state.moved = false;
      else
        this.state.moved = true;

      this.moveParams.dx = x - this.state.initialX;
      this.moveParams.dy = y - this.state.initialY;

      let targetRight = -scaleFactor * this.moveParams.dx;
      let targetUp = scaleFactor * this.moveParams.dy;
      let upVec = up(orientation);
      let rightVec = right(orientation);
      let targetR = rightVec.mul(targetRight);
      let targetU = upVec.mul(targetUp);
      let target = targetR.add(targetU);
      WbWorld.instance.viewpoint.position = position.add(target);
      WbWorld.instance.viewpoint.updatePosition();
      this.scene.render();
    } else {
      const touch1 = event.targetTouches['1'];
      const x1 = Math.round(touch1.clientX);
      const y1 = Math.round(touch1.clientY);
      const distanceX = x - x1;
      const distanceY = y - y1;
      const newTouchDistance = distanceX * distanceX + distanceY * distanceY;
      const pinchSize = this.state.touchDistance - newTouchDistance;

      const moveX1 = x - this.state.x;
      const moveX2 = x1 - this.state.x1;
      const moveY1 = y - this.state.y;
      const moveY2 = y1 - this.state.y1;
      const ratio = 1;

      if (Math.abs(pinchSize) > 500 * ratio) { // zoom and tilt
        let d;
        if (Math.abs(moveX2) < Math.abs(moveX1))
          d = moveX1;
        else
          d = moveX2;
        this.moveParams.tiltAngle = 0.0004 * d;
        this.moveParams.zoomScale = this.moveParams.scaleFactor * 0.015 * pinchSize;
        let rollVector = direction(orientation);
        let zDisplacement = rollVector.mul(scaleFactor * 5 * this.moveParams.dy);
        let roll2 = fromAxisAngle(rollVector.x, rollVector.y, rollVector.z, 0.01 * this.moveParams.dx);
        let roll3 = glm.quat();
        roll3.w = roll2.w;
        roll3.x = roll2.x;
        roll3.y = roll2.y;
        roll3.z = roll2.z;

        WbWorld.instance.viewpoint.position = position.add(zDisplacement);
        WbWorld.instance.viewpoint.orientation = quaternionToVec4(roll3.mul(vec4ToQuaternion(orientation)));
        WbWorld.instance.viewpoint.updatePosition();
        WbWorld.instance.viewpoint.updateOrientation();

        this.scene.render();
      } else if (Math.abs(moveY2 - moveY1) < 3 * ratio && Math.abs(moveX2 - moveX1) < 3 * ratio) { // rotation (pitch and yaw)
        this.moveParams.dx = moveX1 * 0.8;
        this.moveParams.dy = moveY1 * 0.5;

        let halfPitchAngle = -0.005 * this.moveParams.dy;
        let halfYawAngle = -0.005 * this.moveParams.dx;
        if (this.picker.selectedId === -1) {
          halfPitchAngle /= -8;
          halfYawAngle /= -8;
        }
        let sinusYaw = Math.sin(halfYawAngle);
        let sinusPitch = Math.sin(halfPitchAngle);
        let pitch = right(orientation);
        let pitchRotation = glm.quat(Math.cos(halfPitchAngle), sinusPitch * pitch.x, sinusPitch * pitch.y, sinusPitch * pitch.z);
        let worldUpVector = WbWorld.instance.upVector;
        let yawRotation = glm.quat(Math.cos(halfYawAngle), sinusYaw * worldUpVector.x, sinusYaw * worldUpVector.y, sinusYaw * worldUpVector.z);

        // Updates camera's position and orientation
        let deltaRotation = yawRotation.mul(pitchRotation);
        let currentPosition = deltaRotation.mul(glm.vec3(position.x, position.y, position.z).sub(rotationCenter)).add(rotationCenter);
        let currentOrientation = deltaRotation.mul(vec4ToQuaternion(orientation));
        WbWorld.instance.viewpoint.position = new WbVector3(currentPosition.x, currentPosition.y, currentPosition.z);
        WbWorld.instance.viewpoint.orientation = quaternionToVec4(currentOrientation);
        WbWorld.instance.viewpoint.updatePosition();
        WbWorld.instance.viewpoint.updateOrientation();
        this.scene.render();
      }

      this.state.touchDistance = newTouchDistance;
      this.state.moved = true;
    }
    this.state.x = x;
    this.state.y = y;
    this.state.x1 = x1;
    this.state.y1 = y1;

    if (typeof webots.currentView.ontouchmove === 'function')
      webots.currentView.ontouchmove(event);
  }

  _onTouchStart(event) {
    this.init();
    this._initMouseMove(event.targetTouches['0']);
    if (event.targetTouches.length === 2) {
      const touch1 = event.targetTouches['1'];
      this.state.x1 = touch1.clientX;
      this.state.y1 = touch1.clientY;
      const distanceX = this.state.x - this.state.x1;
      const distanceY = this.state.y - this.state.y1;
      this.state.touchDistance = distanceX * distanceX + distanceY * distanceY;
      this.state.touchOrientation = Math.atan2(this.state.y1 - this.state.y, this.state.x1 - this.state.x);
      this.state.mouseDown = 3; // two fingers: rotation, tilt, zoom
    } else
      this.state.mouseDown = 2; // 1 finger: translation or single click

    this._setupMoveParameters(event.targetTouches['0']);
    this.domElement.addEventListener('touchend', this.ontouchend, true);
    this.domElement.addEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchstart === 'function')
      webots.currentView.ontouchstart(event);
  }

  _onTouchEnd(event) {
    this._clearMouseMove();
    this._selectAndHandleClick();

    this.domElement.removeEventListener('touchend', this.ontouchend, true);
    this.domElement.removeEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchend === 'function')
      webots.currentView.ontouchend(event);
  }

  _initMouseMove(event) {
    this.state.x = event.clientX;
    this.state.y = event.clientY;
    this.state.initialX = null;
    this.state.initialY = null;
    this.state.moved = false;
    this.state.initialTimeStamp = Date.now();
    this.state.longClick = false;
  }

  _setupMoveParameters(event) {
    this.moveParams = {};

    if (this.intersection && this.intersection.object)
      this.moveParams.pickPosition = this.intersection.point;
    else
      this.moveParams.pickPosition = null;

    if (this.intersection == null) {
      const cameraPosition = new glm.vec3();
      this.moveParams.distanceToPickPosition = cameraPosition.length;
    } else
      this.moveParams.distanceToPickPosition = this.intersection.distance;
    if (this.moveParams.distanceToPickPosition < 0.001) // 1 mm
      this.moveParams.distanceToPickPosition = 0.001;

    // Webots mFieldOfView corresponds to the horizontal FOV, i.e. viewpoint.fovX.
    const viewHeight = parseFloat($(this.scene.domElement).css('height').slice(0, -2));
    const viewWidth = parseFloat($(this.scene.domElement).css('width').slice(0, -2));
    this.moveParams.scaleFactor /= Math.max(viewHeight, viewWidth);
  }

  _clearMouseMove() {
    const timeDelay = this.mobileDevice ? 100 : 1000;
    this.state.longClick = Date.now() - this.state.initialTimeStamp >= timeDelay;
    if (this.state.moved === false)
      this.previousSelection = this.selection;
    else
      this.previousSelection = null;
    this.state.previousMouseDown = this.state.mouseDown;
    this.state.mouseDown = 0;
    this.state.initialTimeStamp = null;
    this.state.initialX = null;
    this.state.initialY = null;
    this.moveParams = {};
  }

  _selectAndHandleClick() {
    if (this.state.moved === false && (!this.state.longClick || this.mobileDevice)) {
      Selector.select(this.picker.selectedId);

      if (typeof WbWorld.instance.nodes.get(Selector.selectedId) !== 'undefined')
        WbWorld.instance.nodes.get(Selector.selectedId).updateBoundingObjectVisibility();

      if (typeof WbWorld.instance.nodes.get(Selector.previousId) !== 'undefined')
        WbWorld.instance.nodes.get(Selector.previousId).updateBoundingObjectVisibility();

      this.scene.render();
    }
  }
}

MouseEvents.convertMouseEventPositionToRelativePosition = (element, eventX, eventY) => {
  const rect = element.getBoundingClientRect();
  const pos = new glm.vec2();
  pos.x = Math.round(eventX - rect.left);
  pos.y = Math.round(eventY - rect.top);
  return pos;
};

MouseEvents.Click = {
  RIGHT_CLICK: 0,
  LEFT_CLICK: 1,
  WHEEL_CLICK: 2
};

export {MouseEvents};
