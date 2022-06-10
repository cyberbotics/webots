import {direction, up, right, length, vec4ToQuaternion, quaternionToVec4, fromAxisAngle} from './nodes/utils/utils.js';
import Selector from './Selector.js';
import SystemInfo from './system_info.js';
import {webots} from './webots.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbWorld from './nodes/WbWorld.js';
import WbWrenPicker from './wren/WbWrenPicker.js';

export default class MouseEvents {
  constructor(scene, domElement, mobileDevice) {
    this._scene = scene;
    this._domElement = domElement;
    this._mobileDevice = mobileDevice;

    this._state = {
      'initialized': false,
      'mouseDown': 0,
      'moved': false,
      'wheelFocus': false,
      'wheelTimeout': null
    };
    this._moveParams = {};
    this._enableNavigation = true;

    this.onmousemove = (event) => { this._onMouseMove(event); };
    this.onmouseup = (event) => { this._onMouseUp(event); };
    this.ontouchmove = (event) => { this._onTouchMove(event); };
    this.ontouchend = (event) => { this._onTouchEnd(event); };
    this._domElement.addEventListener('mousedown', (event) => { this._onMouseDown(event); }, false);
    this._domElement.addEventListener('mouseover', (event) => { this._onMouseOver(event); }, false);
    this._domElement.addEventListener('mouseleave', (event) => { this.onMouseLeave(event); }, false);
    this._domElement.addEventListener('wheel', (event) => { this._onMouseWheel(event); }, false);
    this._domElement.addEventListener('touchstart', (event) => { this._onTouchStart(event); }, true);
    this._domElement.addEventListener('contextmenu', (event) => { event.preventDefault(); }, false);
    this._domElement.addEventListener('mousemove', () => this._detectImmobility());
    // Prevent '#playerDiv' to raise the context menu of the browser.
    // This bug has been seen on Windows 10 / Firefox only.
    this._domElement.parentNode.addEventListener('contextmenu', (event) => { event.preventDefault(); }, false);
  }

  init() {
    if (typeof this.picker === 'undefined')
      this.picker = new WbWrenPicker();
  }

  _onMouseDown(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    this.init();

    this._state.wheelFocus = true;
    this._initMouseMove(event);
    switch (event.button) {
      case MouseEvents.Click.RIGHT_CLICK:
        this._state.mouseDown |= 1;
        break;
      case MouseEvents.Click.LEFT_CLICK:
        this._state.mouseDown |= 4;
        break;
      case MouseEvents.Click.WHEEL_CLICK:
        this._state.mouseDown |= 2;
        break;
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this._state.mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this._state.mouseDown = 2;

    if (this._state.mouseDown !== 0) {
      this._state.initialX = event.clientX;
      this._state.initialY = event.clientY;
      document.addEventListener('mousemove', this.onmousemove, false);
      document.addEventListener('mouseup', this.onmouseup, false);
    }

    if (typeof webots.currentView.onmousedown === 'function')
      webots.currentView.onmousedown(event);

    const position = MouseEvents.convertMouseEventPositionToRelativePosition(canvas, this._state.x, this._state.y);
    this.picker.pick(position.x, position.y);
    if (this.picker.selectedId !== -1) {
      this._rotationCenter = new WbVector3((this.picker.coordinates.x / canvas.width) * 2 - 1,
        (this.picker.coordinates.y / canvas.height) * 2 - 1, this.picker.coordinates.z);
      this._rotationCenter = WbWorld.instance.viewpoint.toWorld(this._rotationCenter);
      this._rotationCenter = glm.vec3(this._rotationCenter.x, this._rotationCenter.y, this._rotationCenter.z);
    } else
      this._rotationCenter = glm.vec3(WbWorld.instance.viewpoint.position.x, WbWorld.instance.viewpoint.position.y,
        WbWorld.instance.viewpoint.position.z);
  }

  _onMouseMove(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    if (!this._enableNavigation && event.button === 0) {
      if (typeof webots.currentView.onmousemove === 'function')
        webots.currentView.onmousemove(event);
      return;
    }

    if (typeof this._state.x === 'undefined')
      // mousedown event has not been called yet.
      // This could happen for example when another application has focus while loading the scene.
      return;
    if ('buttons' in event)
      this._state.mouseDown = event.buttons;
    else if ('which' in event) { // Safari only
      switch (event.which) {
        case 0: this._state.mouseDown = 0; break;
        case 1: this._state.mouseDown = 1; break;
        case 2: this._state.pressedButton = 4; break;
        case 3: this._state.pressedButton = 2; break;
        default: this._state.pressedButton = 0; break;
      }
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this._state.mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this._state.mouseDown = 2;

    if (this._state.mouseDown === 0)
      return;

    if (this._state.initialTimeStamp === null)
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;

    this._moveParams.dx = event.clientX - this._state.x;
    this._moveParams.dy = event.clientY - this._state.y;

    const orientation = WbWorld.instance.viewpoint.orientation;
    const position = WbWorld.instance.viewpoint.position;

    if (this._state.mouseDown === 1) { // left mouse button to rotate viewpoint
      let halfPitchAngle = 0.005 * this._moveParams.dy;
      let halfYawAngle = -0.005 * this._moveParams.dx;
      if (this.picker.selectedId === -1) {
        halfPitchAngle /= -8;
        halfYawAngle /= -8;
      }
      const sinusYaw = Math.sin(halfYawAngle);
      const sinusPitch = Math.sin(halfPitchAngle);
      const pitch = right(orientation);
      const pitchRotation = glm.quat(Math.cos(halfPitchAngle), sinusPitch * pitch.x, sinusPitch * pitch.y,
        sinusPitch * pitch.z);
      const worldUpVector = WbWorld.instance.upVector;
      const yawRotation = glm.quat(Math.cos(halfYawAngle), sinusYaw * worldUpVector.x, sinusYaw * worldUpVector.y,
        sinusYaw * worldUpVector.z);
      // Updates camera's position and orientation
      const deltaRotation = yawRotation.mul(pitchRotation);
      const currentPosition = deltaRotation.mul(glm.vec3(position.x, position.y, position.z).sub(this._rotationCenter))
        .add(this._rotationCenter);
      const currentOrientation = deltaRotation.mul(vec4ToQuaternion(orientation));
      WbWorld.instance.viewpoint.position = new WbVector3(currentPosition.x, currentPosition.y, currentPosition.z);
      WbWorld.instance.viewpoint.orientation = quaternionToVec4(currentOrientation);
      WbWorld.instance.viewpoint.updatePosition();
      WbWorld.instance.viewpoint.updateOrientation();
      this._scene.render();
    } else {
      let distanceToPickPosition = 0.001;
      if (this.picker.selectedId !== -1)
        distanceToPickPosition = length(position.sub(this._rotationCenter));
      else
        distanceToPickPosition = length(position);

      if (distanceToPickPosition < 0.001)
        distanceToPickPosition = 0.001;

      const scaleFactor = distanceToPickPosition * 2 * Math.tan(WbWorld.instance.viewpoint.fieldOfView / 2) /
        Math.max(canvas.width, canvas.height);

      if (this._state.mouseDown === 2) { // right mouse button to translate viewpoint
        const targetRight = scaleFactor * this._moveParams.dx;
        const targetUp = scaleFactor * this._moveParams.dy;
        const upVec = up(orientation);
        const rightVec = right(orientation);
        const targetR = rightVec.mul(targetRight);
        const targetU = upVec.mul(targetUp);
        const target = targetR.add(targetU);
        WbWorld.instance.viewpoint.position = position.add(target);
        WbWorld.instance.viewpoint.updatePosition();
        this._scene.render();
      } else if (this._state.mouseDown === 3 || this._state.mouseDown === 4) {
        // both left and right button or middle button to zoom
        const rollVector = direction(orientation);
        const zDisplacement = rollVector.mul(scaleFactor * -5 * this._moveParams.dy);
        const roll2 = fromAxisAngle(rollVector.x, rollVector.y, rollVector.z, 0.01 * this._moveParams.dx);
        const roll3 = glm.quat();
        roll3.w = roll2.w;
        roll3.x = roll2.x;
        roll3.y = roll2.y;
        roll3.z = roll2.z;

        WbWorld.instance.viewpoint.position = position.add(zDisplacement);
        WbWorld.instance.viewpoint.orientation = quaternionToVec4(roll3.mul(vec4ToQuaternion(orientation)));
        WbWorld.instance.viewpoint.updatePosition();
        WbWorld.instance.viewpoint.updateOrientation();

        this._scene.render();
      }
    }
    this._state.moved = event.clientX !== this._state.x || event.clientY !== this._state.y;
    this._state.x = event.clientX;
    this._state.y = event.clientY;

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
    if (typeof WbWorld.instance === 'undefined')
      return;

    this.init();
    this._detectImmobility();

    event.preventDefault(); // do not scroll page

    if (!this._enableNavigation || this._state.wheelFocus === false) {
      let offset = event.deltaY;
      if (event.deltaMode === 1)
        offset *= 40; // standard line height in pixel
      window.scroll(0, window.pageYOffset + offset);
      if (this._state.wheelTimeout) { // you have to rest at least 1.5 seconds over the x3d canvas
        clearTimeout(this._state.wheelTimeout); // so that the wheel focus will get enabled and
        // allow you to zoom in/out.
        this._state.wheelTimeout = setTimeout((event) => { this._wheelTimeoutCallback(event); }, 1500);
      }
      return;
    }

    let distanceToPickPosition;
    const position = WbWorld.instance.viewpoint.position;
    if (this.picker.selectedId !== -1)
      distanceToPickPosition = length(position.sub(this._rotationCenter));
    else
      distanceToPickPosition = length(position);

    if (distanceToPickPosition < 0.001)
      distanceToPickPosition = 0.001;

    const direct = direction(WbWorld.instance.viewpoint.orientation);
    const scaleFactor = 0.1 * (event.deltaY < 0.0 ? 1 : -1) * distanceToPickPosition;
    const zDisplacement = direct.mul(scaleFactor);

    WbWorld.instance.viewpoint.position = position.add(zDisplacement);
    WbWorld.instance.viewpoint.updatePosition();

    this._scene.render();
  }

  _wheelTimeoutCallback(event) {
    this._state.wheelTimeout = null;
    this._state.wheelFocus = true;
  }

  _onMouseOver(event) {
    this._state.wheelTimeout = setTimeout((event) => { this._wheelTimeoutCallback(event); }, 1500);

    if (typeof this.showToolbar !== 'undefined')
      this.showToolbar();
  }

  onMouseLeave(event) {
    clearTimeout(this._moveTimeout);

    if (typeof event !== 'undefined' && event.relatedTarget != null &&
      event.relatedTarget.id === 'time-slider')
      return;

    if (this._state.wheelTimeout != null) {
      clearTimeout(this._state.wheelTimeout);
      this._state.wheelTimeout = null;
    }

    this._state.wheelFocus = false;

    if (typeof webots.currentView.onmouseleave === 'function')
      webots.currentView.onmouseleave(event);

    if (typeof this.hideToolbar !== 'undefined')
      this.hideToolbar();
  }

  _onTouchMove(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    if (!this._enableNavigation || event.targetTouches.length === 0 || event.targetTouches.length > 2)
      return;
    if (this._state.initialTimeStamp === null)
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;
    if ((this._state.mouseDown !== 2) !== (event.targetTouches.length > 1))
      // Gesture single/multi touch changed after initialization.
      return;

    const touch = event.targetTouches['0'];
    const x = Math.round(touch.clientX); // discard decimal values returned on android
    const y = Math.round(touch.clientY);

    this._moveParams.dx = x - this._state.x;
    this._moveParams.dy = y - this._state.y;

    const orientation = WbWorld.instance.viewpoint.orientation;
    const position = WbWorld.instance.viewpoint.position;

    let distanceToPickPosition = 0.001;
    if (this.picker.selectedId !== -1)
      distanceToPickPosition = length(position.sub(this._rotationCenter));
    else
      distanceToPickPosition = length(position);

    if (distanceToPickPosition < 0.001)
      distanceToPickPosition = 0.001;

    const scaleFactor = distanceToPickPosition * 2 * Math.tan(WbWorld.instance.viewpoint.fieldOfView / 2) /
      Math.max(canvas.width, canvas.height);

    if (this._state.mouseDown === 2) { // translation
      const targetRight = scaleFactor * this._moveParams.dx;
      const targetUp = scaleFactor * this._moveParams.dy;
      const upVec = up(orientation);
      const rightVec = right(orientation);
      const targetR = rightVec.mul(targetRight);
      const targetU = upVec.mul(targetUp);
      const target = targetR.add(targetU);
      WbWorld.instance.viewpoint.position = position.add(target);
      WbWorld.instance.viewpoint.updatePosition();
      this._scene.render();
    } else {
      const touch1 = event.targetTouches['1'];
      const x1 = Math.round(touch1.clientX);
      const y1 = Math.round(touch1.clientY);
      const distanceX = x - x1;
      const distanceY = y - y1;
      const newTouchDistance = distanceX * distanceX + distanceY * distanceY;
      const pinchSize = this._state.touchDistance - newTouchDistance;
      const ratio = 1;

      if (Math.abs(pinchSize) > 500 * ratio) { // zoom and tilt
        const rollVector = direction(orientation);
        const zDisplacement = rollVector.mul(scaleFactor * pinchSize * -0.015);

        WbWorld.instance.viewpoint.position = position.add(zDisplacement);
        WbWorld.instance.viewpoint.updatePosition();

        this._scene.render();
      } else { // rotation (pitch and yaw)
        let halfPitchAngle = 0.005 * this._moveParams.dy;
        let halfYawAngle = -0.005 * this._moveParams.dx;
        if (this.picker.selectedId === -1) {
          halfPitchAngle /= -8;
          halfYawAngle /= -8;
        }
        const sinusYaw = Math.sin(halfYawAngle);
        const sinusPitch = Math.sin(halfPitchAngle);
        const pitch = right(orientation);
        const pitchRotation = glm.quat(Math.cos(halfPitchAngle), sinusPitch * pitch.x, sinusPitch * pitch.y,
          sinusPitch * pitch.z);
        const worldUpVector = WbWorld.instance.upVector;
        const yawRotation = glm.quat(Math.cos(halfYawAngle), sinusYaw * worldUpVector.x, sinusYaw * worldUpVector.y,
          sinusYaw * worldUpVector.z);

        // Updates camera's position and orientation
        const deltaRotation = yawRotation.mul(pitchRotation);
        const currentPosition = deltaRotation.mul(glm.vec3(position.x, position.y, position.z).sub(this._rotationCenter))
          .add(this._rotationCenter);
        const currentOrientation = deltaRotation.mul(vec4ToQuaternion(orientation));
        WbWorld.instance.viewpoint.position = new WbVector3(currentPosition.x, currentPosition.y, currentPosition.z);
        WbWorld.instance.viewpoint.orientation = quaternionToVec4(currentOrientation);
        WbWorld.instance.viewpoint.updatePosition();
        WbWorld.instance.viewpoint.updateOrientation();
        this._scene.render();
      }

      this._state.touchDistance = newTouchDistance;
      this._state.moved = true;
    }
    this._state.x = x;
    this._state.y = y;

    if (typeof webots.currentView.ontouchmove === 'function')
      webots.currentView.ontouchmove(event);
  }

  _onTouchStart(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    this.init();
    this._initMouseMove(event.targetTouches['0']);
    if (event.targetTouches.length === 2) {
      const touch1 = event.targetTouches['1'];
      this._state.x1 = touch1.clientX;
      this._state.y1 = touch1.clientY;
      const distanceX = this._state.x - this._state.x1;
      const distanceY = this._state.y - this._state.y1;
      this._state.touchDistance = distanceX * distanceX + distanceY * distanceY;
      this._state.touchOrientation = Math.atan2(this._state.y1 - this._state.y, this._state.x1 - this._state.x);
      this._state.mouseDown = 3; // two fingers: rotation, tilt, zoom
    } else
      this._state.mouseDown = 2; // 1 finger: translation or single click

    this._domElement.addEventListener('touchend', this.ontouchend, true);
    this._domElement.addEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchstart === 'function')
      webots.currentView.ontouchstart(event);

    const position = MouseEvents.convertMouseEventPositionToRelativePosition(canvas, this._state.x, this._state.y);
    this.picker.pick(position.x, position.y);
    if (this.picker.selectedId !== -1) {
      this._rotationCenter = new WbVector3((this.picker.coordinates.x / canvas.width) * 2 - 1,
        (this.picker.coordinates.y / canvas.height) * 2 - 1, this.picker.coordinates.z);
      this._rotationCenter = WbWorld.instance.viewpoint.toWorld(this._rotationCenter);
      this._rotationCenter = glm.vec3(this._rotationCenter.x, this._rotationCenter.y, this._rotationCenter.z);
    } else
      this._rotationCenter = glm.vec3(WbWorld.instance.viewpoint.position.x, WbWorld.instance.viewpoint.position.y,
        WbWorld.instance.viewpoint.position.z);
  }

  _onTouchEnd(event) {
    this._clearMouseMove();
    this._selectAndHandleClick();

    this._domElement.removeEventListener('touchend', this.ontouchend, true);
    this._domElement.removeEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchend === 'function')
      webots.currentView.ontouchend(event);
  }

  _initMouseMove(event) {
    this._state.x = event.clientX;
    this._state.y = event.clientY;
    this._state.initialX = null;
    this._state.initialY = null;
    this._state.moved = false;
    this._state.initialTimeStamp = Date.now();
    this._state.longClick = false;
  }

  _clearMouseMove() {
    const timeDelay = this._mobileDevice ? 100 : 1000;
    this._state.longClick = Date.now() - this._state.initialTimeStamp >= timeDelay;
    this._state.previousMouseDown = this._state.mouseDown;
    this._state.mouseDown = 0;
    this._state.initialTimeStamp = null;
    this._state.initialX = null;
    this._state.initialY = null;
    this._moveParams = {};
  }

  _selectAndHandleClick() {
    if (this._state.moved === false && (!this._state.longClick || this._mobileDevice)) {
      Selector.select(this.picker.selectedId);

      if (typeof WbWorld.instance.nodes.get(Selector.selectedId) !== 'undefined')
        WbWorld.instance.nodes.get(Selector.selectedId).updateBoundingObjectVisibility();

      if (typeof WbWorld.instance.nodes.get(Selector.previousId) !== 'undefined')
        WbWorld.instance.nodes.get(Selector.previousId).updateBoundingObjectVisibility();

      this._scene.render();
    }
  }

  _detectImmobility() {
    clearTimeout(this._moveTimeout);
    if (typeof this.showToolbar !== 'undefined')
      this.showToolbar();

    this._moveTimeout = setTimeout(() => {
      if (typeof this.hideToolbar !== 'undefined')
        this.hideToolbar();
    }, 3000);
  }
}

MouseEvents.convertMouseEventPositionToRelativePosition = (element, eventX, eventY) => {
  const rectangle = element.getBoundingClientRect();
  const position = new glm.vec2();
  position.x = Math.round(eventX - rectangle.left);
  position.y = Math.round(eventY - rectangle.top);
  return position;
};

MouseEvents.Click = {
  RIGHT_CLICK: 0,
  LEFT_CLICK: 1,
  WHEEL_CLICK: 2
};
