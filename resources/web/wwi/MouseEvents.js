import {direction, up, right, length, vec4ToQuaternion, quaternionToVec4, fromAxisAngle} from './nodes/utils/utils.js';
import Selector from './Selector.js';
import SystemInfo from './system_info.js';
import {webots} from './webots.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbWorld from './nodes/WbWorld.js';
import WbWrenPicker from './wren/WbWrenPicker.js';

export default class MouseEvents {
  #domElement;
  #enableNavigation;
  #mobileDevice;
  #moveParams;
  #moveTimeout;
  #rotationCenter;
  #scene;
  #state;
  constructor(scene, domElement, mobileDevice) {
    this.#scene = scene;
    this.#domElement = domElement;
    this.#mobileDevice = mobileDevice;

    this.#state = {
      'initialized': false,
      'mouseDown': 0,
      'moved': false,
      'wheelFocus': false,
      'wheelTimeout': null
    };
    this.#moveParams = {};
    this.#enableNavigation = true;

    this.onmousemove = (event) => { this.#onMouseMove(event); };
    this.onmouseup = (event) => { this.#onMouseUp(event); };
    this.ontouchmove = (event) => { this.#onTouchMove(event); };
    this.ontouchend = (event) => { this.#onTouchEnd(event); };
    this.#domElement.addEventListener('mousedown', (event) => { this.#onMouseDown(event); }, false);
    this.#domElement.addEventListener('mouseover', (event) => { this.#onMouseOver(event); }, false);
    this.#domElement.addEventListener('mouseleave', (event) => { this.onMouseLeave(event); }, false);
    this.#domElement.addEventListener('wheel', (event) => { this.#onMouseWheel(event); }, false);
    this.#domElement.addEventListener('touchstart', (event) => { this.#onTouchStart(event); }, true);
    this.#domElement.addEventListener('contextmenu', (event) => { event.preventDefault(); }, false);
    this.#domElement.addEventListener('mousemove', () => this.#detectImmobility());
    // Prevent '#playerDiv' to raise the context menu of the browser.
    // This bug has been seen on Windows 10 / Firefox only.
    this.#domElement.parentNode.addEventListener('contextmenu', (event) => { event.preventDefault(); }, false);
  }

  init() {
    if (typeof this.picker === 'undefined')
      this.picker = new WbWrenPicker();
  }

  #onMouseDown(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    this.init();

    this.#state.wheelFocus = true;
    this.#initMouseMove(event);
    switch (event.button) {
      case MouseEvents.Click.RIGHT_CLICK:
        this.#state.mouseDown |= 1;
        break;
      case MouseEvents.Click.LEFT_CLICK:
        this.#state.mouseDown |= 4;
        break;
      case MouseEvents.Click.WHEEL_CLICK:
        this.#state.mouseDown |= 2;
        break;
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this.#state.mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this.#state.mouseDown = 2;

    if (this.#state.mouseDown !== 0) {
      this.#state.initialX = event.clientX;
      this.#state.initialY = event.clientY;
      document.addEventListener('mousemove', this.onmousemove, false);
      document.addEventListener('mouseup', this.onmouseup, false);
    }

    if (typeof webots.currentView.onmousedown === 'function')
      webots.currentView.onmousedown(event);

    const position = MouseEvents.convertMouseEventPositionToRelativePosition(canvas, this.#state.x, this.#state.y);
    this.picker.pick(position.x, position.y);
    if (this.picker.selectedId !== -1) {
      this.#rotationCenter = new WbVector3((this.picker.coordinates.x / canvas.width) * 2 - 1,
        (this.picker.coordinates.y / canvas.height) * 2 - 1, this.picker.coordinates.z);
      this.#rotationCenter = WbWorld.instance.viewpoint.toWorld(this.#rotationCenter);
      this.#rotationCenter = glm.vec3(this.#rotationCenter.x, this.#rotationCenter.y, this.#rotationCenter.z);
    } else
      this.#rotationCenter = glm.vec3(WbWorld.instance.viewpoint.position.x, WbWorld.instance.viewpoint.position.y,
        WbWorld.instance.viewpoint.position.z);
  }

  #onMouseMove(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    if (!this.#enableNavigation && event.button === 0) {
      if (typeof webots.currentView.onmousemove === 'function')
        webots.currentView.onmousemove(event);
      return;
    }

    if (typeof this.#state.x === 'undefined')
      // mousedown event has not been called yet.
      // This could happen for example when another application has focus while loading the scene.
      return;
    if ('buttons' in event)
      this.#state.mouseDown = event.buttons;
    else if ('which' in event) { // Safari only
      switch (event.which) {
        case 0: this.#state.mouseDown = 0; break;
        case 1: this.#state.mouseDown = 1; break;
        case 2: this.#state.pressedButton = 4; break;
        case 3: this.#state.pressedButton = 2; break;
        default: this.#state.pressedButton = 0; break;
      }
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this.#state.mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this.#state.mouseDown = 2;

    if (this.#state.mouseDown === 0)
      return;

    if (this.#state.initialTimeStamp === null)
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;

    this.#moveParams.dx = event.clientX - this.#state.x;
    this.#moveParams.dy = event.clientY - this.#state.y;

    const orientation = WbWorld.instance.viewpoint.orientation;
    const position = WbWorld.instance.viewpoint.position;

    if (this.#state.mouseDown === 1) { // left mouse button to rotate viewpoint
      let halfPitchAngle = 0.005 * this.#moveParams.dy;
      let halfYawAngle = -0.005 * this.#moveParams.dx;
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
      const currentPosition = deltaRotation.mul(glm.vec3(position.x, position.y, position.z).sub(this.#rotationCenter))
        .add(this.#rotationCenter);
      const currentOrientation = deltaRotation.mul(vec4ToQuaternion(orientation));
      WbWorld.instance.viewpoint.position = new WbVector3(currentPosition.x, currentPosition.y, currentPosition.z);
      WbWorld.instance.viewpoint.orientation = quaternionToVec4(currentOrientation);
      WbWorld.instance.viewpoint.updatePosition();
      WbWorld.instance.viewpoint.updateOrientation();
      this.#scene.render();
    } else {
      let distanceToPickPosition = 0.001;
      if (this.picker.selectedId !== -1)
        distanceToPickPosition = length(position.sub(this.#rotationCenter));
      else
        distanceToPickPosition = length(position);

      if (distanceToPickPosition < 0.001)
        distanceToPickPosition = 0.001;

      const scaleFactor = distanceToPickPosition * 2 * Math.tan(WbWorld.instance.viewpoint.fieldOfView / 2) /
        Math.max(canvas.width, canvas.height);

      if (this.#state.mouseDown === 2) { // right mouse button to translate viewpoint
        const targetRight = scaleFactor * this.#moveParams.dx;
        const targetUp = scaleFactor * this.#moveParams.dy;
        const upVec = up(orientation);
        const rightVec = right(orientation);
        const targetR = rightVec.mul(targetRight);
        const targetU = upVec.mul(targetUp);
        const target = targetR.add(targetU);
        WbWorld.instance.viewpoint.position = position.add(target);
        WbWorld.instance.viewpoint.updatePosition();
        this.#scene.render();
      } else if (this.#state.mouseDown === 3 || this.#state.mouseDown === 4) {
        // both left and right button or middle button to zoom
        const rollVector = direction(orientation);
        const zDisplacement = rollVector.mul(scaleFactor * -5 * this.#moveParams.dy);
        const roll2 = fromAxisAngle(rollVector.x, rollVector.y, rollVector.z, 0.01 * this.#moveParams.dx);
        const roll3 = glm.quat();
        roll3.w = roll2.w;
        roll3.x = roll2.x;
        roll3.y = roll2.y;
        roll3.z = roll2.z;

        WbWorld.instance.viewpoint.position = position.add(zDisplacement);
        WbWorld.instance.viewpoint.orientation = quaternionToVec4(roll3.mul(vec4ToQuaternion(orientation)));
        WbWorld.instance.viewpoint.updatePosition();
        WbWorld.instance.viewpoint.updateOrientation();

        this.#scene.render();
      }
    }
    this.#state.moved = event.clientX !== this.#state.x || event.clientY !== this.#state.y;
    this.#state.x = event.clientX;
    this.#state.y = event.clientY;

    if (typeof webots.currentView.onmousemove === 'function')
      webots.currentView.onmousemove(event);
    if (typeof webots.currentView.onmousedrag === 'function')
      webots.currentView.onmousedrag(event);
  }

  #onMouseUp(event) {
    this.#clearMouseMove();
    this.#selectAndHandleClick();

    document.removeEventListener('mousemove', this.onmousemove, false);
    document.removeEventListener('mouseup', this.onmouseup, false);

    if (typeof webots.currentView.onmouseup === 'function')
      webots.currentView.onmouseup(event);
  }

  #onMouseWheel(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    this.init();
    this.#detectImmobility();

    event.preventDefault(); // do not scroll page

    if (!this.#enableNavigation || this.#state.wheelFocus === false) {
      let offset = event.deltaY;
      if (event.deltaMode === 1)
        offset *= 40; // standard line height in pixel
      window.scroll(0, window.pageYOffset + offset);
      if (this.#state.wheelTimeout) { // you have to rest at least 1.5 seconds over the x3d canvas
        clearTimeout(this.#state.wheelTimeout); // so that the wheel focus will get enabled and
        // allow you to zoom in/out.
        this.#state.wheelTimeout = setTimeout((event) => { this.#wheelTimeoutCallback(event); }, 1500);
      }
      return;
    }

    let distanceToPickPosition;
    const position = WbWorld.instance.viewpoint.position;
    if (this.picker.selectedId !== -1)
      distanceToPickPosition = length(position.sub(this.#rotationCenter));
    else
      distanceToPickPosition = length(position);

    if (distanceToPickPosition < 0.001)
      distanceToPickPosition = 0.001;

    const direct = direction(WbWorld.instance.viewpoint.orientation);
    const scaleFactor = 0.1 * (event.deltaY < 0.0 ? 1 : -1) * distanceToPickPosition;
    const zDisplacement = direct.mul(scaleFactor);

    WbWorld.instance.viewpoint.position = position.add(zDisplacement);
    WbWorld.instance.viewpoint.updatePosition();

    this.#scene.render();
  }

  #wheelTimeoutCallback(event) {
    this.#state.wheelTimeout = null;
    this.#state.wheelFocus = true;
  }

  #onMouseOver(event) {
    this.#state.wheelTimeout = setTimeout((event) => { this.#wheelTimeoutCallback(event); }, 1500);

    if (typeof this.showToolbar !== 'undefined')
      this.showToolbar();
  }

  onMouseLeave(event) {
    clearTimeout(this.#moveTimeout);

    if (typeof event !== 'undefined' && event.relatedTarget != null &&
      event.relatedTarget.id === 'time-slider')
      return;

    if (this.#state.wheelTimeout != null) {
      clearTimeout(this.#state.wheelTimeout);
      this.#state.wheelTimeout = null;
    }

    this.#state.wheelFocus = false;

    if (typeof webots.currentView.onmouseleave === 'function')
      webots.currentView.onmouseleave(event);

    if (typeof this.hideToolbar !== 'undefined')
      this.hideToolbar();
  }

  #onTouchMove(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    if (!this.#enableNavigation || event.targetTouches.length === 0 || event.targetTouches.length > 2)
      return;
    if (this.#state.initialTimeStamp === null)
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;
    if ((this.#state.mouseDown !== 2) !== (event.targetTouches.length > 1))
      // Gesture single/multi touch changed after initialization.
      return;

    const touch = event.targetTouches['0'];
    const x = Math.round(touch.clientX); // discard decimal values returned on android
    const y = Math.round(touch.clientY);

    this.#moveParams.dx = x - this.#state.x;
    this.#moveParams.dy = y - this.#state.y;

    const orientation = WbWorld.instance.viewpoint.orientation;
    const position = WbWorld.instance.viewpoint.position;

    let distanceToPickPosition = 0.001;
    if (this.picker.selectedId !== -1)
      distanceToPickPosition = length(position.sub(this.#rotationCenter));
    else
      distanceToPickPosition = length(position);

    if (distanceToPickPosition < 0.001)
      distanceToPickPosition = 0.001;

    const scaleFactor = distanceToPickPosition * 2 * Math.tan(WbWorld.instance.viewpoint.fieldOfView / 2) /
      Math.max(canvas.width, canvas.height);

    if (this.#state.mouseDown === 2) { // translation
      const targetRight = scaleFactor * this.#moveParams.dx;
      const targetUp = scaleFactor * this.#moveParams.dy;
      const upVec = up(orientation);
      const rightVec = right(orientation);
      const targetR = rightVec.mul(targetRight);
      const targetU = upVec.mul(targetUp);
      const target = targetR.add(targetU);
      WbWorld.instance.viewpoint.position = position.add(target);
      WbWorld.instance.viewpoint.updatePosition();
      this.#scene.render();
    } else {
      const touch1 = event.targetTouches['1'];
      const x1 = Math.round(touch1.clientX);
      const y1 = Math.round(touch1.clientY);
      const distanceX = x - x1;
      const distanceY = y - y1;
      const newTouchDistance = distanceX * distanceX + distanceY * distanceY;
      const pinchSize = this.#state.touchDistance - newTouchDistance;
      const ratio = 1;

      if (Math.abs(pinchSize) > 500 * ratio) { // zoom and tilt
        const rollVector = direction(orientation);
        const zDisplacement = rollVector.mul(scaleFactor * pinchSize * -0.015);

        WbWorld.instance.viewpoint.position = position.add(zDisplacement);
        WbWorld.instance.viewpoint.updatePosition();

        this.#scene.render();
      } else { // rotation (pitch and yaw)
        let halfPitchAngle = 0.005 * this.#moveParams.dy;
        let halfYawAngle = -0.005 * this.#moveParams.dx;
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
        const currentPosition = deltaRotation.mul(glm.vec3(position.x, position.y, position.z).sub(this.#rotationCenter))
          .add(this.#rotationCenter);
        const currentOrientation = deltaRotation.mul(vec4ToQuaternion(orientation));
        WbWorld.instance.viewpoint.position = new WbVector3(currentPosition.x, currentPosition.y, currentPosition.z);
        WbWorld.instance.viewpoint.orientation = quaternionToVec4(currentOrientation);
        WbWorld.instance.viewpoint.updatePosition();
        WbWorld.instance.viewpoint.updateOrientation();
        this.#scene.render();
      }

      this.#state.touchDistance = newTouchDistance;
      this.#state.moved = true;
    }
    this.#state.x = x;
    this.#state.y = y;

    if (typeof webots.currentView.ontouchmove === 'function')
      webots.currentView.ontouchmove(event);
  }

  #onTouchStart(event) {
    if (typeof WbWorld.instance === 'undefined')
      return;

    this.init();
    this.#initMouseMove(event.targetTouches['0']);
    if (event.targetTouches.length === 2) {
      const touch1 = event.targetTouches['1'];
      this.#state.x1 = touch1.clientX;
      this.#state.y1 = touch1.clientY;
      const distanceX = this.#state.x - this.#state.x1;
      const distanceY = this.#state.y - this.#state.y1;
      this.#state.touchDistance = distanceX * distanceX + distanceY * distanceY;
      this.#state.touchOrientation = Math.atan2(this.#state.y1 - this.#state.y, this.#state.x1 - this.#state.x);
      this.#state.mouseDown = 3; // two fingers: rotation, tilt, zoom
    } else
      this.#state.mouseDown = 2; // 1 finger: translation or single click

    this.#domElement.addEventListener('touchend', this.ontouchend, true);
    this.#domElement.addEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchstart === 'function')
      webots.currentView.ontouchstart(event);

    const position = MouseEvents.convertMouseEventPositionToRelativePosition(canvas, this.#state.x, this.#state.y);
    this.picker.pick(position.x, position.y);
    if (this.picker.selectedId !== -1) {
      this.#rotationCenter = new WbVector3((this.picker.coordinates.x / canvas.width) * 2 - 1,
        (this.picker.coordinates.y / canvas.height) * 2 - 1, this.picker.coordinates.z);
      this.#rotationCenter = WbWorld.instance.viewpoint.toWorld(this.#rotationCenter);
      this.#rotationCenter = glm.vec3(this.#rotationCenter.x, this.#rotationCenter.y, this.#rotationCenter.z);
    } else
      this.#rotationCenter = glm.vec3(WbWorld.instance.viewpoint.position.x, WbWorld.instance.viewpoint.position.y,
        WbWorld.instance.viewpoint.position.z);
  }

  #onTouchEnd(event) {
    this.#clearMouseMove();
    this.#selectAndHandleClick();

    this.#domElement.removeEventListener('touchend', this.ontouchend, true);
    this.#domElement.removeEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchend === 'function')
      webots.currentView.ontouchend(event);
  }

  #initMouseMove(event) {
    this.#state.x = event.clientX;
    this.#state.y = event.clientY;
    this.#state.initialX = null;
    this.#state.initialY = null;
    this.#state.moved = false;
    this.#state.initialTimeStamp = Date.now();
    this.#state.longClick = false;
  }

  #clearMouseMove() {
    const timeDelay = this.#mobileDevice ? 100 : 1000;
    this.#state.longClick = Date.now() - this.#state.initialTimeStamp >= timeDelay;
    this.#state.previousMouseDown = this.#state.mouseDown;
    this.#state.mouseDown = 0;
    this.#state.initialTimeStamp = null;
    this.#state.initialX = null;
    this.#state.initialY = null;
    this.#moveParams = {};
  }

  #selectAndHandleClick() {
    if (this.#state.moved === false && (!this.#state.longClick || this.#mobileDevice)) {
      Selector.select(this.picker.selectedId);

      if (typeof WbWorld.instance.nodes.get(Selector.selectedId) !== 'undefined')
        WbWorld.instance.nodes.get(Selector.selectedId).updateBoundingObjectVisibility();

      if (typeof WbWorld.instance.nodes.get(Selector.previousId) !== 'undefined')
        WbWorld.instance.nodes.get(Selector.previousId).updateBoundingObjectVisibility();

      this.#scene.render();
    }
  }

  #detectImmobility() {
    clearTimeout(this.#moveTimeout);
    if (typeof this.showToolbar !== 'undefined')
      this.showToolbar();

    this.#moveTimeout = setTimeout(() => {
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
