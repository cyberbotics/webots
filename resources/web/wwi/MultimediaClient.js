'use strict';
import SystemInfo from './system_info.js';
import {webots} from './webots.js';

export default class MultimediaClient {
  #domElement;
  #lastMousePosition;
  #lastHeight;
  #lastTouchEvent;
  #lastWidth;
  #mouseDown;
  #touchEvent;
  #view;
  #viewMode;
  constructor(view, parentObject) {
    this.#view = view;
    this.#domElement = document.createElement('img');
    this.#domElement.style.background = 'white';
    this.#domElement.id = 'remoteScene';
    this.#domElement.setAttribute('draggable', false);
    parentObject.appendChild(this.#domElement);

    this.#viewMode = false;

    this.#mouseDown = 0;
    this.onmousemove = (e) => { this.#onMouseMove(e); };
    this.#lastMousePosition = null;
    this.#touchEvent = { move: false };
    this.ontouchmove = (e) => { this.#onTouchMove(e); };
    this.ontouchend = (e) => { this.#onTouchEnd(e); };
  }

  disconnect() {
    this.#domElement.src = '';
  }

  finalize(onready) {
    this.#domElement.addEventListener('mousedown', (e) => { this.#onMouseDown(e); }, false);
    this.#domElement.addEventListener('mouseup', (e) => { this.#onMouseUp(e); }, false);
    this.#domElement.addEventListener('wheel', (e) => { this.#onWheel(e); }, false);
    this.#domElement.addEventListener('touchstart', (event) => { this.#onTouchStart(event); }, true);
    this.#domElement.addEventListener('contextmenu', (event) => { event.preventDefault(); }, false);
    if (typeof onready === 'function')
      onready();
  }

  setFollowed(solidId, mode) {
    const socket = this.#view.stream.socket;
    if (!socket || socket.readyState !== 1)
      return;
    socket.send('follow: ' + mode + ',' + solidId);
  }

  requestNewSize() {
    if (this.#lastWidth === this.#domElement.width && this.#lastHeight === this.#domElement.height)
      return;
    this.#view.stream.socket.send('resize: ' + this.#domElement.width + 'x' + this.#domElement.height);
    this.#lastWidth = this.#domElement.width;
    this.#lastHeight = this.#domElement.height;
  }

  resize(width, height) {
    this.requestNewSize();
  }

  processServerMessage(data) {
    if (data.startsWith('multimedia: ')) {
      const list = data.split(' ');
      const httpUrl = 'http' + this.#view.stream.wsServer.slice(2); // replace 'ws' with 'http'
      const url = httpUrl + list[1];
      this.#domElement.src = url;
      this.#viewMode = list.length > 4; // client in view mode
      if (this.#viewMode) {
        this.#domElement.style.width = list[3] + 'px';
        this.#domElement.style.height = list[4] + 'px';
      }
      console.log('Multimedia streamed on ' + url);
    } else if (data.startsWith('resize: ')) {
      if (this.#viewMode) {
        let list = data.split(' ');
        this.#domElement.style.width = list[1] + 'px';
        this.#domElement.style.height = list[2] + 'px';
      } // else ignore resize triggered from this instance
    } else
      return false;
    return true;
  }

  #onMouseDown(event) {
    if (this.#viewMode)
      return false;

    this.#mouseDown = 0;
    switch (event.button) {
      case 0: // MOUSE.LEFT
        this.#mouseDown |= 1;
        break;
      case 1: // MOUSE.MIDDLE
        this.#mouseDown |= 4;
        break;
      case 2: // MOUSE.RIGHT
        this.#mouseDown |= 2;
        break;
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this.#mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this.#mouseDown = 2;

    event.target.addEventListener('mousemove', this.onmousemove, false);
    this.#sendMouseEvent(-1, this.#computeRemoteMouseEvent(event), 0);
    event.preventDefault();
    return false;
  }

  #onMouseMove(event) {
    if (this.#mouseDown === 0) {
      event.target.removeEventListener('mousemove', this.onmousemove, false);
      return false;
    } else if (event.buttons === 0) {
      // mouse button released outside the 3D scene
      // send a mouse up event to complete the drag
      let mouseUpEvent = this.#computeRemoteMouseEvent(event);
      mouseUpEvent.offsetX = this.#lastMousePosition.x;
      mouseUpEvent.offsetY = this.#lastMousePosition.y;
      event.target.removeEventListener('mousemove', this.onmousemove, false);
      this.#sendMouseEvent(1, mouseUpEvent, 0);
      event.preventDefault();
      return false;
    }
    this.#sendMouseEvent(0, this.#computeRemoteMouseEvent(event), 0);
    return false;
  }

  #onMouseUp(event) {
    if (this.#viewMode)
      return;

    event.target.removeEventListener('mousemove', this.onmousemove, false);
    this.#sendMouseEvent(1, this.#computeRemoteMouseEvent(event), 0);
    event.preventDefault();
    return false;
  }

  #onWheel(event) {
    if (this.#viewMode)
      return false;

    this.#sendMouseEvent(2, this.#computeRemoteMouseEvent(event), Math.sign(event.deltaY));
    return false;
  }

  #onTouchStart(event) {
    if (this.#viewMode)
      return;

    this.#mouseDown = 0;
    let touch0 = event.targetTouches['0'];
    this.#touchEvent.x = Math.round(touch0.clientX); // discard decimal values returned on android
    this.#touchEvent.y = Math.round(touch0.clientY);
    if (event.targetTouches.length === 2) {
      const touch1 = event.targetTouches['1'];
      this.#touchEvent.x1 = touch1.clientX;
      this.#touchEvent.y1 = touch1.clientY;
      const distanceX = this.#touchEvent.x - this.#touchEvent.x1;
      const distanceY = this.#touchEvent.y - this.#touchEvent.y1;
      this.#touchEvent.distance = distanceX * distanceX + distanceY * distanceY;
      this.#touchEvent.orientation = Math.atan2(this.#touchEvent.y1 - this.#touchEvent.y,
        this.#touchEvent.x1 - this.#touchEvent.x);
      this.#mouseDown = 3; // two fingers: rotation, tilt, zoom
    } else
      this.#mouseDown = 2; // 1 finger: translation or single click

    event.target.addEventListener('touchend', this.ontouchend, true);
    event.target.addEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchstart === 'function')
      webots.currentView.ontouchstart(event);

    this.#touchEvent.initialTimeStamp = Date.now();
    this.#touchEvent.moved = false;
    this.#touchEvent.initialX = null;
    this.#touchEvent.initialY = null;
    this.#touchEvent.mode = undefined;
    this.#lastTouchEvent = this.#computeTouchEvent(event);
    this.#sendMouseEvent(-1, this.#lastTouchEvent, 2);
    event.preventDefault();
    return false;
  }

  #onTouchMove(event) {
    if (this.#viewMode || event.targetTouches.length === 0 || event.targetTouches.length > 2)
      return false;
    if (typeof this.#touchEvent.initialTimeStamp === 'undefined')
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;
    if ((typeof this.#touchEvent.mode !== 'undefined') && (this.#mouseDown !== 3))
      // Gesture single/multi touch changed after initialization.
      return false;

    let touch0 = event.targetTouches['0'];
    let x = Math.round(touch0.clientX); // discard decimal values returned on android
    let y = Math.round(touch0.clientY);
    let dx = x - this.#touchEvent.x;
    let dy = y - this.#touchEvent.y;

    if (this.#mouseDown === 2) { // translation
      // On small phone screens (Android) this is needed to correctly detect clicks and longClicks.
      if (this.#touchEvent.initialX == null && this.#touchEvent.initialY == null) {
        this.#touchEvent.initialX = Math.round(this.#touchEvent.x);
        this.#touchEvent.initialY = Math.round(this.#touchEvent.y);
      }
      if (Math.abs(dx) < 2 && Math.abs(dy) < 2 &&
        Math.abs(this.#touchEvent.initialX - x) < 5 && Math.abs(this.#touchEvent.initialY - y) < 5)
        this.#touchEvent.moved = false;

      else
        this.#touchEvent.moved = true;

      this.#lastTouchEvent = this.#computeTouchEvent(event);
      if (this.#touchEvent.moved)
        this.#sendMouseEvent(0, this.#lastTouchEvent, 2);
    } else {
      let touch1 = event.targetTouches['1'];
      let x1 = Math.round(touch1.clientX);
      let y1 = Math.round(touch1.clientY);
      let distanceX = x - x1;
      let distanceY = y - y1;
      let newTouchDistance = distanceX * distanceX + distanceY * distanceY;
      let pinchSize = this.#touchEvent.distance - newTouchDistance;

      let moveX1 = x - this.#touchEvent.x;
      let moveX2 = x1 - this.#touchEvent.x1;
      let moveY1 = y - this.#touchEvent.y;
      let moveY2 = y1 - this.#touchEvent.y1;
      let ratio = window.devicePixelRatio || 1;

      // Initialize multi-touch gesture.
      if (typeof this.#touchEvent.mode === 'undefined') {
        if (Date.now() - this.#touchEvent.initialTimeStamp < 100)
          // Wait some ms to be able to detect the gesture.
          return;
        if (Math.abs(pinchSize) > 500 * ratio) { // zoom and tilt
          this.#mouseDown = 3;
          this.#touchEvent.mode = 2;
        } else if (Math.abs(moveY2 - moveY1) < 3 * ratio && Math.abs(moveX2 - moveX1) < 3 * ratio) { // rotation (pitch and yaw)
          this.#mouseDown = 3;
          this.#touchEvent.mode = 1;
        } else
          return;

        if (typeof this.#touchEvent.mode !== 'undefined')
          // send rotation/zoom picked position
          this.#sendMessage('touch -1 ' + this.#touchEvent.mode + ' ' + x + ' ' + y);
      } else if (this.#touchEvent.mode === 2) {
        let d;
        if (Math.abs(moveX2) < Math.abs(moveX1))
          d = moveX1;
        else
          d = moveX2;
        let tiltAngle = 0.0004 * d;
        let zoomScale = 0.015 * pinchSize;
        this.#sendMessage('touch 0 ' + this.#touchEvent.mode + ' ' + tiltAngle + ' ' + zoomScale);
      } else { // rotation
        dx = moveX1 * 0.8;
        dy = moveY1 * 0.5;
        this.#sendMessage('touch 0 ' + this.#touchEvent.mode + ' ' + dx + ' ' + dy);
      }

      this.#touchEvent.moved = true;
      this.#touchEvent.distance = newTouchDistance;
      this.#touchEvent.x1 = x1;
      this.#touchEvent.y1 = y1;
    }

    this.#touchEvent.x = x;
    this.#touchEvent.y = y;

    if (typeof webots.currentView.ontouchmove === 'function')
      webots.currentView.ontouchmove(event);
    event.preventDefault();
    return false;
  }

  #onTouchEnd(event) {
    if (this.#viewMode)
      return false;

    this.#domElement.removeEventListener('touchend', this.ontouchend, true);
    this.#domElement.removeEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchend === 'function')
      webots.currentView.ontouchend(event);

    if (typeof this.#lastTouchEvent === 'undefined') {
      this.#touchEvent.initialTimeStamp = undefined;
      event.preventDefault();
      return false;
    }

    if (typeof this.#touchEvent.mode === 'undefined') {
      let longClick = Date.now() - this.#touchEvent.initialTimeStamp >= 100;
      if (this.#touchEvent.move === false && !longClick) {
        // A single short click corresponds to left click to select items.
        this.#lastTouchEvent.button = 0;
        this.#lastTouchEvent.buttons = 1;
        this.#sendMouseEvent(-1, this.#lastTouchEvent, longClick);
      }
      this.#sendMouseEvent(1, this.#lastTouchEvent, longClick);
    }
    this.#lastTouchEvent = undefined;
    this.#touchEvent.initialTimeStamp = undefined;
    this.#touchEvent.move = false;
    this.#touchEvent.mode = undefined;
    event.preventDefault();
    return false;
  }

  #computeRemoteMouseEvent(event) {
    let remoteEvent = {};
    remoteEvent.button = event.button;
    remoteEvent.buttons = this.#mouseDown;
    remoteEvent.modifier = (event.shiftKey ? 1 : 0) + (event.ctrlKey ? 2 : 0) + (event.altKey ? 4 : 0);
    remoteEvent.offsetX = event.offsetX;
    remoteEvent.offsetY = event.offsetY;
    return remoteEvent;
  }

  #computeTouchEvent(event, move) {
    let remoteEvent = {};
    remoteEvent.buttons = this.#mouseDown;
    remoteEvent.modifier = 0; // disabled
    const touch = event.targetTouches['0'];
    remoteEvent.offsetX = Math.round(touch.clientX); // discard decimal values returned on android
    remoteEvent.offsetY = Math.round(touch.clientY);

    if (this.#touchEvent.mode === 3) // zoom/tilt
      remoteEvent.button = 3;
    else if (this.#touchEvent.mode === 1) // rotation
      remoteEvent.button = 1;
    else
      remoteEvent.button = 2; // 1 finger: translation or single click

    // Adjust touch point coordinates so that they are local to the remote scene.
    remoteEvent.offsetX -= this.#domElement.x;
    remoteEvent.offsetY -= this.#domElement.y;
    return remoteEvent;
  }

  #sendMessage(message) {
    let socket = this.#view.stream.socket;
    if (!socket || socket.readyState !== 1)
      return;
    socket.send(message);
  }

  #sendMouseEvent(type, event, wheel) {
    this.#sendMessage('mouse ' + type + ' ' + event.button + ' ' + event.buttons + ' ' +
                event.offsetX + ' ' + event.offsetY + ' ' + event.modifier + ' ' + wheel);
    this.#lastMousePosition = {x: event.offsetX, y: event.offsetY};
  }
}
