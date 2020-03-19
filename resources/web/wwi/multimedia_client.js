/* global webots, SystemInfo */

'use strict';

class MultimediaClient { // eslint-disable-line no-unused-vars
  constructor(view, parentObject, contextMenu) {
    this.view = view;
    this.domElement = document.createElement('img');
    this.domElement.style.background = 'white';
    this.domElement.id = 'remoteScene';
    this.domElement.setAttribute('draggable', false);
    parentObject.appendChild(this.domElement);

    this.navigationEnabled = true;
    this.contextMenu = contextMenu;
    this.robotWindows = [];
    this.worldInfo = {title: null, infoWindow: null};

    this.mouseDown = 0;
    this.onmousemove = (e) => { this._onMouseMove(e); };
    this.lastMousePosition = null;
    this.touchEvent = { move: false };
    this.ontouchmove = (e) => { this._onTouchMove(e); };
    this.ontouchend = (e) => { this._onTouchEnd(e); };
  }

  disconnect() {
    this.domElement.src = '';
  }

  finalize(onready) {
    this.domElement.addEventListener('mousedown', (e) => { this._onMouseDown(e); }, false);
    this.domElement.addEventListener('mouseup', (e) => { this._onMouseUp(e); }, false);
    this.domElement.addEventListener('wheel', (e) => { this._onWheel(e); }, false);
    this.domElement.addEventListener('touchstart', (event) => { this._onTouchStart(event); }, true);
    this.domElement.addEventListener('contextmenu', (e) => { this._onContextMenu(e); }, false);
    if (typeof onready === 'function')
      onready();
  }

  setWorldInfo(title, infoWindowName) {
    this.worldInfo = {title: title, infoWindow: infoWindowName};
  }

  setRobotWindow(robotName, windowName) {
    this.robotWindows.push([robotName, windowName]);
  }

  setFollowed(solidId, mode) {
    var socket = this.view.stream.socket;
    if (!socket || socket.readyState !== 1)
      return;
    socket.send('follow: ' + mode + ',' + solidId);
  }

  showContextMenu(object) {
    this.contextMenu.show(object, this.lastMousePosition);
  }

  requestNewSize() {
    if (this.lastWidth === this.domElement.width && this.lastHeight === this.domElement.height)
      return;
    this.view.stream.socket.send('resize: ' + this.domElement.width + 'x' + this.domElement.height);
    this.lastWidth = this.domElement.width;
    this.lastHeight = this.domElement.height;
  }

  resize(width, height) {
    this.requestNewSize();
  }

  processServerMessage(data) {
    if (data.startsWith('multimedia: ')) {
      let list = data.split(' ');
      let httpUrl = this.view.stream.wsServer.replace('ws', 'http');
      let url = httpUrl + list[1];
      this.view.toolBar.setMode(list[2]);
      this.domElement.src = url;
      if (list.length > 4) {
        this.domElement.style.width = list[3] + 'px';
        this.domElement.style.height = list[4] + 'px';
        this.navigationEnabled = false; // client in view mode
      }
      console.log('Multimedia streamed on ' + url);
    } else if (data.startsWith('resize: ')) {
      let list = data.split(' ');
      if (this.domElement.width !== parseInt(list[1]) || this.domElement.height !== parseInt(list[2])) {
        this.domElement.style.width = list[1] + 'px';
        this.domElement.style.height = list[2] + 'px';
        this.navigationEnabled = false; // client in view mode
      }
    } else if (data.startsWith('robot window: ')) {
      let robotInfo = data.substring(data.indexOf(':') + 1).trim();
      let separatorIndex = robotInfo.indexOf(':');
      let nameSize = parseFloat(robotInfo.substring(0, separatorIndex));
      let robotName = robotInfo.substring(separatorIndex + 1, nameSize + separatorIndex + 1);
      let windowName = robotInfo.substring(separatorIndex + nameSize + 2);
      this.setRobotWindow(robotName, windowName);
    } else if (data.startsWith('world info: ')) {
      // world info: <name size>:<info window name>:<world title>
      let info = data.substring(data.indexOf(':') + 1).trim();
      let separatorIndex = info.indexOf(':');
      let nameSize = parseFloat(info.substring(0, separatorIndex));
      let infoWindowName = info.substring(separatorIndex + 1, nameSize + separatorIndex + 1);
      let title = info.substring(separatorIndex + nameSize + 2);
      this.setWorldInfo(title, infoWindowName);
    } else if (data.startsWith('context menu: ')) {
      let info = data.substring(data.indexOf(':') + 1).trim();
      this.showContextMenu(JSON.parse(info));
    } else
      return false;
    return true;
  }

  _onMouseDown(event) {
    if (!this.navigationEnabled)
      return false;

    this.mouseDown = 0;
    switch (event.button) {
      case 0: // MOUSE.LEFT
        this.mouseDown |= 1;
        break;
      case 1: // MOUSE.MIDDLE
        this.mouseDown |= 4;
        break;
      case 2: // MOUSE.RIGHT
        this.mouseDown |= 2;
        break;
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this.mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this.mouseDown = 2;

    event.target.addEventListener('mousemove', this.onmousemove, false);
    this._sendMouseEvent(-1, this._computeRemoteMouseEvent(event), 0);
    event.preventDefault();
    return false;
  }

  _onMouseMove(event) {
    if (this.mouseDown === 0) {
      event.target.removeEventListener('mousemove', this.onmousemove, false);
      return false;
    }
    this._sendMouseEvent(0, this._computeRemoteMouseEvent(event), 0);
    return false;
  }

  _onMouseUp(event) {
    if (!this.navigationEnabled)
      return;

    event.target.removeEventListener('mousemove', this.onmousemove, false);
    this._sendMouseEvent(1, this._computeRemoteMouseEvent(event), 0);
    event.preventDefault();
    return false;
  }

  _onWheel(event) {
    if (!this.navigationEnabled)
      return false;

    this._sendMouseEvent(2, this._computeRemoteMouseEvent(event), Math.sign(event.deltaY));
    return false;
  }

  _onContextMenu(event) {
    event.preventDefault();
    return false;
  }

  _onTouchStart(event) {
    if (!this.navigationEnabled)
      return;

    this.mouseDown = 0;
    let touch0 = event.targetTouches['0'];
    this.touchEvent.x = Math.round(touch0.clientX); // discard decimal values returned on android
    this.touchEvent.y = Math.round(touch0.clientY);
    if (event.targetTouches.length === 2) {
      var touch1 = event.targetTouches['1'];
      this.touchEvent.x1 = touch1.clientX;
      this.touchEvent.y1 = touch1.clientY;
      var distanceX = this.touchEvent.x - this.touchEvent.x1;
      var distanceY = this.touchEvent.y - this.touchEvent.y1;
      this.touchEvent.distance = distanceX * distanceX + distanceY * distanceY;
      this.touchEvent.orientation = Math.atan2(this.touchEvent.y1 - this.touchEvent.y, this.touchEvent.x1 - this.touchEvent.x);
      this.mouseDown = 3; // two fingers: rotation, tilt, zoom
    } else
      this.mouseDown = 2; // 1 finger: translation or single click

    event.target.addEventListener('touchend', this.ontouchend, true);
    event.target.addEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchstart === 'function')
      webots.currentView.ontouchstart(event);

    this.touchEvent.initialTimeStamp = Date.now();
    this.touchEvent.moved = false;
    this.touchEvent.initialX = null;
    this.touchEvent.initialY = null;
    this.touchEvent.mode = undefined;
    this.lastTouchEvent = this._computeTouchEvent(event);
    this._sendMouseEvent(-1, this.lastTouchEvent, 2);
    event.preventDefault();
    return false;
  }

  _onTouchMove(event) {
    if (!this.navigationEnabled || event.targetTouches.length === 0 || event.targetTouches.length > 2)
      return false;
    if (typeof this.touchEvent.initialTimeStamp === 'undefined')
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;
    if ((typeof this.touchEvent.mode !== 'undefined') && (this.mouseDown !== 3))
      // Gesture single/multi touch changed after initialization.
      return false;

    let touch0 = event.targetTouches['0'];
    let x = Math.round(touch0.clientX); // discard decimal values returned on android
    let y = Math.round(touch0.clientY);
    let dx = x - this.touchEvent.x;
    let dy = y - this.touchEvent.y;

    if (this.mouseDown === 2) { // translation
      // On small phone screens (Android) this is needed to correctly detect clicks and longClicks.
      if (this.touchEvent.initialX == null && this.touchEvent.initialY == null) {
        this.touchEvent.initialX = Math.round(this.touchEvent.x);
        this.touchEvent.initialY = Math.round(this.touchEvent.y);
      }
      if (Math.abs(dx) < 2 && Math.abs(dy) < 2 &&
        Math.abs(this.touchEvent.initialX - x) < 5 && Math.abs(this.touchEvent.initialY - y) < 5)
        this.touchEvent.moved = false;

      else
        this.touchEvent.moved = true;

      this.lastTouchEvent = this._computeTouchEvent(event);
      if (this.touchEvent.moved)
        this._sendMouseEvent(0, this.lastTouchEvent, 2);
    } else {
      let touch1 = event.targetTouches['1'];
      let x1 = Math.round(touch1.clientX);
      let y1 = Math.round(touch1.clientY);
      let distanceX = x - x1;
      let distanceY = y - y1;
      let newTouchDistance = distanceX * distanceX + distanceY * distanceY;
      let pinchSize = this.touchEvent.distance - newTouchDistance;

      let moveX1 = x - this.touchEvent.x;
      let moveX2 = x1 - this.touchEvent.x1;
      let moveY1 = y - this.touchEvent.y;
      let moveY2 = y1 - this.touchEvent.y1;
      let ratio = window.devicePixelRatio || 1;

      // Initialize multi-touch gesture.
      if (typeof this.touchEvent.mode === 'undefined') {
        if (Date.now() - this.touchEvent.initialTimeStamp < 100)
          // Wait some ms to be able to detect the gesture.
          return;
        if (Math.abs(pinchSize) > 500 * ratio) { // zoom and tilt
          this.mouseDown = 3;
          this.touchEvent.mode = 2;
        } else if (Math.abs(moveY2 - moveY1) < 3 * ratio && Math.abs(moveX2 - moveX1) < 3 * ratio) { // rotation (pitch and yaw)
          this.mouseDown = 3;
          this.touchEvent.mode = 1;
        } else
          return;

        if (typeof this.touchEvent.mode !== 'undefined')
          // send rotation/zoom picked position
          this._sendMessage('touch -1 ' + this.touchEvent.mode + ' ' + x + ' ' + y);
      } else if (this.touchEvent.mode === 2) {
        let d;
        if (Math.abs(moveX2) < Math.abs(moveX1))
          d = moveX1;
        else
          d = moveX2;
        let tiltAngle, zoomScale;
        if (500 * Math.abs(d) > Math.abs(pinchSize)) {
          tiltAngle = 0.02 * d;
          zoomScale = 0;
        } else {
          tiltAngle = 0;
          zoomScale = 0.015 * pinchSize;
        }
        this._sendMessage('touch 0 ' + this.touchEvent.mode + ' ' + tiltAngle + ' ' + zoomScale);
      } else { // rotation
        dx = moveX1 * 0.8;
        dy = moveY1 * 0.5;
        this._sendMessage('touch 0 ' + this.touchEvent.mode + ' ' + dx + ' ' + dy);
      }

      this.touchEvent.moved = true;
      this.touchEvent.distance = newTouchDistance;
      this.touchEvent.x1 = x1;
      this.touchEvent.y1 = y1;
    }

    this.touchEvent.x = x;
    this.touchEvent.y = y;

    if (typeof webots.currentView.ontouchmove === 'function')
      webots.currentView.ontouchmove(event);
    return false;
  }

  _onTouchEnd(event) {
    if (!this.navigationEnabled)
      return false;

    this.domElement.removeEventListener('touchend', this.ontouchend, true);
    this.domElement.removeEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchend === 'function')
      webots.currentView.ontouchend(event);

    if (typeof this.lastTouchEvent === 'undefined') {
      this.touchEvent.initialTimeStamp = undefined;
      event.preventDefault();
      return false;
    }

    if (typeof this.touchEvent.mode === 'undefined') {
      let longClick = Date.now() - this.touchEvent.initialTimeStamp >= 100;
      if (this.touchEvent.move === false && !longClick) {
        // A single short click corresponds to left click to select items.
        this.lastTouchEvent.button = 0;
        this.lastTouchEvent.buttons = 1;
        this._sendMouseEvent(-1, this.lastTouchEvent, longClick);
      }
      this._sendMouseEvent(1, this.lastTouchEvent, longClick);
    }
    this.lastTouchEvent = undefined;
    this.touchEvent.initialTimeStamp = undefined;
    this.touchEvent.move = false;
    this.touchEvent.mode = undefined;
    event.preventDefault();
    return false;
  }

  _computeRemoteMouseEvent(event) {
    let remoteEvent = {};
    remoteEvent.button = event.button;
    remoteEvent.buttons = this.mouseDown;
    remoteEvent.modifier = (event.shiftKey ? 1 : 0) + (event.ctrlKey ? 2 : 0) + (event.altKey ? 4 : 0);
    remoteEvent.offsetX = event.offsetX;
    remoteEvent.offsetY = event.offsetY;
    return remoteEvent;
  }

  _computeTouchEvent(event, move) {
    let remoteEvent = {};
    remoteEvent.buttons = this.mouseDown;
    remoteEvent.modifier = 0; // disabled
    var touch = event.targetTouches['0'];
    remoteEvent.offsetX = Math.round(touch.clientX); // discard decimal values returned on android
    remoteEvent.offsetY = Math.round(touch.clientY);

    if (this.touchEvent.mode === 3) // zoom/tilt
      remoteEvent.button = 3;
    else if (this.touchEvent.mode === 1) // rotation
      remoteEvent.button = 1;
    else
      remoteEvent.button = 2; // 1 finger: translation or single click

    // Adjust touch point coordinates so that they are local to the remote scene.
    remoteEvent.offsetX -= this.domElement.x;
    remoteEvent.offsetY -= this.domElement.y;
    return remoteEvent;
  }

  _sendMessage(message) {
    this.contextMenu.hide();
    var socket = this.view.stream.socket;
    if (!socket || socket.readyState !== 1)
      return;
    socket.send(message);
  }

  _sendMouseEvent(type, event, wheel) {
    this.contextMenu.hide();
    this._sendMessage('mouse ' + type + ' ' + event.button + ' ' + event.buttons + ' ' +
                event.offsetX + ' ' + event.offsetY + ' ' + event.modifier + ' ' + wheel);
    this.lastMousePosition = {x: event.offsetX, y: event.offsetY};
  }
}
