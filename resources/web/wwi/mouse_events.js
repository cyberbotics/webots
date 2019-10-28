/* global webots, THREE, SystemInfo */
'use strict';

class MouseEvents { // eslint-disable-line no-unused-vars
  constructor(scene, contextMenu, domElement, mobileDevice) {
    this.scene = scene;
    this.contextMenu = contextMenu;
    this.domElement = domElement;
    this.mobileDevice = mobileDevice;

    this.state = {
      'initialized': false,
      'mouseDown': 0,
      'moved': false,
      'wheelFocus': false,
      'wheelTimeout': null,
      'hiddenContextMenu': false
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

  _onMouseDown(event) {
    this.state.wheelFocus = true;
    this._initMouseMove(event);

    switch (event.button) {
      case THREE.MOUSE.LEFT:
        this.state.mouseDown |= 1;
        break;
      case THREE.MOUSE.MIDDLE:
        this.state.mouseDown |= 4;
        break;
      case THREE.MOUSE.RIGHT:
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

    if (this.state.mouseDown === 1) { // left mouse button to rotate viewpoint
      this.scene.viewpoint.rotate(this.moveParams);
    } else {
      if (this.state.mouseDown === 2) { // right mouse button to translate viewpoint
        this.moveParams.dx = event.clientX - this.state.initialX;
        this.moveParams.dy = event.clientY - this.state.initialY;
        this.scene.viewpoint.translate(this.moveParams);
      } else if (this.state.mouseDown === 3 || this.state.mouseDown === 4) { // both left and right button or middle button to zoom
        this.moveParams.tiltAngle = 0.01 * this.moveParams.dx;
        this.moveParams.zoomScale = this.moveParams.scaleFactor * 5 * this.moveParams.dy;
        this.scene.viewpoint.zoomAndTilt(this.moveParams, true);
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
    event.preventDefault(); // do not scroll page
    if (!('initialCameraPosition' in this.moveParams))
      this._setupMoveParameters(event);
    // else another drag event is already active

    if (!this.enableNavigation || this.state.wheelFocus === false) {
      var offset = event.deltaY;
      if (event.deltaMode === 1)
        offset *= 40; // standard line height in pixel
      window.scroll(0, window.pageYOffset + offset);
      if (this.state.wheelTimeout) { // you have to rest at least 1.5 seconds over the x3d canvas
        clearTimeout(this.state.wheelTimeout); // so that the wheel focus will get enabled and
        this.state.wheelTimeout = setTimeout((event) => { this._wheelTimeoutCallback(event); }, 1500); // allow you to zoom in/out.
      }
      return;
    }
    this.scene.viewpoint.zoom(this.moveParams.distanceToPickPosition, event.deltaY);

    if (typeof webots.currentView.onmousewheel === 'function')
      webots.currentView.onmousewheel(event);
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

    var touch = event.targetTouches['0'];
    var x = Math.round(touch.clientX); // discard decimal values returned on android
    var y = Math.round(touch.clientY);

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
      this.scene.viewpoint.translate(this.moveParams);
    } else {
      var touch1 = event.targetTouches['1'];
      var x1 = Math.round(touch1.clientX);
      var y1 = Math.round(touch1.clientY);
      var distanceX = x - x1;
      var distanceY = y - y1;
      var newTouchDistance = distanceX * distanceX + distanceY * distanceY;
      var pinchSize = this.state.touchDistance - newTouchDistance;

      var moveX1 = x - this.state.x;
      var moveX2 = x1 - this.state.x1;
      var moveY1 = y - this.state.y;
      var moveY2 = y1 - this.state.y1;
      var ratio = window.devicePixelRatio || 1;

      if (Math.abs(pinchSize) > 500 * ratio) { // zoom and tilt
        var d;
        if (Math.abs(moveX2) < Math.abs(moveX1))
          d = moveX1;
        else
          d = moveX2;
        this.moveParams.tiltAngle = 0.0004 * d;
        this.moveParams.zoomScale = this.moveParams.scaleFactor * 0.015 * pinchSize;
        this.scene.viewpoint.zoomAndTilt(this.moveParams);
      } else if (Math.abs(moveY2 - moveY1) < 3 * ratio && Math.abs(moveX2 - moveX1) < 3 * ratio) { // rotation (pitch and yaw)
        this.moveParams.dx = moveX1 * 0.8;
        this.moveParams.dy = moveY1 * 0.5;
        this.scene.viewpoint.rotate(this.moveParams);
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
    this._initMouseMove(event.targetTouches['0']);
    if (event.targetTouches.length === 2) {
      var touch1 = event.targetTouches['1'];
      this.state.x1 = touch1.clientX;
      this.state.y1 = touch1.clientY;
      var distanceX = this.state.x - this.state.x1;
      var distanceY = this.state.y - this.state.y1;
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
    if (this.contextMenu)
      this.hiddenContextMenu = this.contextMenu.toggle();
  }

  _setupMoveParameters(event) {
    this.moveParams = {};
    var relativePosition = MouseEvents.convertMouseEventPositionToRelativePosition(this.scene.renderer.domElement, event.clientX, event.clientY);
    var screenPosition = MouseEvents.convertMouseEventPositionToScreenPosition(this.scene.renderer.domElement, event.clientX, event.clientY);
    this.intersection = this.scene.pick(relativePosition, screenPosition);

    if (this.intersection && this.intersection.object)
      this.moveParams.pickPosition = this.intersection.point;
    else
      this.moveParams.pickPosition = null;

    if (this.intersection == null) {
      var cameraPosition = new THREE.Vector3();
      this.scene.viewpoint.camera.getWorldPosition(cameraPosition);
      this.moveParams.distanceToPickPosition = cameraPosition.length();
    } else
      this.moveParams.distanceToPickPosition = this.intersection.distance;
    if (this.moveParams.distanceToPickPosition < 0.001) // 1 mm
      this.moveParams.distanceToPickPosition = 0.001;

    // Webots mFieldOfView corresponds to the horizontal FOV, i.e. viewpoint.fovX.
    this.moveParams.scaleFactor = this.moveParams.distanceToPickPosition * 2 * Math.tan(0.5 * this.scene.viewpoint.camera.fovX);
    var viewHeight = parseFloat($(this.scene.domElement).css('height').slice(0, -2));
    var viewWidth = parseFloat($(this.scene.domElement).css('width').slice(0, -2));
    this.moveParams.scaleFactor /= Math.max(viewHeight, viewWidth);

    this.moveParams.initialCameraPosition = this.scene.viewpoint.camera.position.clone();
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
      var object;
      if (this.intersection) {
        object = this.intersection.object;
        if (object)
          object = this.scene.getTopX3dNode(object);
      }
      this.scene.selector.select(object);

      if (((this.mobileDevice && this.state.longClick) || (!this.mobileDevice && this.state.previousMouseDown === 2)) &&
        this.hiddenContextMenu === false && this.contextMenu)
        // Right click: show popup menu.
        this.contextMenu.show(object, {x: this.state.x, y: this.state.y});
    }
  }
}

MouseEvents.convertMouseEventPositionToScreenPosition = (element, eventX, eventY) => {
  var rect = element.getBoundingClientRect();
  var pos = new THREE.Vector2();
  pos.x = ((eventX - rect.left) / (rect.right - rect.left)) * 2 - 1;
  pos.y = -((eventY - rect.top) / (rect.bottom - rect.top)) * 2 + 1;
  return pos;
};

MouseEvents.convertMouseEventPositionToRelativePosition = (element, eventX, eventY) => {
  var rect = element.getBoundingClientRect();
  var pos = new THREE.Vector2();
  pos.x = Math.round(eventX - rect.left);
  pos.y = Math.round(eventY - rect.top);
  return pos;
};
