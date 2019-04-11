/* global webots, THREE */
'use strict';

function MouseEvents(sceneManager, contextMenu, domElement) {
  this.sceneManager = sceneManager;
  this.contextMenu = contextMenu;
  this.domElement = domElement;

  this.state = {
    'initialized': false,
    'mouseDown': 0,
    'moved': false,
    'wheelFocus': false,
    'wheelTimeout': null,
    'hiddenContextMenu': false
  };
  this.enableNavigation = true;

  var that = this;
  this.onmousemove = function(event) { that._onMouseMove(event); };
  this.onmouseup = function(event) { that._onMouseUp(event); };
  this.ontouchmove = function(event) { that._onTouchMove(event); };
  this.ontouchend = function(event) {
    that._clearMouseMove();
    that.domElement.removeEventListener('touchend', that._onTouchEnd, true);
    that.domElement.removeEventListener('touchmove', that._onTouchMove, true);
    if (that._onTouchEnd)
      that._onTouchEnd(event);
  };
  domElement.addEventListener('mousedown', function(event) { that._onMouseDown(event); }, false);
  domElement.addEventListener('mouseover', function(event) { that._onMouseOver(event); }, false);
  domElement.addEventListener('mouseleave', function(event) { that._onMouseLeave(event); }, false);
  domElement.addEventListener('wheel', function(event) { that._onMouseWheel(event); }, false);
  domElement.addEventListener('touchstart', function(event) { that._onTouchStart(event); }, true);
  domElement.addEventListener('contextmenu', function(event) { event.preventDefault(); }, false);
};

MouseEvents.prototype = {
  constructor: MouseEvents,

  _onMouseDown: function(event) {
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

    if (this.state.mouseDown !== 0) {
      var relativePosition = MouseEvents.convertMouseEventPositionToRelativePosition(this.sceneManager.renderer, event.clientX, event.clientY);
      var screenPosition = MouseEvents.convertMouseEventPositionToScreenPosition(event.clientX, event.clientY);
      this.intersection = this.sceneManager.pick(relativePosition, screenPosition);

      document.addEventListener('mousemove', this.onmousemove, false);
      document.addEventListener('mouseup', this.onmouseup, false);
    }

    if (webots.currentView.onmousedown)
      webots.currentView.onmousedown(event);
  },

  _onMouseMove: function(event) {
    if (!this.enableNavigation && event.button === 0) {
      if (webots.currentView.onmousemove)
        webots.currentView.onmousemove(event);
      return;
    }

    if (this.state.x === undefined)
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
    if (this.state.mouseDown === 0)
      return;

    if (this.state.initialTimeStamp === null)
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;

    var params = {};
    params.dx = event.clientX - this.state.x;
    params.dy = event.clientY - this.state.y;

    if (this.state.mouseDown === 1) { // left mouse button to rotate viewpoint
      if (this.intersection && this.intersection.object) {
        params.pickPosition = new THREE.Vector3();
        this.intersection.object.getWorldPosition(params.pickPosition);
      } else
        params.pickPosition = null;
      this.sceneManager.viewpoint.rotate(params);
    } else {
      if (this.intersection == null) {
        var cameraPosition = new THREE.Vector3();
        this.sceneManager.viewpoint.camera.getWorldPosition(cameraPosition);
        params.distanceToPickPosition = cameraPosition.length();
      } else
        params.distanceToPickPosition = this.intersection.distance;
      if (params.distanceToPickPosition < 0.001) // 1 mm
        params.distanceToPickPosition = 0.001;
      // FIXME this is different from webots. We need to understand why the same formula doesn't work.
      // THREEjs Camera fov is half of Webots Viewpoint fov.
      params.scaleFactor = params.distanceToPickPosition * 2.4 * Math.tan(THREE.Math.degToRad(this.sceneManager.viewpoint.camera.fov));
      var viewHeight = parseFloat($(this.sceneManager.domElement).css('height').slice(0, -2));
      var viewWidth = parseFloat($(this.sceneManager.domElement).css('width').slice(0, -2));
      params.scaleFactor /= Math.max(viewHeight, viewWidth);
      if (this.state.mouseDown === 2) { // right mouse button to translate viewpoint
        this.sceneManager.viewpoint.translate(params);
      } else if (this.state.mouseDown === 3 || this.state.mouseDown === 4) { // both left and right button or middle button to zoom
        params.tiltAngle = 0.01 * params.dx;
        params.zoomScale = params.scaleFactor * 5 * params.dy;
        this.sceneManager.viewpoint.zoomAndTilt(params, true);
      }
    }
    this.state.moved = event.clientX !== this.state.x || event.clientY !== this.state.y;
    this.state.x = event.clientX;
    this.state.y = event.clientY;

    if (webots.currentView.onmousemove)
      webots.currentView.onmousemove(event);
    if (webots.currentView.onmousedrag)
      webots.currentView.onmousedrag(event);
  },

  _onMouseUp: function(event) {
    this._clearMouseMove();
    if (this.state.moved === false && (!this.state.longClick || this.mobileDevice)) {
      var object = this.intersection.object;
      if (object)
        object = this.sceneManager.getTopX3dNode(object);
      this.sceneManager.selector.select(object);

      if (((this.mobileDevice && this.state.longClick) || (!this.mobileDevice && this.state.previousMouseDown === 2)) &&
        this.hiddenContextMenu === false && this.contextMenu)
        // Right click: show popup menu.
        this.contextMenu.show(object, {x: this.state.x, y: this.state.y});
    }

    document.removeEventListener('mousemove', this.onmousemove, false);
    document.removeEventListener('mouseup', this.onmouseup, false);

    if (webots.currentView.onmouseup)
      webots.currentView.onmouseup(event);
  },

  _onMouseWheel: function(event) {
    event.preventDefault(); // do not scroll page

    if (this.intersection.distance < 0.001) // 1 mm
      this.intersection.distance = 0.001;
    if (!this.enableNavigation || this.state.wheelFocus === false) {
      var offset = event.deltaY;
      if (event.deltaMode === 1)
        offset *= 40; // standard line height in pixel
      window.scroll(0, window.pageYOffset + offset);
      if (this.state.wheelTimeout) { // you have to rest at least 1.5 seconds over the x3d canvas
        clearTimeout(this.state.wheelTimeout); // so that the wheel focus will get enabled and
        var that = this;
        this.state.wheelTimeout = setTimeout(function(event) { that._wheelTimeoutCallback(event); }, 1500); // allow you to zoom in/out.
      }
      return;
    }
    this.sceneManager.viewpoint.zoom(this.intersection.distance, event.deltaY);

    if (webots.currentView.onmousewheel)
      webots.currentView.onmousewheel(event);
  },

  _wheelTimeoutCallback: function(event) {
    this.state.wheelTimeout = null;
    this.state.wheelFocus = true;
  },

  _onMouseOver: function(event) {
    var that = this;
    this.state.wheelTimeout = setTimeout(function(event) { that._wheelTimeoutCallback(event); }, 1500);
  },

  _onMouseLeave: function(event) {
    if (this.state.wheelTimeout != null) {
      clearTimeout(this.state.wheelTimeout);
      this.state.wheelTimeout = null;
    }
    this.state.wheelFocus = false;

    if (webots.currentView.onmouseleave)
      webots.currentView.onmouseleave(event);
  },

  _onTouchMove: function(event) {
    if (!this.enableNavigation || event.targetTouches.length === 0 || event.targetTouches.length > 2)
      return;
    if (this.state.initialTimeStamp === null)
      // Prevent applying mouse move action before drag initialization in mousedrag event.
      return;
    if ((this.state.mouseDown !== 2) !== (event.targetTouches.length > 1))
      // Gesture single/multi touch changed after initialization.
      return;

    var touch = event.targetTouches['0'];

    var params = {};
    if (this.intersection == null) {
      var cameraPosition = new THREE.Vector3();
      this.sceneManager.viewpoint.camera.getWorldPosition(cameraPosition);
      params.distanceToPickPosition = cameraPosition.length();
    } else
      params.distanceToPickPosition = this.intersection.distance;
    if (params.distanceToPickPosition < 0.001) // 1 mm
      params.distanceToPickPosition = 0.001;
    // FIXME this is different from webots. We need to understand why the same formula doesn't work.
    // THREEjs Camera fov is half of Webots Viewpoint fov.
    params.scaleFactor = params.distanceToPickPosition * 2.4 * Math.tan(THREE.Math.degToRad(this.sceneManager.viewpoint.camera.fov));
    var viewHeight = parseFloat($(this.sceneManager.domElement).css('height').slice(0, -2));
    var viewWidth = parseFloat($(this.sceneManager.domElement).css('width').slice(0, -2));
    params.scaleFactor /= Math.max(viewHeight, viewWidth);

    var x = Math.round(touch.clientX); // discard decimal values returned on android
    var y = Math.round(touch.clientY);

    if (this.state.mouseDown === 2) { // translation
      params.dx = x - this.state.x;
      params.dy = y - this.state.y;
      this.sceneManager.viewpoint.translate(params);

      // On small phone screens (Android) this is needed to correctly detect clicks and longClicks.
      if (this.state.initialX == null && this.state.initialY == null) {
        this.state.initialX = Math.round(this.state.x);
        this.state.initialY = Math.round(this.state.y);
      }
      if (Math.abs(params.dx) < 2 && Math.abs(params.dy) < 2 &&
          Math.abs(this.state.initialX - x) < 5 && Math.abs(this.state.initialY - y) < 5)
        this.state.moved = false;
      else
        this.state.moved = true;
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
        params.tiltAngle = 0.0004 * d;
        params.zoomScale = params.scaleFactor * 0.015 * pinchSize;
        this.sceneManager.viewpoint.zoomAndTilt(params);
      } else if (Math.abs(moveY2 - moveY1) < 3 * ratio && Math.abs(moveX2 - moveX1) < 3 * ratio) { // rotation (pitch and yaw)
        params.dx = moveX1 * 0.8;
        params.dy = moveY1 * 0.5;
        this.sceneManager.viewpoint.rotate(params);
      }

      this.state.touchDistance = newTouchDistance;
      this.state.moved = true;
    }

    this.state.x = x;
    this.state.y = y;
    this.state.x1 = x1;
    this.state.y1 = y1;

    if (webots.currentView.ontouchmove)
      webots.currentView.ontouchmove(event);
  },

  _onTouchStart: function(event) {
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

    this.domElement.addEventListener('touchend', this.ontouchend, true);
    this.domElement.addEventListener('touchmove', this.ontouchmove, true);

    if (webots.currentView.ontouchstart)
      webots.currentView.ontouchstart(event);
  },

  _onTouchEnd: function(event) {
    this._clearMouseMove();
    this.domElement.removeEventListener('touchend', this.ontouchend, true);
    this.domElement.removeEventListener('touchmove', this.ontouchmove, true);

    if (webots.currentView.ontouchend)
      webots.currentView.ontouchend(event);
  },

  _initMouseMove: function(event) {
    this.state.x = event.clientX;
    this.state.y = event.clientY;
    this.state.initialX = null;
    this.state.initialY = null;
    this.state.moved = false;
    this.state.initialTimeStamp = Date.now();
    this.state.longClick = false;
    if (this.contextMenu)
      this.hiddenContextMenu = this.contextMenu.toggle();
  },

  _clearMouseMove: function() {
    const timeDelay = this.state.mobileDevice ? 100 : 1000;
    this.state.longClick = Date.now() - this.state.initialTimeStamp >= timeDelay;
    if (this.state.moved === false) {
      this.previousSelection = this.selection;
      this.sceneManager.selector.clearSelection();
    } else
      this.previousSelection = null;
    this.state.previousMouseDown = this.state.mouseDown;
    this.state.mouseDown = 0;
    this.state.initialTimeStamp = null;
    this.state.initialX = null;
    this.state.initialY = null;
  }
};

MouseEvents.convertMouseEventPositionToScreenPosition = function(eventX, eventY) {
  return new THREE.Vector2(
    (eventX / window.innerWidt) * 2 - 1,
    -(eventY / window.innerHeight) * 2 + 1
  );
};

MouseEvents.convertMouseEventPositionToRelativePosition = function(renderer, eventX, eventY) {
  var rect = renderer.domElement.getBoundingClientRect();
  var pos = new THREE.Vector2();
  pos.x = Math.round(eventX - rect.left);
  pos.y = Math.round(eventY - rect.top);
  return pos;
};
