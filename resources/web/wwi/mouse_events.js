/* global webots, THREE */
'use strict';

function MouseEvents(scene, contextMenu, domElement) {
  this.scene = scene;
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
  this.moveParams = {};
  this.enableNavigation = true;

  var that = this;
  this.onmousemove = function(event) { that._onMouseMove(event); };
  this.onmouseup = function(event) { that._onMouseUp(event); };
  this.ontouchmove = function(event) { that._onTouchMove(event); };
  this.ontouchend = function(event) {
    that._clearMouseMove();
    that.domElement.removeEventListener('touchend', that._onTouchEnd, true);
    that.domElement.removeEventListener('touchmove', that._onTouchMove, true);
    if (typeof that._onTouchEnd === 'function')
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
      this._setupMoveParameters(event, true);
      this.state.initialX = event.clientX;
      this.state.initialY = event.clientY;
      document.addEventListener('mousemove', this.onmousemove, false);
      document.addEventListener('mouseup', this.onmouseup, false);
    }

    if (typeof webots.currentView.onmousedown === 'function')
      webots.currentView.onmousedown(event);
  },

  _onMouseMove: function(event) {
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
  },

  _onMouseUp: function(event) {
    this._clearMouseMove();
    if (this.state.moved === false && (!this.state.longClick || this.mobileDevice)) {
      var object = this.intersection.object;
      if (object)
        object = this.scene.getTopX3dNode(object);
      this.scene.selector.select(object);

      if (((this.mobileDevice && this.state.longClick) || (!this.mobileDevice && this.state.previousMouseDown === 2)) &&
        this.hiddenContextMenu === false && this.contextMenu)
        // Right click: show popup menu.
        this.contextMenu.show(object, {x: this.state.x, y: this.state.y});
    }

    document.removeEventListener('mousemove', this.onmousemove, false);
    document.removeEventListener('mouseup', this.onmouseup, false);

    if (typeof webots.currentView.onmouseup === 'function')
      webots.currentView.onmouseup(event);
  },

  _onMouseWheel: function(event) {
    event.preventDefault(); // do not scroll page

    this._setupMoveParameters(event, false);

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
    this.scene.viewpoint.zoom(this.moveParams.distanceToPickPosition, event.deltaY);

    if (typeof webots.currentView.onmousewheel === 'function')
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

    if (typeof webots.currentView.onmouseleave === 'function')
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

    this._setupMoveParameters(event.targetTouches['0'], true);
    this.domElement.addEventListener('touchend', this.ontouchend, true);
    this.domElement.addEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchstart === 'function')
      webots.currentView.ontouchstart(event);
  },

  _onTouchEnd: function(event) {
    this._clearMouseMove();
    this.domElement.removeEventListener('touchend', this.ontouchend, true);
    this.domElement.removeEventListener('touchmove', this.ontouchmove, true);

    if (typeof webots.currentView.ontouchend === 'function')
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

  _setupMoveParameters: function(event, computeScale) {
    this.moveParams = {};
    var relativePosition = MouseEvents.convertMouseEventPositionToRelativePosition(this.scene.renderer, event.clientX, event.clientY);
    var screenPosition = MouseEvents.convertMouseEventPositionToScreenPosition(event.clientX, event.clientY);
    this.intersection = this.scene.pick(relativePosition, screenPosition);

    if (this.intersection && this.intersection.object) {
      this.moveParams.pickPosition = new THREE.Vector3();
      this.intersection.object.getWorldPosition(this.moveParams.pickPosition);
    } else
      this.moveParams.pickPosition = null;

    if (this.intersection == null) {
      var cameraPosition = new THREE.Vector3();
      this.scene.viewpoint.camera.getWorldPosition(cameraPosition);
      this.moveParams.distanceToPickPosition = cameraPosition.length();
    } else
      this.moveParams.distanceToPickPosition = this.intersection.distance;
    if (this.moveParams.distanceToPickPosition < 0.001) // 1 mm
      this.moveParams.distanceToPickPosition = 0.001;

    if (computeScale) {
      // THREEjs Camera fov is half of Webots Viewpoint fov.
      this.moveParams.scaleFactor = this.moveParams.distanceToPickPosition * 2 * Math.tan(THREE.Math.degToRad(this.scene.viewpoint.camera.fov));
      var viewHeight = parseFloat($(this.scene.domElement).css('height').slice(0, -2));
      var viewWidth = parseFloat($(this.scene.domElement).css('width').slice(0, -2));
      this.moveParams.scaleFactor /= Math.max(viewHeight, viewWidth);
    }

    this.moveParams.initialCameraPosition = this.scene.viewpoint.camera.position.clone();
  },

  _clearMouseMove: function() {
    const timeDelay = this.state.mobileDevice ? 100 : 1000;
    this.state.longClick = Date.now() - this.state.initialTimeStamp >= timeDelay;
    if (this.state.moved === false) {
      this.previousSelection = this.selection;
      this.scene.selector.clearSelection();
    } else
      this.previousSelection = null;
    this.state.previousMouseDown = this.state.mouseDown;
    this.state.mouseDown = 0;
    this.state.initialTimeStamp = null;
    this.state.initialX = null;
    this.state.initialY = null;
    this.moveParams = {};
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
