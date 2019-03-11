/* global THREE */
'use strict';

function MouseEvents(sceneManager, contextMenu, domElement) {
  this.sceneManager = sceneManager;
  this.contextMenu = contextMenu;
  this.domElement = domElement;
  this.enabledContextMenu = true;
  // this.controls = new THREE.OrbitControls(this.sceneManager.camera, this.sceneManager.domElement);

  this.MOUSE_BUTTONS = {LEFT: THREE.MOUSE.LEFT, MIDDLE: THREE.MOUSE.MIDDLE, RIGHT: THREE.MOUSE.RIGHT};
  this.ACTION = { NONE: -1, ROTATE: 0, DOLLY: 1, PAN: 2, TOUCH_ROTATE: 3, TOUCH_DOLLY_PAN: 4 };
  this.EPS = 0.000001;
  this.action = this.ACTION.NONE;

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
  domElement.addEventListener('mousedown', function(event) { that.onMouseDown(event); }, false);
  domElement.addEventListener('mouseover', function(event) { that.onMouseOver(event); }, false);
  domElement.addEventListener('mouseleave', function(event) { that.onMouseLeave(event); }, false);
  domElement.addEventListener('wheel', function(event) { that.onMouseWheel(event); }, false);
  // domElement.addEventListener('touchstart', this.onTouchStart, false);
  // domElement.addEventListener('touchend', this.onTouchEnd, false);
  // domElement.addEventListener('touchmove', this.onTouchMove, false);
  domElement.addEventListener('contextmenu', function(event) { event.preventDefault(); }, false);
};

MouseEvents.prototype = {
  constructor: MouseEvents,

  onMouseDown: function(event) {
    switch (event.button) {
      case this.MOUSE_BUTTONS.LEFT:
        if (event.ctrlKey || event.metaKey || event.shiftKey) {
          //this.handleMouseDownPan(event);
          this.action = this.ACTION.PAN;
        } else {
          //this.handleMouseDownRotate(event);
          this.action = this.ACTION.ROTATE;
        }
        this.state.mouseDown |= 1;
        break;
      case this.MOUSE_BUTTONS.MIDDLE:
        //this.handleMouseDownDolly(event);
        this.action = this.ACTION.DOLLY;
        this.state.mouseDown |= 4;
        break;
      case this.MOUSE_BUTTONS.RIGHT:
        //this.handleMouseDownPan(event);
        this.action = this.ACTION.PAN;
        this.state.mouseDown |= 2;
        break;
    }

    this.state.wheelFocus = true;
    this._initMouseMove(event);

    var relativePosition = MouseEvents.convertMouseEventPositionToRelativePosition(this.sceneManager.renderer, event.clientX, event.clientY);
    var screenPosition = MouseEvents.convertMouseEventPositionToScreenPosition(this.sceneManager.renderer, event.clientX, event.clientY);
    this.intersection = this.sceneManager.pick(relativePosition, screenPosition);

    if (this.action !== this.ACTION.NONE) {
      var that = this;
      document.addEventListener('mousemove', function(event) { that.onMouseMove(event); }, false);
      document.addEventListener('mouseup', function(event) { that.onMouseUp(event); }, false);
    }
  },

  onMouseMove: function(event) {
    if (!this.enableNavigation && event.button === 0)
      return;
    if (this.state.x === undefined)
      // mousedown event has not been called yet
      // this could happen for example when another application has focus while loading the scene
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
    if (this.state.mouseDown === 0) {
      if (this.animation && this.animation.playSlider && this.animation.sliding) {
        // TODO is this needed?
        /* var w = event.target.clientWidth - 66; // size of the borders of the slider
        var x = event.clientX - event.target.getBoundingClientRect().left - 48; // size of the left border (including play button) of the slider
        var value = 100 * x / w;
        if (value < 0)
          value = 0;
        else if (value >= 100)
          value = 99.999;
        that.animation.playSlider.slider('value', value);
        // setting the value should trigger the change event, unfortunately, doesn't seem to work reliably,
        // therefore, we need to trigger this event manually:
        var ui = {};
        ui.value = value;
        that.animation.playSlider.slider('option', 'change').call(that.animation.playSlider, event, ui); */
      }
      return;
    }

    if (this.state.initialTimeStamp === null)
      // prevent applying mouse move action before drag initialization in mousedrag event
      return;

    var params = {};
    params.dx = event.clientX - this.state.x;
    params.dy = event.clientY - this.state.y;

    if (this.intersection == null)
      params.distanceToPickPosition = this.sceneManager.viewpoint.camera.position;
    else
      params.distanceToPickPosition = this.intersection.distance;

    if (params.distanceToPickPosition < 0.001) // 1 mm
      params.distanceToPickPosition = 0.001;

    // FIXME this is different from webots. We need to understand why the same formula doesn't work.
    params.scaleFactor = 1.90 * Math.tan(this.sceneManager.viewpoint.camera.fov / 2);
    var viewHeight = parseFloat($(this.sceneManager.domElement).css('height').slice(0, -2));
    var viewWidth = parseFloat($(this.sceneManager.domElement).css('width').slice(0, -2));
    params.scaleFactor /= Math.max(viewHeight, viewWidth);

    if (this.state.mouseDown === 1) { // left mouse button to rotate viewpoint
      params.distanceToPickPosition = 0;
      this.sceneManager.viewpoint.rotate(params);
    } else if (this.state.mouseDown === 2) // right mouse button to translate viewpoint {}
      this.sceneManager.viewpoint.translateViewpoint(params);
    else if (this.state.mouseDown === 3 || this.state.mouseDown === 4) { // both left and right button or middle button to zoom
      params.tiltAngle = 0.01 * params.dx;
      params.zoomScale = params.distanceToPickPosition * params.scaleFactor * 10 * params.dy; // FIXME this is different from webots.
      this.sceneManager.viewpoint.zoomAndTilt(params, true);
    }
    this.state.moved = event.clientX !== this.state.x || event.clientY !== this.state.y;
    this.state.x = event.clientX;
    this.state.y = event.clientY;
  },

  onMouseUp: function(event) {
    this._clearMouseMove();
    if (this.state.moved === false && (!this.state.longClick || this.mobileDevice)) {
      var object = this.intersection.object;
      if (object)
        object = this.sceneManager.getTopX3dNode(object);
      // if (this.previousSelection == null || this.previousSelection.id !== s.id || (this.state.previousMouseDown === 2 && (!this.mobileDevice || this.state.longClick)))
      this.sceneManager.selector.select(object);

      if (((this.mobileDevice && this.state.longClick) || (!this.mobileDevice && this.state.previousMouseDown === 2)) &&
        this.hiddenContextMenu === false && this.enabledContextMenu)
        // right click: show popup menu
        this.contextMenu.show(object, {x: this.state.x, y: this.state.y});
    }
  },

  onMouseWheel: function(event) {
    if (this.intersection.distance < 0.001) // 1 mm
      this.intersection.distance = 0.001;
    if (!this.enableNavigation || this.state.wheelFocus === false) {
      var offset = event.deltaY;
      if (event.deltaMode === 1)
        offset *= 40; // standard line height in pixel
      window.scroll(0, window.pageYOffset + offset);
      if (this.state.wheelTimeout) { // you have to rest at least 1.5 seconds over the x3d canvas
        clearTimeout(this.state.wheelTimeout); // so that the wheel focus will get enabled and
        var that;
        this.state.wheelTimeout = setTimeout(function(event) { that.wheelTimeoutCallback(event); }, 1500); // allow you to zoom in/out.
      }
      return;
    }
    this.sceneManager.viewpoint.zoom(this.intersection.distance, event.deltaY);
  },

  wheelTimeoutCallback: function(event) {
    this.state.wheelTimeout = null;
    this.state.wheelFocus = true;
  },

  onMouseOver: function(event) {
    var that = this;
    this.state.wheelTimeout = setTimeout(function(event) { that.wheelTimeoutCallback(event); }, 1500);
  },

  onMouseLeave: function(event) {
    if (this.state.wheelTimeout != null) {
      clearTimeout(this.state.wheelTimeout);
      this.state.wheelTimeout = null;
    }
    this.state.wheelFocus = false;
  },

  _initMouseMove: function(event) {
    this.state.x = event.clientX;
    this.state.y = event.clientY;
    this.state.initialX = null;
    this.state.initialY = null;
    this.state.moved = false;
    this.state.initialTimeStamp = Date.now();
    this.state.longClick = false;
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
    this.action = this.ACTION.NONE;

    // TODO fix listener function
    document.removeEventListener('mousemove', this.onMouseMove, false);
    document.removeEventListener('mouseup', this.onMouseUp, false);
  }

  /* if (that.mobileDevice) {
    that.x3dNode.addEventListener('touchmove', function(event) {
      if (!that.enableNavigation || event.targetTouches.length === 0 || event.targetTouches.length > 2)
        return;
      if (that.state.initialTimeStamp === null)
        // prevent applying mouse move action before drag initialization in mousedrag event
        return;
      if ((that.state.mouseDown !== 2) !== (event.targetTouches.length > 1))
        // gesture single/multi touch changed after initialization
        return;

      var touch = event.targetTouches['0'];

      var params = {};
      var viewpoint = that.x3dSceneManager.getElementsByTagName('Viewpoint')[0];
      params.vp = x3dom.fields.SFVec3f.parse(viewpoint.getAttribute('position'));
      params.vo = x3dom.fields.SFVec4f.parse(viewpoint.getAttribute('orientation'));
      params.c = Math.cos(params.vo.w);
      params.s = Math.sin(params.vo.w);
      params.scaleFactor = 1.90 * Math.tan(that.viewpointFieldOfView / 2);
      var viewHeight = parseFloat($(that.x3dNode).css('height').slice(0, -2));
      var viewWidth = parseFloat($(that.x3dNode).css('width').slice(0, -2));
      params.scaleFactor /= Math.max(viewHeight, viewWidth);

      if (that.state.pickPosition == null)
        params.distanceToPickPosition = params.vp.length();
      else
        params.distanceToPickPosition = params.vp.subtract(that.state.pickPosition).length() - 0.05; // FIXME this is different from webots.
      if (params.distanceToPickPosition < 0.001) // 1 mm
        params.distanceToPickPosition = 0.001;
      var x = Math.round(touch.clientX); // discard decimal values returned on android
      var y = Math.round(touch.clientY);

      if (that.state.mouseDown === 2) { // translation
        params.dx = x - that.state.x;
        params.dy = y - that.state.y;
        translateViewpoint(viewpoint, params);

        // on small phone screens (Android) this is needed to correctly detect clicks and longClicks
        if (that.state.initialX == null && that.state.initialY == null) {
          that.state.initialX = Math.round(that.state.x);
          that.state.initialY = Math.round(that.state.y);
        }
        if (Math.abs(params.dx) < 2 && Math.abs(params.dy) < 2 &&
            Math.abs(that.state.initialX - x) < 5 && Math.abs(that.state.initialY - y) < 5)
          that.state.moved = false;
        else
          that.state.moved = true;
      } else {
        var touch1 = event.targetTouches['1'];
        var x1 = Math.round(touch1.clientX);
        var y1 = Math.round(touch1.clientY);
        var distanceX = x - x1;
        var distanceY = y - y1;
        var newTouchDistance = distanceX * distanceX + distanceY * distanceY;
        var pinchSize = that.state.touchDistance - newTouchDistance;

        var moveX1 = x - that.state.x;
        var moveX2 = x1 - that.state.x1;
        var moveY1 = y - that.state.y;
        var moveY2 = y1 - that.state.y1;
        var ratio = window.devicePixelRatio || 1;

        if (Math.abs(pinchSize) > 500 * ratio) { // zoom and tilt
          var d;
          if (Math.abs(moveX2) < Math.abs(moveX1))
            d = moveX1;
          else
            d = moveX2;
          params.tiltAngle = 0.0004 * d;
          params.zoomScale = params.scaleFactor * 0.015 * pinchSize;
          zoomAndTiltViewpoint(viewpoint, params);
        } else if (Math.abs(moveY2 - moveY1) < 3 * ratio && Math.abs(moveX2 - moveX1) < 3 * ratio) { // rotation (pitch and yaw)
          params.dx = moveX1 * 0.8;
          params.dy = moveY1 * 0.5;
          rotateViewpoint(viewpoint, params);
        }

        that.state.touchDistance = newTouchDistance;
        that.state.moved = true;
      }

      that.state.x = x;
      that.state.y = y;
      that.state.x1 = x1;
      that.state.y1 = y1;
      if (that.ontouchmove)
        that.ontouchmove(event);
    }, true);
    that.x3dNode.addEventListener('touchstart', function(event) {
      initMouseMove(event.targetTouches['0']);
      if (event.targetTouches.length === 2) {
        var touch1 = event.targetTouches['1'];
        that.state.x1 = touch1.clientX;
        that.state.y1 = touch1.clientY;
        var distanceX = that.state.x - that.state.x1;
        var distanceY = that.state.y - that.state.y1;
        that.state.touchDistance = distanceX * distanceX + distanceY * distanceY;
        that.state.touchOrientation = Math.atan2(that.state.y1 - that.state.y, that.state.x1 - that.state.x);
        that.state.mouseDown = 3; // two fingers: rotation, tilt, zoom
      } else
        that.state.mouseDown = 2; // 1 finger: translation or single click
    }, true);
    that.x3dNode.addEventListener('touchend', function(event) {
      clearMouseMove();
      if (that.ontouchend)
        that.ontouchend(event);
    }, true); */
};

MouseEvents.convertMouseEventPositionToScreenPosition = function(renderer, eventX, eventY) {
  var rect = renderer.domElement.getBoundingClientRect();
  var pos = new THREE.Vector2();
  pos.x = ((eventX - rect.left) / (rect.right - rect.left)) * 2 - 1;
  pos.y = ((eventY - rect.top) / (rect.bottom - rect.top)) * 2 + 1;
  return pos;
};

MouseEvents.convertMouseEventPositionToRelativePosition = function(renderer, eventX, eventY) {
  var rect = renderer.domElement.getBoundingClientRect();
  var pos = new THREE.Vector2();
  pos.x = Math.round(eventX - rect.left);
  pos.y = Math.round(eventY - rect.top);
  return pos;
};
