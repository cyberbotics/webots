'use strict';

function Video(parentObject, mouseEvents, stream) {
  this.domElement = document.createElement('video');
  this.domElement.style.background = 'grey';
  this.domElement.id = 'remoteVideo';
  this.domElement.class = 'rounded centered';
  this.domElement.autoplay = 'true';
  this.domElement.width = 800;
  this.domElement.height = 600;
  parentObject.appendChild(this.domElement);
  this.mouseEvents = mouseEvents;
  this.stream = stream;

  this.onmousemove = (e) => { this._onMouseMove(e); };
};

Video.prototype = {
  constructor: Video,

  finalize: function(onready) {
    this.domElement.addEventListener('mousedown', (e) => { this._onMouseDown(e); }, false);
    this.domElement.addEventListener('mouseup', (e) => { this._onMouseUp(e); }, false);
    this.domElement.addEventListener('wheel', (e) => { this._onWheel(e); }, false);
    this.domElement.addEventListener('contextmenu', (e) => { this._onContextMenu(e); }, false);
    if (typeof onready === 'function')
      onready();
  },

  sendMouseEvent: function(type, event, wheel) {
    var socket = this.stream.socket;
    if (!socket || socket.readyState !== 1)
      return;
    var modifier = (event.shiftKey ? 1 : 0) + (event.ctrlKey ? 2 : 0) + (event.altKey ? 4 : 0);
    socket.send('mouse ' + type + ' ' + event.button + ' ' + this.mouseEvents.mouseState.mouseDown + ' ' +
                event.offsetX + ' ' + event.offsetY + ' ' + modifier + ' ' + wheel);
  },

  resize: function(width, height) {
    this.domElement.width = width;
    this.domElement.height = height;
    this.stream.socket.send('resize: ' + width + 'x' + height);
  },

  _onMouseDown: function(event) {
    event.target.addEventListener('mousemove', this.onmousemove, false);
    this.sendMouseEvent(-1, event, 0);
    event.preventDefault();
    return false;
  },

  _onMouseMove: function(event) {
    if (this.mouseEvents.mouseState.mouseDown === 0) {
      event.target.removeEventListener('mousemove', this.onmousemove, false);
      return false;
    }
    this.sendMouseEvent(0, event, 0);
    return false;
  },

  _onMouseUp: function(event) {
    event.target.removeEventListener('mousemove', this.onmousemove, false);
    this.sendMouseEvent(1, event, 0);
    event.preventDefault();
    return false;
  },

  _onWheel: function(event) {
    this.sendMouseEvent(2, event, Math.sign(event.deltaY));
    return false;
  },

  _onContextMenu: function(event) {
    event.preventDefault();
    return false;
  }
};
