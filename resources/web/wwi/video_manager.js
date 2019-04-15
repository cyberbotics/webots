'use strict';

function VideoManager(parentObject, mouseEvents, stream) {
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
};

VideoManager.prototype = {
  constructor: VideoManager,

  finalize: function(onready) {
    this.domElement.addEventListener('mousedown', this.onVideoMouseDown, false);
    this.domElement.addEventListener('mouseup', this.onVideoMouseUp, false);
    this.domElement.addEventListener('wheel', this.onVideoWheel, false);
    this.domElement.addEventListener('contextmenu', this.onVideoContextMenu, false);
    if (typeof onready === 'function')
      onready();
  },

  sendVideoMouseEvent: function(type, event, wheel) {
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

  onVideoMouseDown: function(event) {
    event.target.addEventListener('mousemove', this.onVideoMouseMove, false);
    this.sendVideoMouseEvent(-1, event, 0);
    event.preventDefault();
    return false;
  },

  onVideoMouseMove: function(event) {
    if (this.mouseEvents.mouseState.mouseDown === 0) {
      event.target.removeEventListener('mousemove', this.onVideoMouseMove, false);
      return false;
    }
    this.sendVideoMouseEvent(0, event, 0);
    return false;
  },

  onVideoMouseUp: function(event) {
    event.target.removeEventListener('mousemove', this.onVideoMouseMove, false);
    this.sendVideoMouseEvent(1, event, 0);
    event.preventDefault();
    return false;
  },

  onVideoWheel: function(event) {
    this.sendVideoMouseEvent(2, event, Math.sign(event.deltaY));
    return false;
  },

  onVideoContextMenu: function(event) {
    event.preventDefault();
    return false;
  }
};
