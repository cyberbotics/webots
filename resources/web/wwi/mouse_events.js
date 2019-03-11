/* global THREE */
'use strict';

function MouseEvents(sceneManager) {
  this.sceneManager = sceneManager;
};

MouseEvents.prototype = {
  constructor: MouseEvents,

  mousedown: function(event) {
    var relativePosition = MouseEvents.convertMouseEventPositionToRelativePosition(this.sceneManager.renderer, event.clientX, event.clientY);
    var screenPosition = MouseEvents.convertMouseEventPositionToScreenPosition(this.sceneManager.renderer, event.clientX, event.clientY);
    var pickedObject = this.sceneManager.getObjectAt(relativePosition, screenPosition);
    if (pickedObject)
      this.sceneManager.selector.selectObject(pickedObject);
  }
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
