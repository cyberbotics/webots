'use strict';

function Selector() {
  this.selectedObject = null;
  this.selectedRepresentations = [];
};

Selector.prototype = {
  constructor: Selector,

  select: function(object) {
    if (this.selectedObject === object)
      return;

    this.clearSelection();
    var children = [object];
    while (children.length > 0) {
      var child = children.pop();
      if (child.userData && child.userData.x3dType === 'Switch') {
        child.visible = true;
        this.selectedRepresentations.push(child);
      }
      if (child.children)
        children = children.concat(child.children);
    }
    if (this.selectedRepresentations.length > 0)
      this.selectedObject = object;
  },

  clearSelection: function() {
    for (var i = 0; i < this.selectedRepresentations.length; i++)
      this.selectedRepresentations[i].visible = false;
    this.selectedRepresentations = [];
    this.selectedObject = null;
  }
};
