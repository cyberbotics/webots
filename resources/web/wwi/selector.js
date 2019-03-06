/* global THREE, Observable */
'use strict';

class Selector extends Observable { // eslint-disable-line no-unused-vars
  constructor(outlinePass) {
    super();
    this.outlinePass = outlinePass;
    this.selectedObject = null;
    this.selectedRepresentations = [];
  }

  selectObject(object) {
    //if (this.selectedObject === object)
    //  return;

    this.clearSelection();
    object.children.forEach((child) => {
      if (child instanceof THREE.Mesh) {
        var geo = new THREE.EdgesGeometry(child.geometry);
        var mat = new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 2 });
        var wireframe = new THREE.LineSegments(geo, mat);
        wireframe.userData.selectionRepresentation = true;
        child.add(wireframe);
        this.selectedRepresentations.push(child);
      }
    });
    if (this.selectedRepresentations.length > 0)
      // Use OutlinePass instead of adding wireframe geometry
      // this.outlinePass.selectedObjects = selectedRepresentations;
      // this.notify('SelectionChanged', {'part': this.selectedObject});
      this.selectedObject = object;
  }

  clearSelection() {
    for (var i = 0; i < this.selectedRepresentations.length; i++) {
      var mesh = this.selectedRepresentations[i];
      for (var c = 0; c < mesh.children.length; c++) {
        var child = mesh.children[c];
        if (child.userData.selectionRepresentation)
          mesh.remove(child);
      }
    }
    this.selectedRepresentations = [];
    this.selectedObject = null;
    // TODO this.outlinePass.selectedObjects = [];
    this.notify('SelectionChanged', {'part': null});
  }
}
