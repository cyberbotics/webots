/* global THREE, Selector */
/* global convertStringToVec3, convertStringToQuaternion, convertStringTorgb */
'use strict';

class X3dScene { // eslint-disable-line no-unused-vars
  constructor(parentElement) {
    this.domElement = document.createElement('div');
    this.domElement.className = 'webots3DView';
    this.camera = null;
    this.root = null;
    parentElement.appendChild(this.domElement);
  }

  init() {
    this.renderer = new THREE.WebGLRenderer({'antialias': true});
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0xffffff, 1.0);

    this.scene = new THREE.Scene();

    // init default camera
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.001, 400);
    this.camera.position.x = 10;
    this.camera.position.y = 10;
    this.camera.position.z = 10;
    this.camera.lookAt(this.scene.position);

    this.controls = new THREE.OrbitControls(this.camera, this.robotViewerElement);

    this.composer = new THREE.EffectComposer(this.renderer);
    var renderPass = new THREE.RenderPass(this.scene, this.camera);
    this.composer.addPass(renderPass);
    this.highlightOutlinePass = new THREE.OutlinePass(new THREE.Vector2(window.innerWidth, window.innerHeight), this.scene, this.camera);
    this.composer.addPass(this.highlightOutlinePass);
    this.selectionOutlinePass = new THREE.OutlinePass(new THREE.Vector2(window.innerWidth, window.innerHeight), this.scene, this.camera);
    this.selectionOutlinePass.visibleEdgeColor.set(0x00BFFF);
    this.selectionOutlinePass.renderToScreen = true;
    this.composer.addPass(this.selectionOutlinePass);

    this.selector = new Selector(this.selectionOutlinePass);

    window.onresize = () => this.resize(); // when the window has been resized.

    this.domElement.appendChild(this.renderer.domElement);
    this.resize();

    this.gpuPicker = new THREE.GPUPicker({renderer: this.renderer, debug: false});
    this.gpuPicker.setScene(this.scene);
    this.gpuPicker.setCamera(this.camera);
    this.gpuPicker.setFilter(function(object) {
      return object instanceof THREE.Mesh && 'x3dType' in object.userData;
    });

    this.destroyWorld();
  }

  render() {
    requestAnimationFrame(() => this.render());
    this.composer.render();
  }

  resize() {
    var width = this.domElement.clientWidth;
    var height = this.domElement.clientHeight;
    this.renderer.setSize(width, height);
    this.composer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  destroyWorld() {
    if (!this.scene)
      return;
    for (var i = this.scene.children.length - 1; i >= 0; i--)
      this.scene.remove(this.scene.children[i]);
    this.render();
  }

  deleteObject(id) {
    var object = this.scene.getObjectByName('n' + id);
    if (object)
      object.parent.remove(object);
    this.render();
  }

  loadWorldFile(url) {
    var object = new THREE.Object3D();
    var loader = new THREE.X3DLoader(this);
    loader.load(url, (object3d) => {
      if (object)
        object.add(object3d);
    });
    this.scene.add(object);
  }

  loadObject(x3dObject, parentId) {
    var loader = new THREE.X3DLoader(this);
    var object = loader.parse(x3dObject);
    var parentObject = parentId && parentId !== 0 ? this.scene.getObjectByName('n' + parentId) : null;
    if (parentObject)
      parentObject.add(object);
    else
      this.scene.add(object);
  }

  getObjectByCustomId(object, id) {
    if (object.name && object.name.includes(id))
      return object;

    var childrenLength = object.children ? object.children.length : 0;
    for (var i = 0; i < childrenLength; i++) {
      var child = object.children[i];
      var childObject = this.getObjectByCustomId(child, id);
      if (childObject !== undefined)
        return childObject;
    }
    if (object instanceof THREE.Mesh) {
      if (this.getObjectByCustomId(object.material, id))
        return object.material;
      if (this.getObjectByCustomId(object.geometry, id))
        return object.geometry;
    }
    return undefined;
  }

  applyPose(pose) {
    var id = pose.id;
    // TODO if (el.getAttribute('blockWebotsUpdate')) return;
    for (var key in pose) {
      if (key === 'id')
        continue;
      var newValue = pose[key];
      var object = this.getObjectByCustomId(this.scene, 'n' + id);
      if (!object)
        // error
        continue;
      /* TODO update viewpoint
      if (key === 'translation' && this.followedObject &&
         (id === this.followedObject || // animation case
          el.id === this.followedObject || // streaming case
          el.getAttribute('DEF') === this.followedObject)) {
        var objectPosition = x3dom.fields.SFVec3f.parse(el.getAttribute('translation'));
        el.setAttribute(key, value);
        // If this is the followed object, we save a vector with the translation applied
        // to the object to compute the new position of the viewpoint.
        var objectNewPosition = x3dom.fields.SFVec3f.parse(value);
        this.followedObjectDeltaPosition = objectNewPosition.subtract(objectPosition);
      }
      */

      if (key === 'translation') { // Transform node
        var position = convertStringToVec3(newValue);
        object.position.copy(position);
      } else if (key === 'rotation') { // Transform node
        var quaternion = convertStringToQuaternion(newValue);
        object.quaternion.copy(quaternion);
      } else if (key === 'diffuseColor') {
        var diffuseColor = convertStringTorgb(newValue);
        object.color = diffuseColor;
      } else if (key === 'emissiveColor') {
        var emissiveColor = convertStringTorgb(newValue);
        object.emissive = emissiveColor;
      }
      // TODO extend with other animation attributes
      // texture transform: translation
    }
  }

  getObjectAt(relativePosition, screenPosition) {
    // if (this.handle.control.pointerHover(screenPosition))
    //  return;
    // this.handle.hideHandle();
    // TODO are really needed?
    // gpuPicker.setScene(this.scene);
    // gpuPicker.setCamera(this.camera);

    var raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(screenPosition, this.camera);
    var intersection = raycaster.intersectObject(this.scene.root, true); // this.gpuPicker.pick(relativePosition, raycaster);
    var pickedObject = null;
    if (intersection && intersection.length > 0)
      pickedObject = this.getTopX3dNode(intersection[0].object);
    return pickedObject;
    /* if (intersection && intersection.faceIndex > 0) {
      var parent = intersection.object;
      do {
        if (parent.userData.isPartContainer) {
          this.handle.showHandle();
          return parent;
        }
        parent = parent.parent;
      } while (parent);
    }
    this.handle.showHandle(); */
  }

  getTopX3dNode(node) {
    // If it exists, return the upmost Solid, otherwise the top node
    var upmostSolid = null;
    while (node) {
      if (node.userData && node.userData.solid)
        upmostSolid = node;
      if (node.parent === this.scene)
        break;
      node = node.parent;
    }
    if (upmostSolid)
      return upmostSolid;
    return node;
  }
}
