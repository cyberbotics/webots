/* global THREE */
/* global convertStringToVec3, convertStringToQuaternion */
'use strict';

class View { // eslint-disable-line no-unused-vars
  constructor(parentElement) {
    this.domElement = document.createElement('x3d');
    this.domElement.className = 'webots3DView';
    parentElement.appendChild(this.domElement);
  }

  init() {
    this.renderer = new THREE.WebGLRenderer({'antialias': true});
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0xffffff, 1.0);

    this.scene = new THREE.Scene();

    // TODO remove and load from world file
    this.camera = new THREE.PerspectiveCamera(45, 0.3, 0.001, 400);
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

    window.onresize = () => this.resize(); // when the window has been resized.

    this.domElement.appendChild(this.renderer.domElement);
    this.resize();

    this.gpuPicker = new THREE.GPUPicker({renderer: this.renderer, debug: false});
    this.gpuPicker.setFilter(function(object) {
      return object instanceof THREE.Mesh && 'x3dType' in object.userData;
    });

    /* // how to set a textured background.
    var loader = new THREE.CubeTextureLoader();
    loader.setPath( '/robot-designer/assets/common/textures/cubic/' );
    this.scene.background = loader.load( [
      'noon_sunny_empty_right.jpg', 'noon_sunny_empty_left.jpg',
      'noon_sunny_empty_top.jpg', 'noon_sunny_empty_bottom.jpg',
      'noon_sunny_empty_front.jpg', 'noon_sunny_empty_back.jpg'
    ] );
    */

    this.initLight(); // TODO remove
    this.render();
  }

  initLight() {
    var light = new THREE.DirectionalLight(0xdfebff, 1);
    light.position.set(50, 200, 100);
    this.scene.add(light);
    var light2 = new THREE.AmbientLight(0x404040);
    this.scene.add(light2);

    var grid = new THREE.GridHelper(5, 50, 0x880088, 0x440044);
    grid.matrixAutoUpdate = false;
    this.scene.add(grid);
  }

  render() {
    requestAnimationFrame(() => this.render());
    this.composer.render();
  }

  resize() {
    // TODO
    var width = 800; //this.domElement.clientWidth;
    var height = 800;// this.domElement.clientHeight;
    this.renderer.setSize(width, height);
    this.composer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  initWorld() {

  }

  loadWorldFile(url) {
    var object = new THREE.Object3D();
    var loader = new THREE.X3DLoader(this.scene);
    loader.load(url, (object3d) => {
      if (object)
        object.add(object3d);
    });
    this.scene.add(object);
  }

  loadObject(x3dObject, parentId) {
    var loader = new THREE.X3DLoader(this.scene);
    var object3d = loader.parse(x3dObject);
    if (!parentId)
      this.scene.add(object3d);
    // TODO check for parent
    //document.getElementById('n' + parentId).appendChild(x3d);
  }

  destroyWorld() {
    while (this.scene.lastChild)
      this.scene.removeChild(this.scene.lastChild);
    this.initLight(); // TODO remove
    this.render();
  }

  destroyObject(id) {
    // TODO
    var itemToDelete = document.getElementById('n' + id);
    if (itemToDelete) {
      // TODO if (that.selection === itemToDelete)
      //  that.selection = null;
      itemToDelete.parentElement.removeChild(itemToDelete);
    }
    this.render();
  }

  applyPose(objectId, key, newValue) {
    var object = this.scene.getObjectByName('n' + objectId);
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
    } else
    */

    if (key === 'translation') {
      var position = convertStringToVec3(newValue);
      object.position.copy(position);
    } else if (key === 'rotation') {
      var quaternion = convertStringToQuaternion(newValue);
      object.quaternion.copy(quaternion);
    }
    // TODO extend with other animation attributes
  }
}
