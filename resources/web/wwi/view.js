/* global THREE */
'use strict';

class View { // eslint-disable-line no-unused-vars
  constructor(viewElement) {
    this.element = viewElement;
  }

  init() {
    this.renderer = new THREE.WebGLRenderer({'antialias': true});
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0xffffff, 1.0);

    this.scene = new THREE.Scene();

    this.camera = new THREE.PerspectiveCamera(45, 0.3, 0.001, 100);
    this.camera.position.x = 0.1;
    this.camera.position.y = 0.1;
    this.camera.position.z = 0.1;
    this.camera.lookAt(this.scene.position);

    var light = new THREE.DirectionalLight(0xffffff, 0.5);
    this.scene.add(light);
    var light2 = new THREE.AmbientLight(0x404040);
    this.scene.add(light2);

    var grid = new THREE.GridHelper(5, 50, 0x880088, 0x440044);
    grid.matrixAutoUpdate = false;
    this.scene.add(grid);

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

    this.element.appendChild(this.renderer.domElement);
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

    this.render();
  }

  render() {
    requestAnimationFrame(() => this.render());
    this.composer.render();
  }

  resize() {
    var width = this.element.clientWidth;
    var height = 800;//this.element.clientHeight;
    this.renderer.setSize(width, height);
    this.composer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  initWorld() {

  }

  loadScene(url) {
    var object = new THREE.Object3D();
    var loader = new THREE.X3DLoader(this.scene);
    loader.load(url, (object3d) => {
      if (object)
        object.add(object3d);
    });
    this.scene.add(object);
  }

  loadObject(x3dObject) {
    console.log(x3dObject);
    var loader = new THREE.X3DLoader(this.scene);
    var object3d = loader.parse(x3dObject);
    this.scene.add(object3d);
  }
}
