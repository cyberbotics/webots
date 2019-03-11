/* global THREE, Selector, Viewpoint */
/* global convertStringToVec2, convertStringToVec3, convertStringToQuaternion, convertStringTorgb */
'use strict';

class X3dSceneManager { // eslint-disable-line no-unused-vars
  constructor(domElement) {
    this.domElement = domElement;
    this.camera = null;
    this.root = null;
    this.worldInfo = {};
    this.viewpoint = null;
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

    this.viewpoint = new Viewpoint(this.camera);
    this.viewpoint.onCameraPositionChanged = () => {
      if (this.gpuPicker)
        this.gpuPicker.needUpdate = true;
    };

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

    this.domElement.appendChild(this.renderer.domElement);

    this.gpuPicker = new THREE.GPUPicker({renderer: this.renderer, debug: false});
    this.gpuPicker.setFilter(function(object) {
      return object instanceof THREE.Mesh && 'x3dType' in object.userData;
    });
    this.gpuPicker.setScene(this.scene);
    this.gpuPicker.setCamera(this.camera);

    window.onresize = () => this.resize(); // when the window has been resized.
    this.resize();

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
    this.gpuPicker.resizeTexture(width, height);
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
    if (!object)
      return undefined;

    if (object.name && object.name.includes(id))
      return object;

    var childObject;
    var childrenLength = object.children ? object.children.length : 0;
    for (var i = 0; i < childrenLength; i++) {
      var child = object.children[i];
      childObject = this.getObjectByCustomId(child, id);
      if (childObject !== undefined)
        return childObject;
    }
    if (object instanceof THREE.Mesh) {
      childObject = this.getObjectByCustomId(object.material, id);
      if (childObject)
        return childObject;
      childObject = this.getObjectByCustomId(object.geometry, id);
      if (childObject)
        return childObject;
    } else if (object instanceof THREE.Material) {
      childObject = this.getObjectByCustomId(object.map, id);
      if (childObject)
        return childObject;
      childObject = this.getObjectByCustomId(object.aoMap, id);
      if (childObject)
        return childObject;
      childObject = this.getObjectByCustomId(object.roughnessMap, id);
      if (childObject)
        return childObject;
      childObject = this.getObjectByCustomId(object.metalnessMap, id);
      if (childObject)
        return childObject;
      childObject = this.getObjectByCustomId(object.normalMap, id);
      if (childObject)
        return childObject;
      childObject = this.getObjectByCustomId(object.emissiveMap, id);
      if (childObject)
        return childObject;
      // only fields set in x3d.js are checked
    }
    return undefined;
  }

  applyPose(pose) {
    var id = 'n' + pose.id;
    for (var key in pose) {
      if (key === 'id')
        continue;
      var newValue = pose[key];
      var object = this.getObjectByCustomId(this.scene, id);
      if (!object)
        // error
        continue;

      if (key === 'translation') { // Transform node
        if (object instanceof THREE.Texture) {
          var translation = convertStringToVec2(newValue);
          object.offset.set(-translation.x, -translation.y);
          object.needsUpdate = true;
        } else {
          var newPosition = convertStringToVec3(newValue);
          /* TODO this is the old condition
          if (key === 'translation' && this.followedObject &&
             (id === this.followedObject || // animation case
              el.id === this.followedObject || // streaming case
              el.getAttribute('DEF') === this.followedObject)) { */
          // followed object moved.
          if (id === this.viewpoint.followedObject) {
            // If this is the followed object, we save a vector with the translation applied
            // to the object to compute the new position of the viewpoint.
            this.viewpoint.setFollowedObjectDeltaPosition(newPosition, object.position);
          }
          object.position.copy(newPosition);
        }
      } else if (key === 'rotation') { // Transform node
        var quaternion = convertStringToQuaternion(newValue);
        object.quaternion.copy(quaternion);
      } else if (key === 'diffuseColor' || key === 'baseColor') {
        var diffuseColor = convertStringTorgb(newValue);
        object.color = diffuseColor;
      } else if (key === 'emissiveColor') {
        var emissiveColor = convertStringTorgb(newValue);
        object.emissive = emissiveColor;
      }
    }
  }

  getObjectAt(relativePosition, screenPosition) {
    // if (this.handle.control.pointerHover(screenPosition))
    //  return;
    // this.handle.hideHandle();

    // make sure that scene is up to date
    // TODO: update the scene on change instead of on picking
    this.gpuPicker.setScene(this.scene);

    var raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(screenPosition, this.camera);
    var intersection = this.gpuPicker.pick(relativePosition, raycaster);
    var pickedObject = null;
    if (intersection && intersection.object)
      pickedObject = this.getTopX3dNode(intersection.object);
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
