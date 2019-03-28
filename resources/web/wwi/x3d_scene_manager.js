/* global THREE, Selector, Viewpoint */
/* global convertStringToVec2, convertStringToVec3, convertStringToQuaternion, convertStringTorgb */
'use strict';

function X3dSceneManager(domElement) {
  this.domElement = domElement;
  this.camera = null;
  this.root = null;
  this.worldInfo = {};
  this.viewpoint = null;
  this.lights = [];
  this.sceneModified = false;
}

X3dSceneManager.prototype = {
  constructor: X3dSceneManager,

  init: function() {
    var that = this;
    this.renderer = new THREE.WebGLRenderer({'antialias': true});
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0xffffff, 1.0);
    this.renderer.shadowMap.enabled = true;
    this.renderer.gammaInput = true;
    this.renderer.gammaOutput = true;
    this.domElement.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.ambientLight = new THREE.AmbientLight(0x666666, 0.3);
    this.scene.add(this.ambientLight);

    // init default camera
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.001, 400);
    this.camera.position.x = 10;
    this.camera.position.y = 10;
    this.camera.position.z = 10;
    this.camera.lookAt(this.scene.position);

    this.viewpoint = new Viewpoint(this.camera);
    this.viewpoint.onCameraPositionChanged = function() {
      if (that.gpuPicker)
        that.gpuPicker.needUpdate = true;
    };

    this.selector = new Selector();

    this.gpuPicker = new THREE.GPUPicker({renderer: this.renderer, debug: false});
    this.gpuPicker.setFilter(function(object) {
      return object instanceof THREE.Mesh &&
             'x3dType' in object.userData &&
             object.userData.isPickable !== false; // true or undefined
    });
    this.gpuPicker.setScene(this.scene);
    this.gpuPicker.setCamera(this.camera);

    window.onresize = function() { that.resize(); }; // when the window has been resized.
    this.resize();

    this.destroyWorld();
  },

  onSceneUpdateCompleted: function(force = false) {
    if (this.gpuPicker && (force || this.sceneModified)) {
      this.gpuPicker.setScene(this.scene);
      this.sceneModified = false;
    }
  },

  render: function() {
    var that = this;
    requestAnimationFrame(function() { that.render(); });
    this.renderer.render(this.scene, this.camera);
  },

  resize: function() {
    var width = this.domElement.clientWidth;
    var height = this.domElement.clientHeight;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.gpuPicker.resizeTexture(width, height);
    this.renderer.setSize(width, height);
  },

  destroyWorld: function() {
    this.selector.clearSelection();
    if (!this.scene)
      return;
    for (var i = this.scene.children.length - 1; i >= 0; i--) {
      if (this.scene.children[i].isAmbientLight)
        continue;
      this.scene.remove(this.scene.children[i]);
    }
    this.onSceneUpdateCompleted(true);
    this.render();
  },

  deleteObject: function(id) {
    var object = this.scene.getObjectByName('n' + id);
    if (object)
      object.parent.remove(object);
    this.onSceneUpdateCompleted(true);
    this.render();
  },

  setupLights: function(lights) {
    if (!this.root || lights.length === 0)
      return;

    var that = this;
    var sceneBox = new THREE.Box3();
    sceneBox.setFromObject(this.root);
    var boxSize = new THREE.Vector3();
    sceneBox.getSize(boxSize);
    var boxCenter = new THREE.Vector3();
    sceneBox.getCenter(boxCenter);
    var maxSize = Math.max(boxSize.x + boxCenter.x, boxSize.y + boxCenter.y, boxSize.z + boxCenter.z);
    lights.forEach(function(light) {
      if (light.isDirectionalLight) {
        light.position.multiplyScalar(maxSize);
        if (light.castShadow)
          light.shadow.camera.far = maxSize * 2;
      }
      light.intensity = light.intensity;
      if (that.lights.indexOf(light) === -1)
        that.lights.push(light);
    });

    // Compute ambient light color.
    var r = 0;
    var g = 0;
    var b = 0;
    that.lights.forEach(function(light) {
      var i = light.userData.ambientIntensity;
      if (i && i > 0) {
        r += light.color.r * i;
        g += light.color.g * i;
        b += light.color.b * i;
      }
    });
    this.ambientLight.color = new THREE.Color(r, g, b);
  },

  loadWorldFile: function(url) {
    var that = this;
    var loader = new THREE.X3DLoader(this);
    loader.load(url, function(object3d) {
      if (object3d.length > 0) {
        that.scene.add(object3d[0]);
        that.root = object3d[0];
      }
    });
    this.setupLights(loader.lights);
    this.onSceneUpdateCompleted(true);
  },

  loadObject: function(x3dObject, parentId) {
    var that = this;
    var parentObject = parentId && parentId !== 0 ? this.scene.getObjectByName('n' + parentId) : null;
    var loader = new THREE.X3DLoader(this);
    var objects = loader.parse(x3dObject);
    if (parentObject)
      objects.forEach(function(o) { parentObject.add(o); });
    else {
      console.assert(objects.length <= 1); // only one root object is supported
      objects.forEach(function(o) { that.scene.add(o); });
      this.root = objects[0];
    }
    this.setupLights(loader.lights);
    this.onSceneUpdateCompleted(true);
  },

  getObjectByCustomId: function(object, id) {
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
  },

  applyPose: function(pose) {
    var id = pose.id;
    for (var key in pose) {
      if (key === 'id')
        continue;
      var newValue = pose[key];
      var object = this.getObjectByCustomId(this.scene, 'n' + id);
      if (!object)
        // error
        continue;

      if (key === 'translation') {
        if (object instanceof THREE.Texture) {
          var translation = convertStringToVec2(newValue);
          if (object.userData && object.userData.transform) {
            object.userData.transform.translation = translation;
            object.needsUpdate = true;
            this.sceneModified = true;
          }
        } else if (object instanceof THREE.Object3D) {
          var newPosition = convertStringToVec3(newValue);
          // followed object moved.
          if (this.viewpoint.followedObjectId &&
              (id === this.viewpoint.followedObjectId || // animation case
               ('n' + id) === this.viewpoint.followedObjectId || // streaming case
               object.userData.name === this.viewpoint.followedObjectId)) {
            // If this is the followed object, we save a vector with the translation applied
            // to the object to compute the new position of the viewpoint.
            this.viewpoint.setFollowedObjectDeltaPosition(newPosition, object.position);
          }
          object.position.copy(newPosition);
          this.sceneModified = true;
        }
      } else if (key === 'rotation' && object instanceof THREE.Object3D) { // Transform node
        var quaternion = convertStringToQuaternion(newValue);
        object.quaternion.copy(quaternion);
        this.sceneModified = true;
      } else if ((key === 'diffuseColor' || key === 'baseColor') && object instanceof THREE.Material) {
        var diffuseColor = convertStringTorgb(newValue);
        object.color = diffuseColor;
      } else if (key === 'emissiveColor' && object instanceof THREE.Material) {
        var emissiveColor = convertStringTorgb(newValue);
        object.emissive = emissiveColor;
      }
    }
  },

  pick: function(relativePosition, screenPosition) {
    var raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(screenPosition, this.camera);
    return this.gpuPicker.pick(relativePosition, raycaster);
  },

  getTopX3dNode: function(node) {
    // If it exists, return the upmost Solid, otherwise the top node.
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
};
