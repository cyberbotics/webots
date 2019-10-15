/* global THREE, Selector, TextureLoader, Viewpoint */
/* global convertStringToVec2, convertStringToVec3, convertStringToQuaternion, convertStringToColor, horizontalToVerticalFieldOfView */
/* global createDefaultGeometry, createDefaultMaterial */
'use strict';

class X3dScene { // eslint-disable-line no-unused-vars
  constructor(domElement) {
    this.domElement = domElement;
    this.root = undefined;
    this.worldInfo = {};
    this.viewpoint = undefined;
    this.sceneModified = false;
    this.useNodeCache = {};
    this.objectsIdCache = {};
  }

  init(texturePathPrefix = '') {
    this.renderer = new THREE.WebGLRenderer({'antialias': false});
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0xffffff, 1.0);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    this.renderer.gammaInput = false;
    this.renderer.gammaOutput = false;
    this.renderer.physicallyCorrectLights = true;
    this.domElement.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.renderAllAtLoad = false;

    this.viewpoint = new Viewpoint();
    this.viewpoint.onCameraParametersChanged = (updateScene) => {
      if (this.gpuPicker)
        this.gpuPicker.needUpdate = true;
      if (updateScene)
        this.render();
    };

    this.selector = new Selector();
    this.selector.onSelectionChange = () => { this.render(); };

    this.gpuPicker = new THREE.GPUPicker({renderer: this.renderer, debug: false});
    this.gpuPicker.setFilter((object) => {
      return object.isMesh &&
             'x3dType' in object.userData &&
             object.userData.isPickable !== false; // true or undefined
    });
    this.gpuPicker.setScene(this.scene);
    this.gpuPicker.setCamera(this.viewpoint.camera);

    // add antialiasing post-processing effects
    this.composer = new THREE.EffectComposer(this.renderer);
    let renderPass = new THREE.RenderPass(this.scene, this.viewpoint.camera);
    this.composer.addPass(renderPass);
    this.bloomPass = new THREE.Bloom(new THREE.Vector2(window.innerWidth, window.innerHeight));
    this.composer.addPass(this.bloomPass);
    this.hdrResolvePass = new THREE.ShaderPass(THREE.HDRResolveShader);
    this.composer.addPass(this.hdrResolvePass);
    var fxaaPass = new THREE.ShaderPass(THREE.FXAAShader);
    this.composer.addPass(fxaaPass);

    this.resize();

    this.destroyWorld();

    TextureLoader.setTexturePathPrefix(texturePathPrefix);
    TextureLoader.setOnTextureLoad(() => {
      if (this.renderAllAtLoad && !TextureLoader.hasPendingData()) {
        this.renderAllAtLoad = false;
        this.scene.traverse((object) => { object.frustumCulled = true; });
      }
      this.render();
    });
  }

  render() {
    // Apply pass uniforms.
    this.hdrResolvePass.material.uniforms['exposure'].value = 2.0 * this.viewpoint.camera.userData.exposure; // Factor empirically found to match the Webots rendering.
    this.bloomPass.threshold = this.viewpoint.camera.userData.bloomThreshold;
    this.bloomPass.enabled = this.bloomPass.threshold >= 0;

    if (typeof this.preRender === 'function')
      this.preRender(this.scene, this.viewpoint.camera);
    this.composer.render();
    if (typeof this.postRender === 'function')
      this.postRender(this.scene, this.viewpoint.camera);
  }

  resize() {
    var width = this.domElement.clientWidth;
    var height = this.domElement.clientHeight;
    this.viewpoint.camera.aspect = width / height;
    if (this.viewpoint.camera.fovX)
      this.viewpoint.camera.fov = THREE.Math.radToDeg(horizontalToVerticalFieldOfView(this.viewpoint.camera.fovX, this.viewpoint.camera.aspect));
    this.viewpoint.camera.updateProjectionMatrix();
    this.gpuPicker.resizeTexture(width, height);
    this.renderer.setSize(width, height);
    this.composer.setSize(width, height);
    this.render();
  }

  onSceneUpdate() {
    this.sceneModified = true;
    this.render();
  }

  destroyWorld() {
    this.selector.clearSelection();
    if (!this.scene)
      return;
    for (let i = this.scene.children.length - 1; i >= 0; i--)
      this.scene.remove(this.scene.children[i]);
    this.objectsIdCache = {};
    this.useNodeCache = {};
    this.root = undefined;
    this.scene.background = new THREE.Color(0, 0, 0);

    /*
    // Code to debug bloom passes.
    var geometry = new THREE.PlaneGeometry(5, 5);
    var material = new THREE.MeshStandardMaterial({color: 0xffffff, side: THREE.DoubleSide});
    this.bloomPass.debugMaterial = material;
    var plane = new THREE.Mesh(geometry, material);
    this.scene.add(plane);
    */

    this.onSceneUpdate();
    this.render();
  }

  deleteObject(id) {
    var context = {};
    var object = this.getObjectById('n' + id, false, 'scene', context);
    if (typeof object !== 'undefined') {
      var parent;
      if (typeof context !== 'undefined' && typeof context.field !== 'undefined') {
        parent = context.parent;
        if (object.isMaterial)
          parent[context.field] = createDefaultMaterial(parent.geometry);
        else if (object.isGeometry || object.isBufferGeometry)
          parent[context.field] = createDefaultGeometry();
        else
          parent[context.field] = undefined;
      } else {
        parent = object.parent;
        object.parent.remove(object);
      }
      delete this.objectsIdCache[id];
      if (typeof parent !== 'undefined')
        this._updateUseNodesIfNeeded(parent, parent.name.split(';'));
    }
    if (object === this.root)
      this.root = undefined;
    this.onSceneUpdate();
    this.render();
  }

  loadWorldFile(url, onLoad) {
    this.objectsIdCache = {};
    var loader = new THREE.X3DLoader(this);
    loader.load(url, (object3d) => {
      if (object3d.length > 0) {
        this.scene.add(object3d[0]);
        this.root = object3d[0];
      }
      this._setupLights(loader.directionalLights);
      this._setupEnvironmentMap();
      if (this.gpuPicker) {
        this.gpuPicker.setScene(this.scene);
        this.sceneModified = false;
      }

      // Render all the objects at scene load.
      // The frustumCulled parameter will be set back to TRUE once all the textures are loaded.
      this.scene.traverse((o) => {
        o.frustumCulled = false;
      });
      this.renderAllAtLoad = true;

      this.onSceneUpdate();
      if (typeof onLoad === 'function')
        onLoad();
    });
  }

  loadObject(x3dObject, parentId) {
    var parentObject;
    if (parentId && parentId !== 0)
      parentObject = this.getObjectById('n' + parentId);
    var loader = new THREE.X3DLoader(this);
    var objects = loader.parse(x3dObject, parentObject);
    if (typeof parentObject !== 'undefined')
      this._updateUseNodesIfNeeded(parentObject, parentObject.name.split(';'));
    else {
      console.assert(objects.length <= 1 && typeof this.root === 'undefined'); // only one root object is supported
      objects.forEach((o) => { this.scene.add(o); });
      this.root = objects[0];
    }
    this._setupLights(loader.directionalLights);
    this._setupEnvironmentMap();
    if (typeof parentObject === 'undefined') {
      // Render all the objects at scene load.
      // The frustumCulled parameter will be set back to TRUE once all the textures are loaded.
      this.scene.traverse((o) => {
        o.frustumCulled = false;
      });
      this.renderAllAtLoad = true;
    }
    this.onSceneUpdate();
  }

  applyPose(pose, appliedFields = []) {
    var id = pose.id;
    var fields = appliedFields;
    for (let key in pose) {
      if (key === 'id')
        continue;
      if (fields.indexOf(key) !== -1)
        continue;
      var newValue = pose[key];
      var object = this.getObjectById('n' + id, true);
      if (typeof object === 'undefined')
        continue; // error

      var valid = true;
      if (key === 'translation') {
        if (object.isTexture) {
          var translation = convertStringToVec2(newValue);
          if (object.userData && object.userData.transform) {
            object.userData.transform.translation = translation;
            object.needsUpdate = true;
            this.sceneModified = true;
          }
        } else if (object.isObject3D) {
          var newPosition = convertStringToVec3(newValue);
          // Followed object moved.
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
      } else if (key === 'rotation' && object.isObject3D) { // Transform node
        var quaternion = convertStringToQuaternion(newValue);
        object.quaternion.copy(quaternion);
        this.sceneModified = true;
      } else if (object.isMaterial) {
        if (key === 'baseColor')
          object.color = convertStringToColor(newValue); // PBRAppearance node
        else if (key === 'diffuseColor')
          object.color = convertStringToColor(newValue, false); // Appearance node
        else if (key === 'emissiveColor')
          object.emissive = convertStringToColor(newValue, object.userData.x3dType === 'PBRAppearance');
      } else if (key === 'render' && object.isObject3D)
        object.visible = newValue.toLowerCase() === 'true';
      else
        valid = false;

      if (valid)
        fields.push(key);

      this._updateUseNodesIfNeeded(object, id);
    }
    return fields;
  }

  pick(relativePosition, screenPosition) {
    if (this.sceneModified) {
      this.gpuPicker.setScene(this.scene);
      this.sceneModified = false;
    }

    var raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(screenPosition, this.viewpoint.camera);
    return this.gpuPicker.pick(relativePosition, raycaster);
  }

  getCamera() {
    return this.viewpoint.camera;
  }

  getTopX3dNode(node) {
    // If it exists, return the upmost Solid, otherwise the top node.
    var upmostSolid;
    while (node) {
      if (node.userData && node.userData.solid)
        upmostSolid = node;
      if (node.parent === this.scene)
        break;
      node = node.parent;
    }
    if (typeof upmostSolid !== 'undefined')
      return upmostSolid;
    return node;
  }

  getObjectById(id, skipBoundingObject = false, object = 'scene', context = {}) {
    // @param 'object':
    //     Global case: object is the root object in which to search for.
    //     Special case to have a good default value: if object === 'scene', then the scene is used.
    if (object === 'scene')
      object = this.scene;

    if (!object ||
        (skipBoundingObject && typeof object.userData !== 'undefined' && object.userData.x3dType === 'Switch')) {
      context.parent = undefined;
      return undefined;
    }

    if (Array.isArray(object)) {
      for (let i = 0, l = object.length; i < l; i++) {
        var o = this.getObjectById(id, skipBoundingObject, object[i], context);
        if (typeof o !== 'undefined')
          return o;
      }
    }

    if (typeof this.objectsIdCache[id] !== 'undefined') {
      context.field = this.objectsIdCache[id].context.field;
      context.parent = this.objectsIdCache[id].context.parent;
      return this.objectsIdCache[id].object;
    }

    if (object.name && object.name.includes(id)) {
      this.objectsIdCache[id] = { 'object': object, 'context': context };
      return object;
    }

    var childObject;
    if (object.children) {
      for (let childIndex in object.children) {
        context.parent = object;
        childObject = this.getObjectById(id, skipBoundingObject, object.children[childIndex], context);
        if (typeof childObject !== 'undefined')
          return childObject;
      };
    }
    if (object.isMesh || object.isLineSegments || object.isPoint) {
      if (object.material) {
        childObject = this.getObjectById(id, skipBoundingObject, object.material, context);
        if (typeof childObject !== 'undefined') {
          context.field = 'material';
          context.parent = object;
          return childObject;
        }
      }
      if (object.geometry) {
        childObject = this.getObjectById(id, skipBoundingObject, object.geometry, context);
        if (typeof childObject !== 'undefined') {
          context.field = 'geometry';
          context.parent = object;
          return childObject;
        }
      }
    } else if (object.isMaterial) {
      if (object.map) {
        childObject = this.getObjectById(id, skipBoundingObject, object.map, context);
        if (typeof childObject !== 'undefined') {
          context.field = 'map';
          context.parent = object;
          return childObject;
        }
      }
      if (object.aoMap) {
        childObject = this.getObjectById(id, skipBoundingObject, object.aoMap, context);
        if (typeof childObject !== 'undefined') {
          context.field = 'aoMap';
          context.parent = object;
          return childObject;
        }
      }
      if (object.roughnessMap) {
        childObject = this.getObjectById(id, skipBoundingObject, object.roughnessMap, context);
        if (typeof childObject !== 'undefined') {
          context.field = 'roughnessMap';
          context.parent = object;
          return childObject;
        }
      }
      if (object.metalnessMap) {
        childObject = this.getObjectById(id, skipBoundingObject, object.metalnessMap, context);
        if (typeof childObject !== 'undefined') {
          context.field = 'metalnessMap';
          context.parent = object;
          return childObject;
        }
      }
      if (object.normalMap) {
        childObject = this.getObjectById(id, skipBoundingObject, object.normalMap, context);
        if (typeof childObject !== 'undefined') {
          context.field = 'normalMap';
          context.parent = object;
          return childObject;
        }
      }
      if (object.emissiveMap) {
        childObject = this.getObjectById(id, skipBoundingObject, object.emissiveMap, context);
        if (typeof childObject !== 'undefined') {
          context.field = 'emissiveMap';
          context.parent = object;
          return childObject;
        }
      }
      // only fields set in x3d.js are checked
    }
    return undefined;
  }

  // private functions
  _setupLights(directionalLights) {
    if (!this.root)
      return;

    var sceneBox = new THREE.Box3();
    sceneBox.setFromObject(this.root);
    var boxSize = new THREE.Vector3();
    sceneBox.getSize(boxSize);
    var boxCenter = new THREE.Vector3();
    sceneBox.getCenter(boxCenter);
    var halfWidth = boxSize.x / 2 + boxCenter.x;
    var halfDepth = boxSize.z / 2 + boxCenter.z;
    var maxSize = 2 * Math.max(halfWidth, boxSize.y / 2 + boxCenter.y, halfDepth);
    directionalLights.forEach((light) => {
      light.position.multiplyScalar(maxSize);
      light.shadow.camera.far = Math.max(maxSize, light.shadow.camera.far);
      light.shadow.camera.left = -maxSize;
      light.shadow.camera.right = maxSize;
      light.shadow.camera.top = maxSize;
      light.shadow.camera.bottom = -maxSize;
    });
  }

  _setupEnvironmentMap() {
    var isHDR = false;
    var backgroundMap;
    if (this.scene.background) {
      if (typeof this.scene.background.userData !== 'undefined' && this.scene.background.userData.isHDR) {
        isHDR = true;
        backgroundMap = this.scene.background.userData.texture;
      } else if (this.scene.background.isCubeTexture)
        backgroundMap = this.scene.background;
      else if (this.scene.background.isColor) {
        let color = this.scene.background.clone();
        color.convertLinearToSRGB();
        backgroundMap = TextureLoader.createColoredCubeTexture(color);
      }
    }

    this.scene.traverse((child) => {
      if (child.isMesh && child.material && child.material.isMeshStandardMaterial) {
        var material = child.material;
        material.envMap = backgroundMap;
        material.envMapIntensity = isHDR ? 0.6 : 1.0; // Factor empirically found to match the Webots rendering.
        if (typeof this.scene.userData.luminosity !== 'undefined')
          material.envMapIntensity *= this.scene.userData.luminosity;
        material.needsUpdate = true;
      }
    });
  }

  _updateUseNodesIfNeeded(object, id) {
    if (!object)
      return;

    if (Array.isArray(id)) {
      if (id.length > 1)
        id.forEach((item) => this._updateUseNodesIfNeeded(object, item));
      else
        id = id[0];
    }

    if (typeof this.useNodeCache[id] === 'undefined') {
      var node = object;
      var source;
      while (node && node !== this.root) {
        if (typeof node.userData.USE !== 'undefined')
          source = node;
        node = node.parent;
      }
      this.useNodeCache[id] = { 'source': source };
      if (typeof source !== 'undefined') {
        this.useNodeCache[id].target = [];
        source.userData.USE.split(';').forEach((useId) => {
          var useObject = this.getObjectById(useId);
          if (typeof useObject !== 'undefined')
            this.useNodeCache[id].target.push(useObject);
        });
      }
    }
    if (typeof this.useNodeCache[id].source !== 'undefined') {
      // clone again changed DEF node instance
      var sourceNode = this.useNodeCache[id].source;
      var targetNodes = this.useNodeCache[id].target;
      var newTargetNodes = [];
      for (let i = 0, l = targetNodes.length; i < l; i++) {
        var target = targetNodes[i];
        var newClone = sourceNode.clone();
        newClone.name = target.name;
        var parent = target.parent;
        var index = parent.children.indexOf(target);
        parent.remove(target);

        // manually add new child to keep the same child index.
        parent.children.splice(index, 0, newClone);
        newClone.parent = parent;
        object.dispatchEvent({ type: 'added' });

        newTargetNodes.push(newClone);
      }
      this.useNodeCache[id].target = newTargetNodes;
    }
  }
}
