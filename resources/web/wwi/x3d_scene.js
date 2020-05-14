/* global webots, THREE, Selector, TextureLoader, Viewpoint */
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

    // The Mozilla WebGL implementation does not support automatic mipmaps generation on float32 cube textures.
    // - Warning (JS console): "Texture at base level is not unsized internal format or is not color-renderable or texture-filterable."
    // - The related OpenGL specification is known as cryptic about this topic:
    //   - https://www.khronos.org/registry/OpenGL-Refpages/es3/html/glGenerateMipmap.xhtml
    //   - https://stackoverflow.com/questions/44754479/issue-with-rgba32f-texture-format-and-mipmapping-using-opengl-es-3-0
    //   - https://stackoverflow.com/questions/56829454/unable-to-generate-mipmap-for-half-float-texture
    const gl = document.createElement('canvas').getContext('webgl');
    const glVendor = gl.getParameter(gl.VENDOR);
    this.enableHDRReflections = glVendor !== 'Mozilla';
    if (!this.enableHDRReflections)
      console.warn('HDR reflections are not implemented for the current hardware.');
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
    let fxaaPass = new THREE.ShaderPass(THREE.FXAAShader);
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
    // Set maximum rendering frequency.
    // To avoid slowing down the simulation rendering the scene too often, the last rendering time is checked
    // and the rendering is performed only at a given maximum frequency.
    // To be sure that no rendering request is lost, a timeout is set.
    const renderingMinTimeStep = 40; // Rendering maximum frequency: every 40 ms.
    let currentTime = (new Date()).getTime();
    if (this.nextRenderingTime && this.nextRenderingTime > currentTime) {
      if (!this.renderingTimeout)
        this.renderingTimeout = setTimeout(() => this.render(), this.nextRenderingTime - currentTime);
      return;
    }

    // Apply pass uniforms.
    this.hdrResolvePass.material.uniforms['exposure'].value = 2.0 * this.viewpoint.camera.userData.exposure; // Factor empirically found to match the Webots rendering.
    this.bloomPass.threshold = this.viewpoint.camera.userData.bloomThreshold;
    this.bloomPass.enabled = this.bloomPass.threshold >= 0;

    if (typeof this.preRender === 'function')
      this.preRender(this.scene, this.viewpoint.camera);
    this.composer.render();
    if (typeof this.postRender === 'function')
      this.postRender(this.scene, this.viewpoint.camera);

    this.nextRenderingTime = (new Date()).getTime() + renderingMinTimeStep;
    clearTimeout(this.renderingTimeout);
    this.renderingTimeout = null;
  }

  resize() {
    let width = this.domElement.clientWidth;
    let height = this.domElement.clientHeight;
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
    let geometry = new THREE.PlaneGeometry(5, 5);
    let material = new THREE.MeshStandardMaterial({color: 0xffffff, side: THREE.DoubleSide});
    this.bloomPass.debugMaterial = material;
    let plane = new THREE.Mesh(geometry, material);
    this.scene.add(plane);
    */

    this.onSceneUpdate();
  }

  deleteObject(id) {
    let context = {};
    let object = this.getObjectById('n' + id, false, 'scene', context);
    if (typeof object !== 'undefined') {
      let parent;
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
  }

  loadWorldFile(url, onLoad) {
    this.objectsIdCache = {};
    let loader = new THREE.X3DLoader(this);
    loader.enableHDRReflections = this.enableHDRReflections;
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
    let parentObject;
    if (parentId && parentId !== 0)
      parentObject = this.getObjectById('n' + parentId);
    let loader = new THREE.X3DLoader(this);
    loader.enableHDRReflections = this.enableHDRReflections;
    let objects = loader.parse(x3dObject, parentObject);
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
    let id = pose.id;
    let fields = appliedFields;
    for (let key in pose) {
      if (key === 'id')
        continue;
      if (fields.indexOf(key) !== -1)
        continue;
      let newValue = pose[key];
      let object = this.getObjectById('n' + id, true);
      if (typeof object === 'undefined')
        continue; // error

      let valid = true;
      if (key === 'translation') {
        if (object.isTexture) {
          let translation = convertStringToVec2(newValue);
          if (object.userData && object.userData.transform) {
            object.userData.transform.translation = translation;
            object.needsUpdate = true;
            this.sceneModified = true;
          }
        } else if (object.isObject3D) {
          let newPosition = convertStringToVec3(newValue);
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
        let quaternion = convertStringToQuaternion(newValue);
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

    let raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(screenPosition, this.viewpoint.camera);
    return this.gpuPicker.pick(relativePosition, raycaster);
  }

  getCamera() {
    return this.viewpoint.camera;
  }

  getTopX3dNode(node) {
    // If it exists, return the upmost Solid, otherwise the top node.
    let upmostSolid;
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
        let o = this.getObjectById(id, skipBoundingObject, object[i], context);
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

    let childObject;
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

  getRobotWindows() {
    let windows = [];
    let nodes = this.root ? this.root.children : [];
    nodes.forEach((node) => {
      if (node.isObject3D && node.userData && node.userData.window && node.userData.name)
        windows.push([node.userData.name, node.userData.window]);
    });
    return windows;
  }

  processServerMessage(data, view) {
    if (data.startsWith('application/json:')) {
      if (typeof view.time !== 'undefined') { // otherwise ignore late updates until the scene loading is completed
        data = data.substring(data.indexOf(':') + 1);
        let frame = JSON.parse(data);
        view.time = frame.time;
        $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(frame.time));
        if (frame.hasOwnProperty('poses')) {
          for (let i = 0; i < frame.poses.length; i++)
            this.applyPose(frame.poses[i]);
        }
        if (this.viewpoint.updateViewpointPosition(null, view.time))
          this.viewpoint.notifyCameraParametersChanged(false);
        this.onSceneUpdate();
      }
    } else if (data.startsWith('node:')) {
      data = data.substring(data.indexOf(':') + 1);
      let parentId = data.split(':')[0];
      data = data.substring(data.indexOf(':') + 1);
      this.loadObject(data, parentId);
    } else if (data.startsWith('delete:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      this.deleteObject(data);
    } else if (data.startsWith('model:')) {
      $('#webotsProgressMessage').html('Loading 3D scene...');
      $('#webotsProgressPercent').html('');
      this.destroyWorld();
      view.removeLabels();
      data = data.substring(data.indexOf(':') + 1).trim();
      if (!data) // received an empty model case: just destroy the view
        return true;
      this.loadObject(data);
    } else if (data.startsWith('label')) {
      let semiColon = data.indexOf(';');
      let id = data.substring(data.indexOf(':'), semiColon);
      let previousSemiColon;
      let labelProperties = []; // ['font', 'color', 'size', 'x', 'y', 'text']
      for (let i = 0; i < 5; i++) {
        previousSemiColon = semiColon + 1;
        semiColon = data.indexOf(';', previousSemiColon);
        labelProperties.push(data.substring(previousSemiColon, semiColon));
      }
      view.setLabel({
        id: id,
        text: data.substring(semiColon + 1, data.length),
        font: labelProperties[0],
        color: labelProperties[1],
        size: labelProperties[2],
        x: labelProperties[3],
        y: labelProperties[4]
      });
    } else
      return false;
    return true;
  }

  // private functions
  _setupLights(directionalLights) {
    if (!this.root)
      return;

    let sceneBox = new THREE.Box3();
    sceneBox.setFromObject(this.root);
    let boxSize = new THREE.Vector3();
    sceneBox.getSize(boxSize);
    let boxCenter = new THREE.Vector3();
    sceneBox.getCenter(boxCenter);
    let halfWidth = boxSize.x / 2 + boxCenter.x;
    let halfDepth = boxSize.z / 2 + boxCenter.z;
    let maxSize = 2 * Math.max(halfWidth, boxSize.y / 2 + boxCenter.y, halfDepth);
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
    let isHDR = false;
    let backgroundMap;
    if (this.scene.background) {
      if (this.scene.background.isColor) {
        let color = this.scene.background.clone();
        color.convertLinearToSRGB();
        backgroundMap = TextureLoader.createColoredCubeTexture(color);
      } else
        backgroundMap = this.scene.background;
    }

    if (typeof this.scene.userData.irradiance !== 'undefined') {
      isHDR = true;
      backgroundMap = this.scene.userData.irradiance;
    }

    this.scene.traverse((child) => {
      if (child.isMesh && child.material && child.material.isMeshStandardMaterial) {
        let material = child.material;
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
      let node = object;
      let source;
      while (node && node !== this.root) {
        if (typeof node.userData.USE !== 'undefined')
          source = node;
        node = node.parent;
      }
      this.useNodeCache[id] = { 'source': source };
      if (typeof source !== 'undefined') {
        this.useNodeCache[id].target = [];
        source.userData.USE.split(';').forEach((useId) => {
          let useObject = this.getObjectById(useId);
          if (typeof useObject !== 'undefined')
            this.useNodeCache[id].target.push(useObject);
        });
      }
    }
    if (typeof this.useNodeCache[id].source !== 'undefined') {
      // clone again changed DEF node instance
      let sourceNode = this.useNodeCache[id].source;
      let targetNodes = this.useNodeCache[id].target;
      let newTargetNodes = [];
      for (let i = 0, l = targetNodes.length; i < l; i++) {
        let target = targetNodes[i];
        let newClone = sourceNode.clone();
        newClone.name = target.name;
        let parent = target.parent;
        let index = parent.children.indexOf(target);
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
