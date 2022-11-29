import {M_PI_4} from './nodes/utils/constants.js';
import WbAbstractAppearance from './nodes/WbAbstractAppearance.js';
import WbAppearance from './nodes/WbAppearance.js';
import WbBackground from './nodes/WbBackground.js';
import WbBillboard from './nodes/WbBillboard.js';
import WbBox from './nodes/WbBox.js';
import WbCapsule from './nodes/WbCapsule.js';
import WbCadShape from './nodes/WbCadShape.js';
import WbCone from './nodes/WbCone.js';
import WbCylinder from './nodes/WbCylinder.js';
import WbDirectionalLight from './nodes/WbDirectionalLight.js';
import WbElevationGrid from './nodes/WbElevationGrid.js';
import WbFog from './nodes/WbFog.js';
import WbGeometry from './nodes/WbGeometry.js';
import WbGroup from './nodes/WbGroup.js';
import WbImageTexture from './nodes/WbImageTexture.js';
import WbIndexedFaceSet from './nodes/WbIndexedFaceSet.js';
import WbIndexedLineSet from './nodes/WbIndexedLineSet.js';
import WbLight from './nodes/WbLight.js';
import WbMaterial from './nodes/WbMaterial.js';
import WbMesh from './nodes/WbMesh.js';
import WbPbrAppearance from './nodes/WbPbrAppearance.js';
import WbPlane from './nodes/WbPlane.js';
import WbPointLight from './nodes/WbPointLight.js';
import WbPointSet from './nodes/WbPointSet.js';
import WbScene from './nodes/WbScene.js';
import WbShape from './nodes/WbShape.js';
import WbSolid from './nodes/WbSolid.js';
import WbSphere from './nodes/WbSphere.js';
import WbSpotLight from './nodes/WbSpotLight.js';
import WbTextureTransform from './nodes/WbTextureTransform.js';
import WbTrack from './nodes/WbTrack.js';
import WbTrackWheel from './nodes/WbTrackWheel.js';
import WbTransform from './nodes/WbTransform.js';
import WbVector2 from './nodes/utils/WbVector2.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbVector4 from './nodes/utils/WbVector4.js';
import WbViewpoint from './nodes/WbViewpoint.js';
import WbWorld from './nodes/WbWorld.js';
import WbWrenPostProcessingEffects from './wren/WbWrenPostProcessingEffects.js';

import {getAnId} from './nodes/utils/utils.js';

import DefaultUrl from './DefaultUrl.js';
import {webots} from './webots.js';
import {loadImageTextureInWren, loadTextureData} from './image_loader.js';
/*
  This module takes an x3d world, parse it and populate the scene.
*/
export default class Parser {
  #downloadingImage;
  #nodeCounter;
  #nodeNumber;
  #prefix;
  #promises;
  #promiseCounter;
  #promiseNumber;
  constructor(prefix = '') {
    this.#prefix = prefix;
    this.#downloadingImage = new Set();
    this.#promises = [];
    this.#promiseCounter = 0;
    this.#promiseNumber = 0;
    WbWorld.init();
  }

  parse(text, renderer, parent, callback) {
    webots.currentView.progress.setProgressBar('Connecting to webots instance...', 'same', 60 + 0.1 * 30, 'Parsing object...');
    let xml = null;
    if (window.DOMParser) {
      const parser = new DOMParser();
      xml = parser.parseFromString(text, 'text/xml');
    }

    if (typeof xml === 'undefined')
      console.error('File to parse not found');
    else {
      const scene = xml.getElementsByTagName('Scene')[0];
      if (typeof scene === 'undefined') {
        const node = xml.getElementsByTagName('nodes')[0];
        if (typeof node === 'undefined')
          console.error('Unknown content, nor Scene, nor Node');
        else {
          this.#nodeNumber = 0;
          this.#nodeCounter = 0;
          this.#countChildElements(node);
          this.#parseChildren(node, parent);
        }
      } else {
        this.#nodeNumber = 0;
        this.#nodeCounter = 0;
        this.#countChildElements(scene);
        this.#parseNode(scene);
      }
    }

    webots.currentView.progress.setProgressBar('block', 'Finalizing...', 75, 'Finalizing webotsJS nodes...');

    return Promise.all(this.#promises).then(() => {
      this.#promises = [];
      this.#downloadingImage.clear();
      if (typeof this.smaaAreaTexture !== 'undefined' && typeof this.smaaSearchTexture !== 'undefined' &&
        typeof this.gtaoNoiseTexture !== 'undefined') {
        WbWrenPostProcessingEffects.loadResources(this.smaaAreaTexture, this.smaaSearchTexture, this.gtaoNoiseTexture);
        this.smaaAreaTexture = undefined;
        this.smaaSearchTexture = undefined;
        this.gtaoNoiseTexture = undefined;
      }

      if (typeof WbWorld.instance.viewpoint === 'undefined')
        return;
      WbWorld.instance.viewpoint.finalize();

      if (typeof WbBackground.instance !== 'undefined') {
        WbBackground.instance.setCubeArray(this.cubeImages);
        this.cubeImages = undefined;
        WbBackground.instance.setIrradianceCubeArray(this.irradianceCubeURL);
        this.irradianceCubeURL = undefined;
      }
      WbWorld.instance.sceneTree.forEach((node, i) => {
        const percentage = 70 + 30 * (i + 1) / WbWorld.instance.sceneTree.length;
        const info = 'Finalizing node ' + node.id + ': ' + Math.round(100 * (i + 1) / WbWorld.instance.sceneTree.length) + '%';
        webots.currentView.progress.setProgressBar('block', 'same', 75 + 0.25 * percentage, info);
        node.finalize();
      });

      WbWorld.instance.readyForUpdates = true;

      webots.currentView.x3dScene.resize();
      renderer.render();
      setTimeout(() => { webots.currentView.progress.setProgressBar('none'); }, 300);

      if (typeof callback === 'function')
        callback();

      console.timeEnd('Loaded in: ');
    });
  }

  #parseNode(node, parentNode, isBoundingObject) {
    this.#nodeCounter += 1;
    const percentage = 30 + 70 * this.#nodeCounter / this.#nodeNumber;
    const infoPercentage = 100 * this.#nodeCounter / this.#nodeNumber;
    const info = 'Parsing node: ' + node.id + ' (' + node.tagName + ') ' + infoPercentage + '%';
    webots.currentView.progress.setProgressBar('block', 'same', 60 + 0.1 * percentage, info);

    if (typeof WbWorld.instance === 'undefined')
      WbWorld.init();

    let result;
    if (node.tagName === 'Scene') {
      this.#parseScene(node);
      this.#parseChildren(node, parentNode);
    } else if (node.tagName === 'WorldInfo')
      this.#parseWorldInfo(node);
    else if (node.tagName === 'Viewpoint')
      WbWorld.instance.viewpoint = this.#parseViewpoint(node);
    else if (node.tagName === 'Background')
      result = this.#parseBackground(node);
    else if (node.tagName === 'Transform')
      result = this.#parseTransform(node, parentNode, isBoundingObject);
    else if (node.tagName === 'Billboard')
      result = this.#parseBillboard(node, parentNode);
    else if (node.tagName === 'Group')
      result = this.#parseGroup(node, parentNode);
    else if (node.tagName === 'Shape')
      result = this.#parseShape(node, parentNode, isBoundingObject);
    else if (node.tagName === 'CadShape')
      result = this.#parseCadShape(node, parentNode);
    else if (node.tagName === 'DirectionalLight')
      result = this.#parseDirectionalLight(node, parentNode);
    else if (node.tagName === 'PointLight')
      result = this.#parsePointLight(node, parentNode);
    else if (node.tagName === 'SpotLight')
      result = this.#parseSpotLight(node, parentNode);
    else if (node.tagName === 'Fog') {
      if (!WbWorld.instance.hasFog)
        result = this.#parseFog(node);
      else
        console.error('This world already has a fog.');
    } else {
      // Either it is a node added after the whole scene, or it is an unknown node, or a geometry bounding object
      let id;
      if (typeof parentNode !== 'undefined')
        id = parentNode.id;
      result = this.#parseGeometry(node, id);
      // We are forced to check if the result correspond to the class we expect because of the case of a USE
      if (typeof result !== 'undefined' && result instanceof WbGeometry) {
        if (typeof parentNode !== 'undefined') {
          if (parentNode instanceof WbShape) {
            if (typeof parentNode.geometry !== 'undefined')
              parentNode.geometry.delete();
            parentNode.geometry = result;
          } else if (parentNode instanceof WbSolid || parentNode instanceof WbTransform || parentNode instanceof WbGroup) {
            // Bounding object
            if (typeof parentNode.boundingObject !== 'undefined')
              parentNode.boundingObject.delete();
            const shape = new WbShape(getAnId(), false, false, result);
            shape.parent = parentNode.id;
            WbWorld.instance.nodes.set(shape.id, shape);
            result.parent = shape.id;
            if (parentNode instanceof WbSolid)
              parentNode.boundingObject = shape;
            else
              parentNode.children.push(shape);
          }
        }
      } else if (node.tagName === 'PBRAppearance') {
        if (typeof parentNode !== 'undefined' && parentNode instanceof WbShape) {
          if (typeof parentNode.appearance !== 'undefined')
            parentNode.appearance.delete();
          result = this.#parsePbrAppearance(node, id);
          parentNode.appearance = result;
        }
      } else if (node.tagName === 'Appearance') {
        if (typeof parentNode !== 'undefined' && parentNode instanceof WbShape) {
          if (typeof parentNode.appearance !== 'undefined')
            parentNode.appearance.delete();
          result = this.#parseAppearance(node, id);
          parentNode.appearance = result;
        }
      } else if (node.tagName === 'Material') {
        result = this.#parseMaterial(node, id);
        if (typeof result !== 'undefined') {
          if (typeof parentNode !== 'undefined' && parentNode instanceof WbAppearance) {
            if (typeof parentNode.material !== 'undefined')
              parentNode.material.delete();
            parentNode.material = result;
          }
        }
      } else if (node.tagName === 'ImageTexture') {
        result = this.#parseImageTexture(node, id);
        if (typeof result !== 'undefined') {
          if (typeof parentNode !== 'undefined' && parentNode instanceof WbAppearance) {
            if (typeof parentNode.material !== 'undefined')
              parentNode.texture.delete();
            parentNode.texture = result;
          }
        }
      } else if (node.tagName === 'TextureTransform') {
        result = this.#parseTextureTransform(node, id);
        if (typeof result !== 'undefined') {
          if (typeof parentNode !== 'undefined' && parentNode instanceof WbAbstractAppearance) {
            if (typeof parentNode.textureTransform !== 'undefined')
              parentNode.textureTransform.delete();
            parentNode.textureTransform = result;
          }
        }
      } else
        console.error("The parser doesn't support this type of node: " + node.tagName);
    }

    // check if top-level nodes
    if (typeof result !== 'undefined' && typeof parentNode === 'undefined')
      WbWorld.instance.sceneTree.push(result);
  }

  #parseChildren(node, parentNode, isBoundingObject) {
    for (let i = 0; i < node.childNodes.length; i++) {
      const child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined')
        this.#parseNode(child, parentNode, isBoundingObject);
    }
  }

  #parseScene(node) {
    const prefix = DefaultUrl.wrenImagesUrl();
    this.#promises.push(loadTextureData(prefix, 'smaa_area_texture.png').then(image => {
      this.smaaAreaTexture = image;
      this.smaaAreaTexture.isTranslucent = false;
      this.#updatePromiseCounter('Downloading assets: Texture \'smaa_area_texture.png\'...');
    }));
    this.#promises.push(loadTextureData(prefix, 'smaa_search_texture.png').then(image => {
      this.smaaSearchTexture = image;
      this.smaaSearchTexture.isTranslucent = false;
      this.#updatePromiseCounter('Downloading assets: Texture \'smaa_search_texture.png\'...');
    }));
    this.#promises.push(loadTextureData(prefix, 'gtao_noise_texture.png').then(image => {
      this.gtaoNoiseTexture = image;
      this.gtaoNoiseTexture.isTranslucent = true;
      this.#updatePromiseCounter('Downloading assets: Texture \'gtao_noise_texture.png\'...');
    }));
    this.#promiseNumber += 3;

    WbWorld.instance.scene = new WbScene();
  }

  #parseWorldInfo(node) {
    WbWorld.instance.coordinateSystem = getNodeAttribute(node, 'coordinateSystem', 'ENU');
    WbWorld.instance.basicTimeStep = parseInt(getNodeAttribute(node, 'basicTimeStep', 32));
    WbWorld.instance.title = getNodeAttribute(node, 'title', 'No title');
    WbWorld.instance.description = getNodeAttribute(node, 'info', 'No description was provided for this world.');

    // Update information panel when switching between worlds
    let webotsView = document.getElementsByTagName('webots-view')[0];
    if (webotsView && typeof webotsView.toolbar !== 'undefined') {
      let informationPanel = webotsView.toolbar.informationPanel;
      if (typeof informationPanel !== 'undefined') {
        informationPanel.setTitle(WbWorld.instance.title);
        informationPanel.setDescription(WbWorld.instance.description);
      }
    }
    WbWorld.computeUpVector();
  }

  #parseId(node) {
    if (typeof node === 'undefined')
      return;

    let id = getNodeAttribute(node, 'id');
    if (typeof id === 'undefined')
      id = getAnId();

    return id;
  }

  #parseViewpoint(node) {
    const id = this.#parseId(node);
    const fieldOfView = parseFloat(getNodeAttribute(node, 'fieldOfView', M_PI_4));
    const orientation = convertStringToQuaternion(getNodeAttribute(node, 'orientation', '0 0 1 0'));
    const position = convertStringToVec3(getNodeAttribute(node, 'position', '0 0 10'));
    const exposure = parseFloat(getNodeAttribute(node, 'exposure', '1.0'));
    const bloomThreshold = parseFloat(getNodeAttribute(node, 'bloomThreshold', 21));
    const far = parseFloat(getNodeAttribute(node, 'zFar', '2000'));
    const near = parseFloat(getNodeAttribute(node, 'zNear', '0.1'));
    const followSmoothness = parseFloat(getNodeAttribute(node, 'followSmoothness'));
    const followedId = getNodeAttribute(node, 'followedId');
    const ambientOcclusionRadius = parseFloat(getNodeAttribute(node, 'ambientOcclusionRadius', 2));

    return new WbViewpoint(id, fieldOfView, orientation, position, exposure, bloomThreshold, near, far, followSmoothness,
      followedId, ambientOcclusionRadius);
  }

  #parseBackground(node) {
    const id = this.#parseId(node);
    const skyColor = convertStringToVec3(getNodeAttribute(node, 'skyColor', '0 0 0'));
    const luminosity = parseFloat(getNodeAttribute(node, 'luminosity', '1'));

    const backgroundIdx = (WbWorld.instance.coordinateSystem === 'ENU') ? [0, 1, 2, 3, 4, 5] : [5, 0, 1, 2, 3, 4];
    const rotationValues = (WbWorld.instance.coordinateSystem === 'ENU') ? [90, -90, -90, 180, 0, -90] : [0, 0, 0, 0, 0, 0];
    const cubeImageIdx = (WbWorld.instance.coordinateSystem === 'ENU') ? [0, 4, 1, 3, 2, 5] : [2, 5, 3, 4, 1, 0];

    let backgroundUrl = [];
    backgroundUrl[0] = getNodeAttribute(node, 'backUrl');
    backgroundUrl[1] = getNodeAttribute(node, 'bottomUrl');
    backgroundUrl[2] = getNodeAttribute(node, 'frontUrl');
    backgroundUrl[3] = getNodeAttribute(node, 'leftUrl');
    backgroundUrl[4] = getNodeAttribute(node, 'rightUrl');
    backgroundUrl[5] = getNodeAttribute(node, 'topUrl');

    let areUrlsPresent = true;
    for (let i = 0; i < 6; i++) {
      if (typeof backgroundUrl[i] === 'undefined') {
        areUrlsPresent = false;
        break;
      } else
        // filter removes empty elements.
        backgroundUrl[i] = backgroundUrl[i].split('"').filter(element => { if (element !== ' ') return element; })[0];
    }

    this.cubeImages = [];
    if (areUrlsPresent) {
      for (let i = 0; i < 6; i++) {
        this.#promises.push(loadTextureData(this.#prefix, backgroundUrl[backgroundIdx[i]], false, rotationValues[i])
          .then(image => {
            this.cubeImages[cubeImageIdx[i]] = image;
            this.#updatePromiseCounter('Downloading assets: Texture \'background ' + i + '\'...');
          }));
      }
      this.#promiseNumber += 6;
    }

    let backgroundIrradianceUrl = [];
    backgroundIrradianceUrl[0] = getNodeAttribute(node, 'backIrradianceUrl');
    backgroundIrradianceUrl[1] = getNodeAttribute(node, 'bottomIrradianceUrl');
    backgroundIrradianceUrl[2] = getNodeAttribute(node, 'frontIrradianceUrl');
    backgroundIrradianceUrl[3] = getNodeAttribute(node, 'leftIrradianceUrl');
    backgroundIrradianceUrl[4] = getNodeAttribute(node, 'rightIrradianceUrl');
    backgroundIrradianceUrl[5] = getNodeAttribute(node, 'topIrradianceUrl');

    let areIrradianceUrlsPresent = true;
    for (let i = 0; i < 6; i++) {
      if (typeof backgroundIrradianceUrl[i] === 'undefined') {
        areIrradianceUrlsPresent = false;
        break;
      } else // filter removes empty elements.
        backgroundIrradianceUrl[i] = backgroundIrradianceUrl[i].split('"')
          .filter(element => { if (element !== ' ') return element; })[0];
    }

    this.irradianceCubeURL = [];
    if (areIrradianceUrlsPresent) {
      for (let i = 0; i < 6; i++) {
        this.#promises.push(loadTextureData(this.#prefix, backgroundIrradianceUrl[backgroundIdx[i]], true, rotationValues[i])
          .then(image => {
            this.irradianceCubeURL[cubeImageIdx[i]] = image;
            this.#updatePromiseCounter('Downloading assets: Texture \'background irradiance ' + i + '\'...');
          }));
      }
      this.#promiseNumber += 6;
    }

    const background = new WbBackground(id, skyColor, luminosity);
    WbBackground.instance = background;

    WbWorld.instance.nodes.set(background.id, background);

    return background;
  }

  #countChildElements(tree) {
    if (tree !== 'undefined') {
      tree.childNodes.forEach(child => {
        if (child.tagName) {
          this.#nodeNumber += 1;
          this.#countChildElements(child);
        }
      });
    }
  }

  #updatePromiseCounter(info) {
    this.#promiseCounter += 1;
    const percentage = 70 * this.#promiseCounter / this.#promiseNumber;
    webots.currentView.progress.setProgressBar('block', 'same', 75 + 0.25 * percentage, info);
  }

  #checkUse(node, parentNode) {
    let use = getNodeAttribute(node, 'USE');
    if (typeof use === 'undefined')
      return;

    let result = WbWorld.instance.nodes.get(use);

    if (typeof result === 'undefined') {
      use = 'n' + use;
      result = WbWorld.instance.nodes.get(use);
    }

    if (typeof result === 'undefined')
      return;
    const id = this.#parseId(node);

    const useNode = result.clone(id);
    if (typeof parentNode !== 'undefined') {
      useNode.parent = parentNode.id;
      const isBoundingObject = getNodeAttribute(node, 'role', undefined) === 'boundingObject';
      if (isBoundingObject && (result instanceof WbShape || result instanceof WbGroup || result instanceof WbGeometry))
        parentNode.boundingObject = useNode;
      else if (result instanceof WbShape || result instanceof WbGroup || result instanceof WbLight ||
         result instanceof WbCadShape)
        parentNode.children.push(useNode);
    }

    WbWorld.instance.nodes.set(id, useNode);
    return useNode;
  }

  #parseTransform(node, parentNode, isBoundingObject) {
    const use = this.#checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);

    const type = getNodeAttribute(node, 'type', '').toLowerCase();
    const translation = convertStringToVec3(getNodeAttribute(node, 'translation', '0 0 0'));
    const scale = convertStringToVec3(getNodeAttribute(node, 'scale', '1 1 1'));
    const rotation = convertStringToQuaternion(getNodeAttribute(node, 'rotation', '0 0 1 0'));

    let newNode;
    if (type === 'track') {
      const geometriesCount = parseInt(getNodeAttribute(node, 'geometriesCount', '10'));
      newNode = new WbTrack(id, translation, scale, rotation, geometriesCount);
    } else if (type === 'trackwheel') {
      const radius = parseFloat(getNodeAttribute(node, 'radius', '0.1'));
      const inner = getNodeAttribute(node, 'inner', '0').toLowerCase() === '1';

      newNode = new WbTrackWheel(id, translation, scale, rotation, radius, inner);

      parentNode.wheelsList.push(newNode);
    } else if (type === 'solid' || type === 'robot')
      newNode = new WbSolid(id, translation, scale, rotation);
    else {
      if (!isBoundingObject)
        isBoundingObject = getNodeAttribute(node, 'role', undefined) === 'boundingObject';

      newNode = new WbTransform(id, translation, scale, rotation);
    }

    WbWorld.instance.nodes.set(newNode.id, newNode);

    this.#parseChildren(node, newNode, isBoundingObject);

    if (typeof parentNode !== 'undefined') {
      newNode.parent = parentNode.id;
      if (getNodeAttribute(node, 'role', '') === 'animatedGeometry')
        parentNode.geometryField = newNode;
      else if (isBoundingObject && parentNode instanceof WbSolid)
        parentNode.boundingObject = newNode;
      else
        parentNode.children.push(newNode);
    }

    return newNode;
  }

  #parseGroup(node, parentNode) {
    const use = this.#checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);

    const isPropeller = getNodeAttribute(node, 'type', '').toLowerCase() === 'propeller';
    const isBoundingObject = getNodeAttribute(node, 'role', undefined) === 'boundingObject';

    const group = new WbGroup(id, isPropeller);
    WbWorld.instance.nodes.set(group.id, group);
    this.#parseChildren(node, group, isBoundingObject);

    if (typeof parentNode !== 'undefined') {
      group.parent = parentNode.id;
      if (isBoundingObject && parentNode instanceof WbSolid)
        parentNode.boundingObject = group;
      else
        parentNode.children.push(group);
    }

    return group;
  }

  #parseShape(node, parentNode, isBoundingObject) {
    const use = this.#checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);

    const castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';
    const isPickable = getNodeAttribute(node, 'isPickable', 'true').toLowerCase() === 'true';
    if (!isBoundingObject)
      isBoundingObject = getNodeAttribute(node, 'role', undefined) === 'boundingObject';

    let geometry;
    let appearance;
    // Go through the nodes in reverse order to encounter PbrAppearance before normal appearance if both are present.
    for (let i = node.childNodes.length - 1; i >= 0; i--) {
      const child = node.childNodes[i];
      if (typeof child.tagName === 'undefined')
        continue;

      if (typeof appearance === 'undefined' && !isBoundingObject) {
        if (child.tagName === 'Appearance')
          appearance = this.#parseAppearance(child);
        else if (child.tagName === 'PBRAppearance')
          appearance = this.#parsePbrAppearance(child);

        if (typeof appearance !== 'undefined')
          continue;
      }

      if (typeof geometry === 'undefined') {
        geometry = this.#parseGeometry(child, id);
        if (typeof geometry !== 'undefined')
          continue;
      }
      if (!(isBoundingObject && (child.tagName === 'Appearance' || child.tagName === 'PBRAppearance'))) {
        console.error('Parser: error with node: ' + child.tagName +
          '. Either the node is unknown or the same shape contains several appearances/geometries.');
      }
    }

    const shape = new WbShape(id, castShadows, isPickable, geometry, appearance);

    if (typeof parentNode !== 'undefined') {
      if (isBoundingObject && parentNode instanceof WbSolid)
        parentNode.boundingObject = shape;
      else
        parentNode.children.push(shape);
      shape.parent = parentNode.id;
    }

    if (typeof appearance !== 'undefined')
      appearance.parent = shape.id;

    WbWorld.instance.nodes.set(shape.id, shape);

    return shape;
  }

  #parseCadShape(node, parentNode) {
    const use = this.#checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);

    let urls = getNodeAttribute(node, 'url', '');
    if (typeof urls !== 'undefined')
      urls = urls.split('"').filter(element => { if (element !== ' ') return element; }); // filter removes empty elements

    const ccw = getNodeAttribute(node, 'ccw', 'true').toLowerCase() === 'true';
    const castShadows = getNodeAttribute(node, 'castShadows', 'true').toLowerCase() === 'true';
    const isPickable = getNodeAttribute(node, 'isPickable', 'true').toLowerCase() === 'true';

    const cadShape = new WbCadShape(id, urls, ccw, castShadows, isPickable, this.#prefix);

    WbWorld.instance.nodes.set(cadShape.id, cadShape);

    if (typeof parentNode !== 'undefined') {
      cadShape.parent = parentNode.id;
      parentNode.children.push(cadShape);
    }

    this.#promises.push(loadMeshData(this.#prefix, urls).then(meshContent => {
      cadShape.scene = meshContent;
      for (let i = 0; i < cadShape.useList.length; i++) {
        const node = WbWorld.instance.nodes.get(cadShape.useList[i]);
        node.scene = meshContent;
      }
      this.#updatePromiseCounter('Downloading assets: Mesh \'CadShape\'...');
    }));
    this.#promiseNumber += 1;

    return cadShape;
  }

  #parseBillboard(node, parentNode) {
    const id = this.#parseId(node);

    const billboard = new WbBillboard(id);

    WbWorld.instance.nodes.set(billboard.id, billboard);
    this.#parseChildren(node, billboard);

    return billboard;
  }

  #parseDirectionalLight(node, parentNode) {
    const use = this.#checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    const on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const direction = convertStringToVec3(getNodeAttribute(node, 'direction', '0 0 -1'));
    const intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    const ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    const castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    const dirLight = new WbDirectionalLight(id, on, color, direction, intensity, castShadows, ambientIntensity);

    if (typeof parentNode !== 'undefined' && typeof dirLight !== 'undefined') {
      parentNode.children.push(dirLight);
      dirLight.parent = parentNode.id;
    }

    WbWorld.instance.nodes.set(dirLight.id, dirLight);

    return dirLight;
  }

  #parsePointLight(node, parentNode) {
    const use = this.#checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    const on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    const attenuation = convertStringToVec3(getNodeAttribute(node, 'attenuation', '1 0 0'));
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    const location = convertStringToVec3(getNodeAttribute(node, 'location', '0 0 0'));
    const radius = parseFloat(getNodeAttribute(node, 'radius', '100'));
    const ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    const castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    const pointLight = new WbPointLight(id, on, attenuation, color, intensity, location, radius, ambientIntensity,
      castShadows, parentNode);

    if (typeof parentNode !== 'undefined' && typeof pointLight !== 'undefined')
      parentNode.children.push(pointLight);

    WbWorld.instance.nodes.set(pointLight.id, pointLight);

    return pointLight;
  }

  #parseSpotLight(node, parentNode) {
    const use = this.#checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    const on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    const attenuation = convertStringToVec3(getNodeAttribute(node, 'attenuation', '1 0 0'));
    const beamWidth = parseFloat(getNodeAttribute(node, 'beamWidth', '0.785'));
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const cutOffAngle = parseFloat(getNodeAttribute(node, 'cutOffAngle', '0.785'));
    const direction = convertStringToVec3(getNodeAttribute(node, 'direction', '0 0 -1'));
    const intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    const location = convertStringToVec3(getNodeAttribute(node, 'location', '0 0 0'));
    const radius = parseFloat(getNodeAttribute(node, 'radius', '100'));
    const ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    const castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    const spotLight = new WbSpotLight(id, on, attenuation, beamWidth, color, cutOffAngle, direction, intensity, location,
      radius, ambientIntensity, castShadows, parentNode);

    if (typeof parentNode !== 'undefined' && typeof spotLight !== 'undefined')
      parentNode.children.push(spotLight);

    WbWorld.instance.nodes.set(spotLight.id, spotLight);

    return spotLight;
  }

  #parseFog(node) {
    const id = this.#parseId(node);
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const visibilityRange = parseFloat(getNodeAttribute(node, 'visibilityRange', '0'));
    const fogType = getNodeAttribute(node, 'fogType', 'LINEAR');

    const fog = new WbFog(id, color, visibilityRange, fogType);

    WbWorld.instance.nodes.set(fog.id, fog);

    if (typeof fog !== 'undefined')
      WbWorld.instance.hasFog = true;

    return fog;
  }

  #parseGeometry(node, parentId) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined') {
      use.parent = parentId;
      return use;
    }

    const id = this.#parseId(node);

    let geometry;
    if (node.tagName === 'Box')
      geometry = this.#parseBox(node, id);
    else if (node.tagName === 'Sphere')
      geometry = this.#parseSphere(node, id);
    else if (node.tagName === 'Cone')
      geometry = this.#parseCone(node, id);
    else if (node.tagName === 'Plane')
      geometry = this.#parsePlane(node, id);
    else if (node.tagName === 'Cylinder')
      geometry = this.#parseCylinder(node, id);
    else if (node.tagName === 'Capsule')
      geometry = this.#parseCapsule(node, id);
    else if (node.tagName === 'IndexedFaceSet')
      geometry = this.#parseIndexedFaceSet(node, id);
    else if (node.tagName === 'IndexedLineSet')
      geometry = this.#parseIndexedLineSet(node, id);
    else if (node.tagName === 'ElevationGrid')
      geometry = this.#parseElevationGrid(node, id);
    else if (node.tagName === 'PointSet')
      geometry = this.#parsePointSet(node, id);
    else if (node.tagName === 'Mesh')
      geometry = this.#parseMesh(node, id);

    if (typeof parentId !== 'undefined' && typeof geometry !== 'undefined')
      geometry.parent = parentId;

    return geometry;
  }

  #parseBox(node, id) {
    const size = convertStringToVec3(getNodeAttribute(node, 'size', '2 2 2'));

    const box = new WbBox(id, size);
    WbWorld.instance.nodes.set(box.id, box);
    return box;
  }

  #parseSphere(node, id) {
    const radius = parseFloat(getNodeAttribute(node, 'radius', '1'));
    const ico = getNodeAttribute(node, 'ico', 'false').toLowerCase() === 'true';
    const subdivision = parseInt(getNodeAttribute(node, 'subdivision', '1,1'));

    const sphere = new WbSphere(id, radius, ico, subdivision);

    WbWorld.instance.nodes.set(sphere.id, sphere);

    return sphere;
  }

  #parseCone(node, id) {
    const bottomRadius = getNodeAttribute(node, 'bottomRadius', '1');
    const height = getNodeAttribute(node, 'height', '2');
    const subdivision = getNodeAttribute(node, 'subdivision', '32');
    const side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    const bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';

    const cone = new WbCone(id, bottomRadius, height, subdivision, side, bottom);

    WbWorld.instance.nodes.set(cone.id, cone);

    return cone;
  }

  #parseCylinder(node, id) {
    const radius = getNodeAttribute(node, 'radius', '1');
    const height = getNodeAttribute(node, 'height', '2');
    const subdivision = getNodeAttribute(node, 'subdivision', '32');
    const bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';
    const side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    const top = getNodeAttribute(node, 'top', 'true').toLowerCase() === 'true';

    const cylinder = new WbCylinder(id, radius, height, subdivision, bottom, side, top);

    WbWorld.instance.nodes.set(cylinder.id, cylinder);

    return cylinder;
  }

  #parsePlane(node, id) {
    const size = convertStringToVec2(getNodeAttribute(node, 'size', '1,1'));

    const plane = new WbPlane(id, size);

    WbWorld.instance.nodes.set(plane.id, plane);

    return plane;
  }

  #parseCapsule(node, id) {
    const radius = getNodeAttribute(node, 'radius', '1');
    const height = getNodeAttribute(node, 'height', '2');
    const subdivision = getNodeAttribute(node, 'subdivision', '32');
    const bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';
    const side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    const top = getNodeAttribute(node, 'top', 'true').toLowerCase() === 'true';

    const capsule = new WbCapsule(id, radius, height, subdivision, bottom, side, top);

    WbWorld.instance.nodes.set(capsule.id, capsule);

    return capsule;
  }

  #parseIndexedFaceSet(node, id) {
    let coordIndex = convertStringToFloatArray(getNodeAttribute(node, 'coordIndex', ''));
    if (coordIndex)
      coordIndex = coordIndex.filter(element => { return element !== -1; });

    let normalIndex = convertStringToFloatArray(getNodeAttribute(node, 'normalIndex', ''));
    if (normalIndex)
      normalIndex = normalIndex.filter(element => { return element !== -1; });

    let texCoordIndex = convertStringToFloatArray(getNodeAttribute(node, 'texCoordIndex', ''));
    if (texCoordIndex)
      texCoordIndex = texCoordIndex.filter(element => { return element !== -1; });

    const coordArray = [];
    const coordinate = node.getElementsByTagName('Coordinate')[0];
    if (typeof coordinate !== 'undefined') {
      const coords = convertStringToFloatArray(getNodeAttribute(coordinate, 'point', ''));
      for (let i = 0; i < coords.length; i = i + 3)
        coordArray.push(new WbVector3(coords[i], coords[i + 1], coords[i + 2]));
    }

    const texCoordArray = [];
    const textureCoordinate = node.getElementsByTagName('TextureCoordinate')[0];
    if (typeof textureCoordinate !== 'undefined') {
      const texCoords = convertStringToFloatArray(getNodeAttribute(textureCoordinate, 'point', ''));
      for (let i = 0; i < texCoords.length; i = i + 2)
        texCoordArray.push(new WbVector2(texCoords[i], texCoords[i + 1]));
    }

    const normalArray = [];
    const normalNode = node.getElementsByTagName('Normal')[0];
    if (typeof normalNode !== 'undefined') {
      const normals = convertStringToFloatArray(getNodeAttribute(normalNode, 'vector', ''));
      for (let i = 0; i < normals.length; i = i + 3)
        normalArray.push(new WbVector3(normals[i], normals[i + 1], normals[i + 2]));
    }

    const ccw = getNodeAttribute(node, 'ccw', 'true').toLowerCase() === 'true';
    const ifs = new WbIndexedFaceSet(id, coordIndex, normalIndex, texCoordIndex, coordArray, texCoordArray, normalArray, ccw);
    WbWorld.instance.nodes.set(ifs.id, ifs);

    return ifs;
  }

  #parseIndexedLineSet(node, id) {
    const coordinate = node.getElementsByTagName('Coordinate')[0];

    if (typeof coordinate === 'undefined')
      return undefined;

    const indicesStr = convertStringToFloatArray(getNodeAttribute(node, 'coordIndex', ''));

    const verticesStr = convertStringToFloatArray(getNodeAttribute(coordinate, 'point', ''));

    const coord = [];
    for (let i = 0; i < verticesStr.length; i += 3)
      coord.push(new WbVector3(verticesStr[i], verticesStr[i + 1], verticesStr[i + 2]));

    const coordIndex = indicesStr.map(Number);

    const ils = new WbIndexedLineSet(id, coord, coordIndex);
    WbWorld.instance.nodes.set(ils.id, ils);

    return ils;
  }

  #parseElevationGrid(node, id) {
    const heightStr = getNodeAttribute(node, 'height');
    if (typeof heightStr === 'undefined')
      return;

    const xDimension = parseInt(getNodeAttribute(node, 'xDimension', '0'));
    const xSpacing = parseFloat(getNodeAttribute(node, 'xSpacing', '1'));
    const yDimension = parseInt(getNodeAttribute(node, 'yDimension', '0'));
    const ySpacing = parseFloat(getNodeAttribute(node, 'ySpacing', '1'));
    const thickness = parseFloat(getNodeAttribute(node, 'thickness', '1'));

    const height = convertStringToFloatArray(heightStr);

    const eg = new WbElevationGrid(id, height, xDimension, xSpacing, yDimension, ySpacing, thickness);
    WbWorld.instance.nodes.set(eg.id, eg);

    return eg;
  }

  #parsePointSet(node, id) {
    const coordinate = node.getElementsByTagName('Coordinate')[0];

    if (typeof coordinate === 'undefined')
      return;

    const coordArray = convertStringToFloatArray(getNodeAttribute(coordinate, 'point', ''));

    if (typeof coordArray === 'undefined')
      return;

    const coord = [];
    for (let i = 0; i < coordArray.length; i += 3)
      coord.push(new WbVector3(coordArray[i], coordArray[i + 1], coordArray[i + 2]));

    const colorNode = node.getElementsByTagName('Color')[0];
    let color;
    if (typeof colorNode !== 'undefined') {
      const colorArray = convertStringToFloatArray(getNodeAttribute(colorNode, 'color', ''));
      if (typeof colorArray !== 'undefined') {
        color = [];
        for (let i = 0; i < colorArray.length; i += 3)
          color.push(new WbVector3(colorArray[i], colorArray[i + 1], colorArray[i + 2]));
      }
    }

    const ps = new WbPointSet(id, coord, color);
    WbWorld.instance.nodes.set(ps.id, ps);

    return ps;
  }

  #parseMesh(node, id) {
    let urls = getNodeAttribute(node, 'url', '');
    if (typeof urls !== 'undefined')
      urls = urls.split('"').filter(element => { if (element !== ' ') return element; }); // filter removes empty elements

    const ccw = getNodeAttribute(node, 'ccw', 'true').toLowerCase() === 'true';
    const name = getNodeAttribute(node, 'name', '');
    const materialIndex = parseInt(getNodeAttribute(node, 'materialIndex', -1));

    const mesh = new WbMesh(id, urls[0], ccw, name, materialIndex);
    WbWorld.instance.nodes.set(mesh.id, mesh);

    this.#promises.push(loadMeshData(this.#prefix, urls).then(meshContent => {
      mesh.scene = meshContent;
      for (let i = 0; i < mesh.useList.length; i++) {
        const node = WbWorld.instance.nodes.get(mesh.useList[i]);
        node.scene = meshContent;
      }
      this.#updatePromiseCounter('Downloading assets: Mesh \'mesh ' + name + '\'...');
    }));
    this.#promiseNumber += 1;

    return mesh;
  }

  #parseAppearance(node, parentId) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);

    // Get the Material tag.
    const materialNode = node.getElementsByTagName('Material')[0];
    let material;
    if (typeof materialNode !== 'undefined')
      material = this.#parseMaterial(materialNode);

    // Check to see if there is a texture.
    const imageTexture = node.getElementsByTagName('ImageTexture')[0];
    let texture;
    if (typeof imageTexture !== 'undefined')
      texture = this.#parseImageTexture(imageTexture);

    // Check to see if there is a textureTransform.
    const textureTransform = node.getElementsByTagName('TextureTransform')[0];
    let transform;
    if (typeof textureTransform !== 'undefined')
      transform = this.#parseTextureTransform(textureTransform);

    const appearance = new WbAppearance(id, material, texture, transform);
    if (typeof appearance !== 'undefined') {
      if (typeof material !== 'undefined')
        material.parent = appearance.id;

      if (typeof texture !== 'undefined')
        texture.parent = appearance.id;

      if (typeof transform !== 'undefined')
        transform.parent = appearance.id;

      if (typeof parentId !== 'undefined')
        appearance.parent = parentId;

      WbWorld.instance.nodes.set(appearance.id, appearance);
    }

    return appearance;
  }

  #parseMaterial(node, parentId) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);

    const ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0.2'));
    const diffuseColor = convertStringToVec3(getNodeAttribute(node, 'diffuseColor', '0.8 0.8 0.8'));
    const specularColor = convertStringToVec3(getNodeAttribute(node, 'specularColor', '0 0 0'));
    const emissiveColor = convertStringToVec3(getNodeAttribute(node, 'emissiveColor', '0 0 0'));
    const shininess = parseFloat(getNodeAttribute(node, 'shininess', '0.2'));
    const transparency = parseFloat(getNodeAttribute(node, 'transparency', '0'));

    const material = new WbMaterial(id, ambientIntensity, diffuseColor, specularColor, emissiveColor, shininess, transparency);

    if (typeof parentId !== 'undefined')
      material.parent = parentId;

    WbWorld.instance.nodes.set(material.id, material);

    return material;
  }

  #parseImageTexture(node, parentId) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    let url = getNodeAttribute(node, 'url', '');
    if (typeof url !== 'undefined')
      url = url.split('"').filter(element => { if (element !== ' ') return element; })[0]; // filter removes empty elements.
    const isTransparent = getNodeAttribute(node, 'isTransparent', 'false').toLowerCase() === 'true';
    const s = getNodeAttribute(node, 'repeatS', 'true').toLowerCase() === 'true';
    const t = getNodeAttribute(node, 'repeatT', 'true').toLowerCase() === 'true';
    const filtering = parseFloat(getNodeAttribute(node, 'filtering', '4'));

    let imageTexture;
    if (typeof url !== 'undefined' && url !== '') {
      imageTexture = new WbImageTexture(id, url, isTransparent, s, t, filtering);
      if (!this.#downloadingImage.has(url)) {
        this.#downloadingImage.add(url);
        // Load the texture in WREN
        this.#promises.push(loadImageTextureInWren(this.#prefix, url, isTransparent));
      }
    }

    if (typeof imageTexture !== 'undefined') {
      if (typeof parentId !== 'undefined')
        imageTexture.parent = parentId;

      WbWorld.instance.nodes.set(imageTexture.id, imageTexture);
    }

    return imageTexture;
  }

  #parsePbrAppearance(node, parentId) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);

    const baseColor = convertStringToVec3(getNodeAttribute(node, 'baseColor', '1 1 1'));
    const transparency = parseFloat(getNodeAttribute(node, 'transparency', '0'));
    const roughness = parseFloat(getNodeAttribute(node, 'roughness', '0'));
    const metalness = parseFloat(getNodeAttribute(node, 'metalness', '1'));
    const IBLStrength = parseFloat(getNodeAttribute(node, 'IBLStrength', '1'));
    const normalMapFactor = parseFloat(getNodeAttribute(node, 'normalMapFactor', '1'));
    const occlusionMapStrength = parseFloat(getNodeAttribute(node, 'occlusionMapStrength', '1'));
    const emissiveColor = convertStringToVec3(getNodeAttribute(node, 'emissiveColor', '0 0 0'));
    const emissiveIntensity = parseFloat(getNodeAttribute(node, 'emissiveIntensity', '1'));

    // Check to see if there is a textureTransform.
    const textureTransform = node.getElementsByTagName('TextureTransform')[0];
    let transform;
    if (typeof textureTransform !== 'undefined')
      transform = this.#parseTextureTransform(textureTransform);

    const imageTextures = node.getElementsByTagName('ImageTexture');
    let baseColorMap, roughnessMap, metalnessMap, normalMap, occlusionMap, emissiveColorMap;
    for (let i = 0; i < imageTextures.length; i++) {
      const imageTexture = imageTextures[i];
      const role = getNodeAttribute(imageTexture, 'role', undefined);
      if (role === 'baseColor') {
        baseColorMap = this.#parseImageTexture(imageTexture);
        if (typeof baseColorMap !== 'undefined')
          baseColorMap.role = 'baseColorMap';
      } else if (role === 'roughness') {
        roughnessMap = this.#parseImageTexture(imageTexture);
        if (typeof roughnessMap !== 'undefined')
          roughnessMap.role = 'roughnessMap';
      } else if (role === 'metalness') {
        metalnessMap = this.#parseImageTexture(imageTexture);
        if (typeof metalnessMap !== 'undefined')
          metalnessMap.role = 'metalnessMap';
      } else if (role === 'normal') {
        normalMap = this.#parseImageTexture(imageTexture);
        if (typeof normalMap !== 'undefined')
          normalMap.role = 'normalMap';
      } else if (role === 'occlusion') {
        occlusionMap = this.#parseImageTexture(imageTexture);
        if (typeof occlusionMap !== 'undefined')
          occlusionMap.role = 'occlusionMap';
      } else if (role === 'emissiveColor') {
        emissiveColorMap = this.#parseImageTexture(imageTexture);
        if (typeof emissiveColorMap !== 'undefined')
          emissiveColorMap.role = 'emissiveColorMap';
      }
    }

    const pbrAppearance = new WbPbrAppearance(id, baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness,
      metalnessMap, IBLStrength, normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor,
      emissiveColorMap, emissiveIntensity, transform);

    if (typeof pbrAppearance !== 'undefined') {
      if (typeof transform !== 'undefined')
        transform.parent = pbrAppearance.id;

      if (typeof baseColorMap !== 'undefined')
        baseColorMap.parent = pbrAppearance.id;

      if (typeof roughnessMap !== 'undefined')
        roughnessMap.parent = pbrAppearance.id;

      if (typeof metalnessMap !== 'undefined')
        metalnessMap.parent = pbrAppearance.id;

      if (typeof normalMap !== 'undefined')
        normalMap.parent = pbrAppearance.id;

      if (typeof occlusionMap !== 'undefined')
        occlusionMap.parent = pbrAppearance.id;

      if (typeof emissiveColorMap !== 'undefined')
        emissiveColorMap.parent = pbrAppearance.id;

      if (typeof parentId !== 'undefined')
        pbrAppearance.parent = parentId;

      WbWorld.instance.nodes.set(pbrAppearance.id, pbrAppearance);
    }

    return pbrAppearance;
  }

  #parseTextureTransform(node, parentId) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    const center = convertStringToVec2(getNodeAttribute(node, 'center', '0 0'));
    const rotation = parseFloat(getNodeAttribute(node, 'rotation', '0'));
    const scale = convertStringToVec2(getNodeAttribute(node, 'scale', '1 1'));
    const translation = convertStringToVec2(getNodeAttribute(node, 'translation', '0 0'));

    const textureTransform = new WbTextureTransform(id, center, rotation, scale, translation);

    if (typeof parentId !== 'undefined')
      textureTransform.parent = parentId;

    WbWorld.instance.nodes.set(textureTransform.id, textureTransform);

    return textureTransform;
  }
}

function loadMeshData(prefix, urls) {
  if (typeof urls === 'undefined')
    return;

  let worldsPath;
  if (typeof webots.currentView.stream === 'undefined')
    worldsPath = '';
  else {
    worldsPath = webots.currentView.stream.view.currentWorld;
    worldsPath = worldsPath.substring(0, worldsPath.lastIndexOf('/')) + '/';
  }

  for (let i = 0; i < urls.length; i++) {
    if (urls[i].startsWith('webots://')) {
      if (typeof webots.currentView.repository === 'undefined')
        webots.currentView.repository = 'cyberbotics';
      if (typeof webots.currentView.branch === 'undefined' || webots.currentView.branch === '')
        webots.currentView.branch = 'released';
      urls[i] = urls[i].replace('webots://', 'https://raw.githubusercontent.com/' + webots.currentView.repository + '/webots/' + webots.currentView.branch + '/');
    }
    if (typeof prefix !== 'undefined' && !urls[i].startsWith('http'))
      urls[i] = prefix + worldsPath + urls[i];
  }
  if (typeof loadMeshData.assimpjs === 'undefined')
    loadMeshData.assimpjs = assimpjs();

  return loadMeshData.assimpjs.then(function(ajs) {
    // fetch the files to import
    return Promise.all(urls.map((file) => fetch(file))).then((responses) => {
      return Promise.all(responses.map((res) => res.arrayBuffer()));
    }).then((arrayBuffers) => {
      // create new file list object, and add the files
      let fileList = new ajs.FileList();
      for (let i = 0; i < urls.length; i++)
        fileList.AddFile(urls[i], new Uint8Array(arrayBuffers[i]));

      // convert file list to assimp json
      let result = ajs.ConvertFileList(fileList, 'assjson', true);

      // check if the conversion succeeded
      if (!result.IsSuccess() || result.FileCount() === 0) {
        console.error(result.GetErrorCode());
        return;
      }

      // get the result file, and convert to string
      let resultFile = result.GetFile(0);
      let jsonContent = new TextDecoder().decode(resultFile.GetContent());

      return JSON.parse(jsonContent);
    });
  });
}

function getNodeAttribute(node, attributeName, defaultValue) {
  console.assert(node && node.attributes);
  if (attributeName in node.attributes)
    return _sanitizeHTML(node.attributes.getNamedItem(attributeName).value);
  return defaultValue;
}

function convertStringToVec2(string) {
  string = convertStringToFloatArray(string);
  return new WbVector2(string[0], string[1]);
}

function convertStringToVec3(string) {
  string = convertStringToFloatArray(string);
  return new WbVector3(string[0], string[1], string[2]);
}

function convertStringToQuaternion(string) {
  string = convertStringToFloatArray(string);
  return new WbVector4(string[0], string[1], string[2], string[3]);
}

function convertStringToFloatArray(string) {
  const stringList = string.replaceAll(',', ' ').split(/\s/).filter(element => element);
  if (stringList)
    return stringList.map(element => parseFloat(element));
}

function _sanitizeHTML(text) {
  const element = document.createElement('div');
  element.innerText = text;
  return element.innerHTML;
}

export {convertStringToVec3, convertStringToQuaternion};
