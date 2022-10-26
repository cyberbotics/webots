import {M_PI_4} from './nodes/utils/constants.js';
import WbAbstractAppearance from './nodes/WbAbstractAppearance.js';
import WbAccelerometer from './nodes/WbAccelerometer.js';
import WbAltimeter from './nodes/WbAltimeter.js';
import WbAppearance from './nodes/WbAppearance.js';
import WbBackground from './nodes/WbBackground.js';
import WbBallJoint from './nodes/WbBallJoint.js';
import WbBallJointParameters from './nodes/WbBallJointParameters.js';
import WbBillboard from './nodes/WbBillboard.js';
import WbBox from './nodes/WbBox.js';
import WbBrake from './nodes/WbBrake.js';
import WbCadShape from './nodes/WbCadShape.js';
import WbCamera from './nodes/WbCamera.js';
import WbCapsule from './nodes/WbCapsule.js';
import WbCharger from './nodes/WbCharger.js';
import WbColor from './nodes/WbColor.js';
import WbCompass from './nodes/WbCompass.js';
import WbCone from './nodes/WbCone.js';
import WbConnector from './nodes/WbConnector.js';
import WbCoordinate from './nodes/WbCoordinate.js';
import WbCylinder from './nodes/WbCylinder.js';
import WbDirectionalLight from './nodes/WbDirectionalLight.js';
import WbDisplay from './nodes/WbDisplay.js';
import WbDistanceSensor from './nodes/WbDistanceSensor.js';
import WbElevationGrid from './nodes/WbElevationGrid.js';
import WbEmitter from './nodes/WbEmitter.js';
import WbFog from './nodes/WbFog.js';
import WbGeometry from './nodes/WbGeometry.js';
import WbGps from './nodes/WbGps.js';
import WbGroup from './nodes/WbGroup.js';
import WbGyro from './nodes/WbGyro.js';
import WbHingeJoint from './nodes/WbHingeJoint.js';
import WbHingeJointParameters from './nodes/WbHingeJointParameters.js';
import WbHinge2Joint from './nodes/WbHinge2Joint.js';
import WbImageTexture from './nodes/WbImageTexture.js';
import WbIndexedFaceSet from './nodes/WbIndexedFaceSet.js';
import WbIndexedLineSet from './nodes/WbIndexedLineSet.js';
import WbInertialUnit from './nodes/WbInertialUnit.js';
import WbJoint from './nodes/WbJoint.js';
import WbJoinParameters from './nodes/WbJointParameters.js';
import WbLed from './nodes/WbLed.js';
import WbLidar from './nodes/WbLidar.js';
import WbLight from './nodes/WbLight.js';
import WbLightSensor from './nodes/WbLightSensor.js';
import WbLinearMotor from './nodes/WbLinearMotor.js';
import WbMaterial from './nodes/WbMaterial.js';
import WbMesh from './nodes/WbMesh.js';
import WbNormal from './nodes/WbNormal.js';
import WbPbrAppearance from './nodes/WbPbrAppearance.js';
import WbPen from './nodes/WbPen.js';
import WbPlane from './nodes/WbPlane.js';
import WbPointLight from './nodes/WbPointLight.js';
import WbPointSet from './nodes/WbPointSet.js';
import WbPositionSensor from './nodes/WbPositionSensor.js';
import WbRadar from './nodes/WbRadar.js';
import WbRangeFinder from './nodes/WbRangeFinder.js';
import WbReceiver from './nodes/WbReceiver.js';
import WbRotationalMotor from './nodes/WbRotationalMotor.js';
import WbScene from './nodes/WbScene.js';
import WbShape from './nodes/WbShape.js';
import WbSlot from './nodes/WbSlot.js';
import WbSolid from './nodes/WbSolid.js';
import WbSpeaker from './nodes/WbSpeaker.js';
import WbSphere from './nodes/WbSphere.js';
import WbSpotLight from './nodes/WbSpotLight.js';
import WbSliderJoint from './nodes/WbSliderJoint.js';
import WbTextureCoordinate from './nodes/WbTextureCoordinate.js';
import WbTextureTransform from './nodes/WbTextureTransform.js';
import WbTouchSensor from './nodes/WbTouchSensor.js';
import WbTrack from './nodes/WbTrack.js';
import WbTrackWheel from './nodes/WbTrackWheel.js';
import WbTransform from './nodes/WbTransform.js';
import WbVector2 from './nodes/utils/WbVector2.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbVector4 from './nodes/utils/WbVector4.js';
import WbViewpoint from './nodes/WbViewpoint.js';
import WbWorld from './nodes/WbWorld.js';
import WbWrenPostProcessingEffects from './wren/WbWrenPostProcessingEffects.js';

import {getAnId} from './nodes/utils/id_provider.js';

import DefaultUrl from './DefaultUrl.js';
import {webots} from './webots.js';
import {loadImageTextureInWren, loadTextureData} from './image_loader.js';
import {loadMeshData} from './mesh_loader.js';

/*
  This module takes an x3d world, parse it and populate the scene.
*/
export default class Parser {
  #downloadingImage;
  #nodeCounter;
  #nodeNumber;
  #promises;
  #promiseCounter;
  #promiseNumber;
  constructor(prefix = '') {
    this.prefix = prefix;
    this.#downloadingImage = new Set();
    this.#promises = [];
    this.#promiseCounter = 0;
    this.#promiseNumber = 0;
    WbWorld.init();
    WbWorld.instance.prefix = prefix;
  }

  parse(text, renderer, parent, callback) {
    webots.currentView.progress.setProgressBar('Connecting to webots instance...', 'same', 60 + 0.1 * 30, 'Parsing object...');
    let xml = null;
    if (window.DOMParser) {
      const parser = new DOMParser();
      xml = parser.parseFromString(text, 'text/xml');
    }
    console.log(xml);
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
        const info = 'Finalizing node ' + node.id + ': ' + (100 * (i + 1) / WbWorld.instance.sceneTree.length) + '%';
        webots.currentView.progress.setProgressBar('block', 'same', 75 + 0.25 * percentage, info);
        node.finalize();
      });

      WbWorld.instance.readyForUpdates = true;

      webots.currentView.x3dScene.resize();
      renderer.render();
      setTimeout(() => { webots.currentView.progress.setProgressBar('none'); }, 300);

      if (typeof callback === 'function')
        callback();
      console.log(WbWorld.instance)
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
    switch (node.tagName) {
      case 'Scene':
        this.#parseScene(node);
        this.#parseChildren(node, parentNode);
        break;
      case 'WorldInfo':
        this.#parseWorldInfo(node);
        break;
      case 'Viewpoint':
        WbWorld.instance.viewpoint = this.#parseViewpoint(node);
        break;
      case 'Background':
        result = this.#parseBackground(node);
        break;
      case 'HingeJoint':
      case 'SliderJoint':
      case 'Hinge2Joint':
      case 'BallJoint':
        result = this.#parseJoint(node, parentNode);
        break;
      case 'HingeJointParameters':
      case 'JointParameters':
      case 'BallJointParameters':
        result = this.#parseJointParameters(node, parentNode);
        break;
      case 'PositionSensor':
      case 'Brake':
      case 'RotationalMotor':
      case 'LinearMotor':
        result = this.#parseLogicalDevice(node, parentNode);
        break;
      case 'Billboard':
        result = this.#parseBillboard(node, parentNode);
        break;
      case 'Group':
      case 'Propeller':
        result = this.#parseGroup(node, parentNode);
        break;
      case 'Shape':
        result = this.#parseShape(node, parentNode, isBoundingObject);
        break;
      case 'Slot':
        result = this.#parseSlot(node, parentNode);
        break;
      case 'CadShape':
        result = this.#parseCadShape(node, parentNode);
        break;
      case 'DirectionalLight':
        result = this.#parseDirectionalLight(node, parentNode);
        break;
      case 'PointLight':
        result = this.#parsePointLight(node, parentNode);
        break;
      case 'SpotLight':
        result = this.#parseSpotLight(node, parentNode);
        break;
      case 'Fog':
        if (!WbWorld.instance.hasFog)
          result = this.#parseFog(node);
        else
          console.error('This world already has a fog.');
        break;
      case 'Transform':
      case 'Robot':
      case 'Solid':
      case 'Accelerometer':
      case 'Altimeter':
      case 'Camera':
      case 'Charger':
      case 'Compass':
      case 'Connector':
      case 'Display':
      case 'DistanceSensor':
      case 'Emitter':
      case 'GPS':
      case 'Gyro':
      case 'InertialUnit':
      case 'LED':
      case 'Lidar':
      case 'LightSensor':
      case 'Pen':
      case 'Radar':
      case 'RangeFinder':
      case 'Receiver':
      case 'Speaker':
      case 'TouchSensor':
      case 'Track':
      case 'TrackWheel':
        result = this.#parseTransform(node, parentNode, isBoundingObject);
        break;
      case 'Physics':
      case 'ImmersionProperties':
        // skip those nodes as they are not needed for web representation.
        break;
      default:
        // Either it is a node added after the whole scene, or it is an unknown node, or a geometry bounding object
        let id;
        if (typeof parentNode !== 'undefined')
          id = parentNode.id;
        result = this.#parseGeometry(node, id);
        // We are forced to check if the result correspond to the class we expect because of the case of a USE
        if (typeof result !== 'undefined' && result instanceof WbGeometry) {
          if (typeof parentNode !== 'undefined') {
            if (parentNode instanceof WbShape) {
              parentNode.geometry?.delete();
              parentNode.geometry = result;
            } else if (parentNode instanceof WbSolid || parentNode instanceof WbTransform || parentNode instanceof WbGroup) {
              // Bounding object
              parentNode.boundingObject?.delete();
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
            parentNode.appearance?.delete();
            result = this.#parsePbrAppearance(node, id);
            parentNode.appearance = result;
          }
        } else if (node.tagName === 'Appearance') {
          if (typeof parentNode !== 'undefined' && parentNode instanceof WbShape) {
            parentNode.appearance?.delete();
            result = this.#parseAppearance(node, id);
            parentNode.appearance = result;
          }
        } else if (node.tagName === 'Material') {
          result = this.#parseMaterial(node, id);
          if (typeof result !== 'undefined') {
            if (typeof parentNode !== 'undefined' && parentNode instanceof WbAppearance) {
              parentNode.material?.delete();
              parentNode.material = result;
            }
          }
        } else if (node.tagName === 'ImageTexture') {
          result = this.#parseImageTexture(node, id);
          if (typeof result !== 'undefined') {
            if (typeof parentNode !== 'undefined' && parentNode instanceof WbAppearance) {
              parentNode.texture?.delete();
              parentNode.texture = result;
            }
          }
        } else if (node.tagName === 'TextureTransform') {
          result = this.#parseTextureTransform(node, id);
          if (typeof result !== 'undefined') {
            if (typeof parentNode !== 'undefined' && parentNode instanceof WbAbstractAppearance) {
              parentNode.textureTransform?.delete();
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
        this.#promises.push(loadTextureData(this.prefix, backgroundUrl[backgroundIdx[i]], false, rotationValues[i])
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
        this.#promises.push(loadTextureData(this.prefix, backgroundIrradianceUrl[backgroundIdx[i]], true, rotationValues[i])
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
    let name = getNodeAttribute(node, 'name', '');

    let newNode;
    if (type === 'track' || node.tagName === 'Track') {
      const geometriesCount = parseInt(getNodeAttribute(node, 'geometriesCount', '10'));
      newNode = new WbTrack(id, translation, scale, rotation, geometriesCount);
    } else if (type === 'trackwheel' || node.tagName === 'Trackwheel') {
      const radius = parseFloat(getNodeAttribute(node, 'radius', '0.1'));
      const inner = getNodeAttribute(node, 'inner', '0').toLowerCase() === '1';

      newNode = new WbTrackWheel(id, translation, scale, rotation, radius, inner);

      parentNode.wheelsList.push(newNode);
    } else if (node.tagName === 'Robot' || node.tagName === 'Solid' || type === 'solid' || type === 'robot') {
      if (name === '') {
        if (node.tagName === 'Robot' || type === 'robot')
          name = 'robot';
        else
          name = 'solid';
      }
      newNode = new WbSolid(id, translation, scale, rotation, name);
    } else if (node.tagName === 'Accelerometer')
      newNode = new WbAccelerometer(id, translation, scale, rotation, name === '' ? 'accelerometer' : name);
    else if (node.tagName === 'Altimeter')
      newNode = new WbAltimeter(id, translation, scale, rotation, name === '' ? 'altimeter' : name);
    else if (node.tagName === 'Camera')
      newNode = new WbCamera(id, translation, scale, rotation, name === '' ? 'camera' : name);
    else if (node.tagName === 'Charger')
      newNode = new WbCharger(id, translation, scale, rotation, name === '' ? 'charger' : name);
    else if (node.tagName === 'Compass')
      newNode = new WbCompass(id, translation, scale, rotation, name === '' ? 'compass' : name);
    else if (node.tagName === 'Connector')
      newNode = new WbConnector(id, translation, scale, rotation, name === '' ? 'connector' : name);
    else if (node.tagName === 'Display')
      newNode = new WbDisplay(id, translation, scale, rotation, name === '' ? 'display' : name);
    else if (node.tagName === 'DistanceSensor')
      newNode = new WbDistanceSensor(id, translation, scale, rotation, name === '' ? 'distance sensor' : name);
    else if (node.tagName === 'Emitter')
      newNode = new WbEmitter(id, translation, scale, rotation, name === '' ? 'emitter' : name);
    else if (node.tagName === 'GPS')
      newNode = new WbGps(id, translation, scale, rotation, name === '' ? 'gps' : name);
    else if (node.tagName === 'Gyro')
      newNode = new WbGyro(id, translation, scale, rotation, name === '' ? 'gyro' : name);
    else if (node.tagName === 'InertialUnit')
      newNode = new WbInertialUnit(id, translation, scale, rotation, name === '' ? 'inertial unit' : name);
    else if (node.tagName === 'LED')
      newNode = new WbLed(id, translation, scale, rotation, name === '' ? 'led' : name);
    else if (node.tagName === 'Lidar')
      newNode = new WbLidar(id, translation, scale, rotation, name === '' ? 'lidar' : name);
    else if (node.tagName === 'LightSensor')
      newNode = new WbLightSensor(id, translation, scale, rotation, name === '' ? 'light sensor' : name);
    else if (node.tagName === 'Pen')
      newNode = new WbPen(id, translation, scale, rotation, name === '' ? 'pen' : name);
    else if (node.tagName === 'Radar')
      newNode = new WbRadar(id, translation, scale, rotation, name === '' ? 'radar' : name);
    else if (node.tagName === 'RangeFinder')
      newNode = new WbRangeFinder(id, translation, scale, rotation, name === '' ? 'range finder' : name);
    else if (node.tagName === 'Receiver')
      newNode = new WbReceiver(id, translation, scale, rotation, name === '' ? 'receiver' : name);
    else if (node.tagName === 'Speaker')
      newNode = new WbSpeaker(id, translation, scale, rotation, name === '' ? 'speaker' : name);
    else if (node.tagName === 'TouchSensor')
      newNode = new WbTouchSensor(id, translation, scale, rotation, name === '' ? 'touch sensor' : name);
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
      else if (parentNode instanceof WbSlot || parentNode instanceof WbJoint)
        parentNode.endPoint = newNode;
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

    const isPropeller = getNodeAttribute(node, 'type', '').toLowerCase() === 'propeller' || node.tagName === 'Propeller';
    const isBoundingObject = getNodeAttribute(node, 'role', undefined) === 'boundingObject';

    const group = new WbGroup(id, isPropeller);
    WbWorld.instance.nodes.set(group.id, group);
    this.#parseChildren(node, group, isBoundingObject);

    if (typeof parentNode !== 'undefined') {
      group.parent = parentNode.id;
      if (isBoundingObject && parentNode instanceof WbSolid)
        parentNode.boundingObject = group;
      else if (parentNode instanceof WbSlot || parentNode instanceof WbJoint)
        parentNode.endPoint = group;
      else
        parentNode.children.push(group);
    }

    return group;
  }

  #parseSlot(node, parentNode) {
    const use = this.#checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    const type = getNodeAttribute(node, 'type', '');
    const slot = new WbSlot(id, type);
    WbWorld.instance.nodes.set(slot.id, slot);
    this.#parseChildren(node, slot);

    if (typeof parentNode !== 'undefined') {
      slot.parent = parentNode.id;
      if (parentNode instanceof WbSlot || parentNode instanceof WbJoint)
        parentNode.endPoint = slot;
      else
        parentNode.children.push(slot);
    }

    return slot;
  }

  #parseJoint(node, parentNode) {
    const id = this.#parseId(node);

    let joint;
    if (node.tagName === 'HingeJoint')
      joint = new WbHingeJoint(id);
    else if (node.tagName === 'SliderJoint')
      joint = new WbSliderJoint(id);
    else if (node.tagName === 'Hinge2Joint')
      joint = new WbHinge2Joint(id);
    else if (node.tagName === 'BallJoint')
      joint = new WbBallJoint(id);

    WbWorld.instance.nodes.set(joint.id, joint);
    this.#parseChildren(node, joint);

    if (typeof parentNode !== 'undefined') {
      joint.parent = parentNode.id;
      if (parentNode instanceof WbSlot || parentNode instanceof WbJoint)
        parentNode.endPoint = joint;
      else
        parentNode.children.push(joint);
    }

    return joint;
  }

  #parseJointParameters(node, parentNode) {
    const id = this.#parseId(node);

    const position = parseFloat(getNodeAttribute(node, 'position', '0'));
    const anchor = convertStringToVec3(getNodeAttribute(node, 'anchor', '0 0 0'));
    const minStop = parseFloat(getNodeAttribute(node, 'minStop', '0'));
    const maxStop = parseFloat(getNodeAttribute(node, 'maxStop', '0'));

    let jointParameters;
    if (node.tagName === 'JointParameters') {
      const axis = convertStringToVec3(getNodeAttribute(node, 'axis', '0 0 1'));
      jointParameters = new WbJoinParameters(id, position, axis, minStop, maxStop);
    } else if (node.tagName === 'HingeJointParameters') {
      const axis = convertStringToVec3(getNodeAttribute(node, 'axis', '1 0 0'));
      jointParameters = new WbHingeJointParameters(id, position, axis, anchor, minStop, maxStop);
    } else
      jointParameters = new WbBallJointParameters(id, position, undefined, anchor, minStop, maxStop);

    WbWorld.instance.nodes.set(jointParameters.id, jointParameters);

    if (typeof parentNode !== 'undefined') {
      if (parentNode instanceof WbBallJoint) {
        if (jointParameters instanceof WbBallJointParameters)
          parentNode.jointParameters = jointParameters;
        else
          parentNode.jointParameters2 = jointParameters;
      } else if (parentNode instanceof WbHinge2Joint) {
        if (jointParameters instanceof WbHingeJointParameters)
          parentNode.jointParameters = jointParameters;
        else
          parentNode.jointParameters2 = jointParameters;
      } else
        parentNode.jointParameters = jointParameters;
      jointParameters.parent = parentNode.id;
    }

    return jointParameters;
  }

  #parseLogicalDevice(node, parentNode) {
    const id = this.#parseId(node);

    const name = getNodeAttribute(node, 'name', '');

    let logicalDevice;
    if (node.tagName === 'PositionSensor')
      logicalDevice = new WbPositionSensor(id, name);
    else if (node.tagName === 'Brake')
      logicalDevice = new WbBrake(id, name);
    else if (node.tagName === 'RotationalMotor' || node.tagName === 'LinearMotor') {
      const minPosition = parseFloat(getNodeAttribute(node, 'minPosition', '0'));
      const maxPosition = parseFloat(getNodeAttribute(node, 'maxPosition', '0'));
      if (node.tagName === 'RotationalMotor')
        logicalDevice = new WbRotationalMotor(id, name, minPosition, maxPosition);
      else
        logicalDevice = new WbLinearMotor(id, name, minPosition, maxPosition);
    }

    WbWorld.instance.nodes.set(logicalDevice.id, logicalDevice);

    if (typeof parentNode !== 'undefined') {
      logicalDevice.parent = parentNode.id;
      parentNode.device.push(logicalDevice);
    }

    return logicalDevice;
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
      else if (parentNode instanceof WbSlot || parentNode instanceof WbJoint)
        parentNode.endPoint = shape;
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

    let url = getNodeAttribute(node, 'url', '');
    if (typeof url !== 'undefined')
      url = url.split('"').filter(element => { if (element !== ' ') return element; })[0]; // filter removes empty elements

    const ccw = getNodeAttribute(node, 'ccw', 'true').toLowerCase() === 'true';
    const castShadows = getNodeAttribute(node, 'castShadows', 'true').toLowerCase() === 'true';
    const isPickable = getNodeAttribute(node, 'isPickable', 'true').toLowerCase() === 'true';

    const cadShape = new WbCadShape(id, url, ccw, castShadows, isPickable, this.prefix);

    WbWorld.instance.nodes.set(cadShape.id, cadShape);

    if (typeof parentNode !== 'undefined') {
      cadShape.parent = parentNode.id;
      if (parentNode instanceof WbSlot || parentNode instanceof WbJoint)
        parentNode.endPoint = cadShape;
      else
        parentNode.children.push(cadShape);
    }

    this.#promises.push(loadMeshData(this.prefix, url).then(meshContent => {
      cadShape.scene = meshContent[0];
      cadShape.materialPath = meshContent[1];
      console.log(meshContent)
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
      dirLight.parent = parentNode.id;
      if (parentNode instanceof WbSlot || parentNode instanceof WbJoint)
        parentNode.endPoint = dirLight;
      else
        parentNode.children.push(dirLight);
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

    if (typeof parentNode !== 'undefined' && typeof pointLight !== 'undefined') {
      if (parentNode instanceof WbSlot || parentNode instanceof WbJoint)
        parentNode.endPoint = pointLight;
      else
        parentNode.children.push(pointLight);
    }
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
    const coordIndex = convertStringToFloatArray(getNodeAttribute(node, 'coordIndex', ''));
    const normalIndex = convertStringToFloatArray(getNodeAttribute(node, 'normalIndex', ''));
    const texCoordIndex = convertStringToFloatArray(getNodeAttribute(node, 'texCoordIndex', ''));

    const coordinateNode = node.getElementsByTagName('Coordinate');
    let coord;
    if (coordinateNode.length > 0)
      coord = this.#parseCoordinate(coordinateNode[0]);

    const textureCoordinateNode = node.getElementsByTagName('TextureCoordinate');
    let texCoord;
    if (textureCoordinateNode.length > 0)
      texCoord = this.#parseTextureCoordinate(textureCoordinateNode[0]);

    const normalNode = node.getElementsByTagName('Normal');
    let normal;
    if (normalNode.length > 0)
      normal = this.#parseNormal(normalNode[0]);

    const ccw = getNodeAttribute(node, 'ccw', 'true').toLowerCase() === 'true';
    const normalPerVertex = getNodeAttribute(node, 'normalPerVertex', 'true').toLowerCase() === 'true';
    const creaseAngle = parseFloat(getNodeAttribute(node, 'creaseAngle', '0'));
    const ifs = new WbIndexedFaceSet(id, coordIndex, normalIndex, texCoordIndex, coord, texCoord, normal, ccw,
      creaseAngle, normalPerVertex);

    WbWorld.instance.nodes.set(ifs.id, ifs);

    if (typeof coord !== 'undefined')
      coord.parent = ifs.id;

    if (typeof texCoord !== 'undefined')
      texCoord.parent = ifs.id;

    if (typeof normal !== 'undefined')
      normal.parent = ifs.id;
    return ifs;
  }

  #parseIndexedLineSet(node, id) {
    const coordinateNode = node.getElementsByTagName('Coordinate');
    let coord;
    if (coordinateNode.length > 0)
      coord = this.#parseCoordinate(coordinateNode[0]);

    let coordIndex = [];
    const coordinateIndex = getNodeAttribute(node, 'coordIndex', '');
    if (typeof coordinateIndex !== 'undefined') {
      const indicesStr = convertStringToFloatArray(coordinateIndex);
      coordIndex = indicesStr.map(Number);
    }

    const ils = new WbIndexedLineSet(id, coord, coordIndex);
    WbWorld.instance.nodes.set(ils.id, ils);

    if (typeof coord !== 'undefined')
      coord.parent = ils.id;

    return ils;
  }

  #parseElevationGrid(node, id) {
    const heightStr = getNodeAttribute(node, 'height');
    const xDimension = parseInt(getNodeAttribute(node, 'xDimension', '0'));
    const xSpacing = parseFloat(getNodeAttribute(node, 'xSpacing', '1'));
    const yDimension = parseInt(getNodeAttribute(node, 'yDimension', '0'));
    const ySpacing = parseFloat(getNodeAttribute(node, 'ySpacing', '1'));
    const thickness = parseFloat(getNodeAttribute(node, 'thickness', '1'));

    let height;
    if (typeof heightStr !== 'undefined')
      height = convertStringToFloatArray(heightStr);
    else
      height = [];

    const eg = new WbElevationGrid(id, height, xDimension, xSpacing, yDimension, ySpacing, thickness);
    WbWorld.instance.nodes.set(eg.id, eg);

    return eg;
  }

  #parsePointSet(node, id) {
    const coordinateNode = node.getElementsByTagName('Coordinate');
    let coord;
    if (coordinateNode.length > 0)
      coord = this.#parseCoordinate(coordinateNode[0]);

    let colorNode = node.getElementsByTagName('Color');
    let color;
    if (colorNode.length > 0)
      color = this.#parseColor(colorNode[0]);

    const ps = new WbPointSet(id, coord, color);

    if (typeof color !== 'undefined')
      color.parent = ps.id;

    if (typeof coord !== 'undefined')
      coord.parent = ps.id;

    WbWorld.instance.nodes.set(ps.id, ps);

    return ps;
  }

  #parseColor(node) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    let colors = [];
    const colorArray = convertStringToFloatArray(getNodeAttribute(node, 'color', ''));
    if (typeof colorArray !== 'undefined') {
      for (let i = 0; i < colorArray.length; i += 3)
        colors.push(new WbVector3(colorArray[i], colorArray[i + 1], colorArray[i + 2]));
    }

    const color = new WbColor(id, colors);
    WbWorld.instance.nodes.set(color.id, color);
    return color;
  }

  #parseCoordinate(node) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    let point = [];
    const pointArray = convertStringToFloatArray(getNodeAttribute(node, 'point', ''));
    if (typeof pointArray !== 'undefined') {
      for (let i = 0; i < pointArray.length; i += 3)
        point.push(new WbVector3(pointArray[i], pointArray[i + 1], pointArray[i + 2]));
    }

    const coordinate = new WbCoordinate(id, point);
    WbWorld.instance.nodes.set(coordinate.id, coordinate);
    return coordinate;
  }

  #parseTextureCoordinate(node) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    let point = [];
    const pointArray = convertStringToFloatArray(getNodeAttribute(node, 'point', ''));
    if (typeof pointArray !== 'undefined') {
      for (let i = 0; i < pointArray.length; i += 2)
        point.push(new WbVector2(pointArray[i], pointArray[i + 1]));
    }

    const textureCoordinate = new WbTextureCoordinate(id, point);
    WbWorld.instance.nodes.set(textureCoordinate.id, textureCoordinate);
    return textureCoordinate;
  }

  #parseNormal(node) {
    const use = this.#checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this.#parseId(node);
    let point = [];
    const pointArray = convertStringToFloatArray(getNodeAttribute(node, 'vector', ''));
    if (typeof pointArray !== 'undefined') {
      for (let i = 0; i < pointArray.length; i += 3)
        point.push(new WbVector3(pointArray[i], pointArray[i + 1], pointArray[i + 2]));
    }

    const normal = new WbNormal(id, point);
    WbWorld.instance.nodes.set(normal.id, normal);
    return normal;
  }

  #parseMesh(node, id) {
    let url = getNodeAttribute(node, 'url', '');
    if (typeof url !== 'undefined')
      url = url.split('"').filter(element => { if (element !== ' ') return element; })[0]; // filter removes empty elements

    const ccw = getNodeAttribute(node, 'ccw', 'true').toLowerCase() === 'true';
    const name = getNodeAttribute(node, 'name', '');
    const materialIndex = parseInt(getNodeAttribute(node, 'materialIndex', -1));

    const mesh = new WbMesh(id, url, ccw, name, materialIndex);
    WbWorld.instance.nodes.set(mesh.id, mesh);
    if (url) {
      this.#promises.push(loadMeshData(this.prefix, url).then(meshContent => {
        mesh.scene = meshContent[0];
        for (let i = 0; i < mesh.useList.length; i++) {
          const node = WbWorld.instance.nodes.get(mesh.useList[i]);
          node.scene = meshContent;
        }
        this.#updatePromiseCounter('Downloading assets: Mesh \'mesh ' + name + '\'...');
      }));
      this.#promiseNumber += 1;
    }

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
        this.#promises.push(loadImageTextureInWren(this.prefix, url, isTransparent));
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

export {convertStringToVec2, convertStringToVec3, convertStringToQuaternion, convertStringToFloatArray};
