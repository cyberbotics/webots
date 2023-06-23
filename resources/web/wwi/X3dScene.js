import Parser, {
  convertStringToVec3, convertStringToQuaternion, convertStringToFloatArray,
  convertStringToVec2
} from './Parser.js';
import { webots } from './webots.js';
import WrenRenderer from './WrenRenderer.js';

import WbAbstractCamera from './nodes/WbAbstractCamera.js';
import WbBox from './nodes/WbBox.js';
import WbCadShape from './nodes/WbCadShape.js';
import WbCamera from './nodes/WbCamera.js';
import WbCapsule from './nodes/WbCapsule.js';
import WbColor from './nodes/WbColor.js';
import WbCone from './nodes/WbCone.js';
import WbCoordinate from './nodes/WbCoordinate.js';
import WbCylinder from './nodes/WbCylinder.js';
import WbDistanceSensor from './nodes/WbDistanceSensor.js';
import WbElevationGrid from './nodes/WbElevationGrid.js';
import WbFog from './nodes/WbFog.js';
import WbImageTexture from './nodes/WbImageTexture.js';
import WbIndexedFaceSet from './nodes/WbIndexedFaceSet.js';
import WbIndexedLineSet from './nodes/WbIndexedLineSet.js';
import WbLidar from './nodes/WbLidar.js';
import WbLight from './nodes/WbLight.js';
import WbMaterial from './nodes/WbMaterial.js';
import WbMesh from './nodes/WbMesh.js';
import WbPen from './nodes/WbPen.js';
import WbPbrAppearance from './nodes/WbPbrAppearance.js';
import WbPlane from './nodes/WbPlane.js';
import WbPointLight from './nodes/WbPointLight.js';
import WbPose from './nodes/WbPose.js';
import WbRadar from './nodes/WbRadar.js';
import WbSphere from './nodes/WbSphere.js';
import WbTextureCoordinate from './nodes/WbTextureCoordinate.js';
import WbTextureTransform from './nodes/WbTextureTransform.js';
import WbTrackWheel from './nodes/WbTrackWheel.js';
import WbWorld from './nodes/WbWorld.js';

import WbVector2 from './nodes/utils/WbVector2.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbNormal from './nodes/WbNormal.js';
import WbSpotLight from './nodes/WbSpotLight.js';
import WbDirectionalLight from './nodes/WbDirectionalLight.js';
import WbRangeFinder from './nodes/WbRangeFinder.js';
import WbConnector from './nodes/WbConnector.js';
import WbPropeller from './nodes/WbPropeller.js';
import { findUpperShape } from './nodes/utils/node_utilities.js';

export default class X3dScene {
  #nextRenderingTime;
  #renderingTimeout;
  constructor(domElement) {
    this.domElement = domElement;
    // Each time a render is needed, we ensure that there will be 10 additional renderings to avoid gtao artifacts
    this.remainingRenderings = 10;
  }

  init(texturePathPrefix = '') {
    webots.currentView.prefix = texturePathPrefix;
    this.renderer = new WrenRenderer();

    this.resize();

    this.destroyWorld();
  }

  render(toRemoveGtaoArtifact) {
    // Set maximum rendering frequency.
    // To avoid slowing down the simulation rendering the scene too often, the last rendering time is checked
    // and the rendering is performed only at a given maximum frequency.
    // To be sure that no rendering request is lost, a timeout is set.
    const renderingMinTimeStep = 40; // Rendering maximum frequency: every 40 ms.
    const currentTime = (new Date()).getTime();
    if (this.#nextRenderingTime && this.#nextRenderingTime > currentTime) {
      if (!this.#renderingTimeout)
        this.#renderingTimeout = setTimeout(() => this.render(toRemoveGtaoArtifact), this.#nextRenderingTime - currentTime);
      return;
    }

    this.renderer.render();

    this.#nextRenderingTime = (new Date()).getTime() + renderingMinTimeStep;
    clearTimeout(this.#renderingTimeout);
    this.#renderingTimeout = null;

    if (toRemoveGtaoArtifact)
      --this.remainingRenderings;
    else
      this.remainingRenderings = 10;

    if (this.remainingRenderings > 0)
      setTimeout(() => this.render(true), 80);
  }

  renderMinimal() {
    this.renderer.renderMinimal();
  }

  resize() {
    const width = this.domElement.clientWidth;
    const height = this.domElement.clientHeight;

    this.renderer.setSize(width, height);

    if (typeof WbWorld.instance === 'undefined')
      return;

    WbWorld.instance?.scene.updateFrameBuffer();

    WbWorld.instance?.viewpoint.updatePostProcessingEffects();

    this.render();
  }

  destroyWorld() {
    if (typeof document.getElementsByTagName('webots-view')[0] !== 'undefined' &&
      typeof document.getElementsByTagName('webots-view')[0].toolbar !== 'undefined') {
      const toolbar = document.getElementsByTagName('webots-view')[0].toolbar;
      toolbar.removeRobotWindows();
      toolbar.terminal?.clear();
    }

    if (typeof WbWorld.instance !== 'undefined') {
      let index = WbWorld.instance.root.children.length - 1;
      while (index >= 0) {
        WbWorld.instance.root.children[index].delete();
        --index;
      }

      WbWorld.instance?.scene.destroy();

      WbWorld.instance = undefined;
    }

    this.renderMinimal();
    clearTimeout(this.#renderingTimeout);
  }

  #deleteObject(id) {
    const object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object !== 'undefined')
      object.delete();

    WbWorld.instance.robots.forEach((robot, i) => {
      if (robot.id === 'n' + id)
        WbWorld.instance.robots.splice(i, 1);
    });

    this.render();
  }

  async loadRawWorldFile(raw, onLoad, progress) {
    const prefix = webots.currentView.prefix;
    const parser = new Parser(prefix);
    await parser.parse(raw, this.renderer).then(() => onLoad());
  }

  loadWorldFile(url, onLoad, progress) {
    const prefix = webots.currentView.prefix;
    const renderer = this.renderer;
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async () => {
      // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) {
        const parser = new Parser(prefix);
        await parser.parse(xmlhttp.responseText, renderer).then(() => onLoad());
      } else if (xmlhttp.status === 404)
        progress.setProgressBar('block', 'Loading world file...', 5, '(error) File not found: ' + url);
    };
    xmlhttp.onerror = () => {
      progress.setProgressBar('block', 'Loading world file...', 5, 'An unknown error occurred during the loading...');
    };
    xmlhttp.send();
  }

  async loadObject(x3dObject, parentId, callback) {
    let parentNode;
    if (typeof parentId !== 'undefined')
      parentNode = WbWorld.instance.nodes.get('n' + parentId);

    const parser = new Parser(webots.currentView.prefix);
    parser.prefix = webots.currentView.prefix;
    await parser.parse(x3dObject, this.renderer, false, parentNode, callback);

    const node = WbWorld.instance.nodes.get(parser.rootNodeId);
    if (typeof node === 'undefined')
      return; // can happen for nodes that have no webotsjs counterpart, like Physics

    const shapeAncestor = findUpperShape(node);
    if (typeof shapeAncestor !== 'undefined') {
      shapeAncestor.unfinalize();
      shapeAncestor.finalize();
    } else
      node.finalize();
  }

  applyUpdate(update) {
    let id = update.id;

    if (typeof id === 'string')
      id = id.replace('n', '');

    if (typeof WbWorld.instance === 'undefined')
      return;

    const object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object === 'undefined')
      return;

    this.#applyUpdateToObject(update, object);

    // Update the related USE nodes
    let length = object.useList.length - 1;
    while (length >= 0) {
      const use = WbWorld.instance.nodes.get(object.useList[length]);
      if (typeof use === 'undefined') {
        // remove a USE node from the list if it has been deleted
        const index = object.useList.indexOf(length);
        this.useList.splice(index, 1);
      } else
        this.#applyUpdateToObject(update, use);

      --length;
    }
  }

  #applyUpdateToObject(update, object) {
    for (let key in update) {
      if (key === 'id')
        continue;

      if (key === 'translation') {
        if (object instanceof WbPose)
          object.translation = convertStringToVec3(update[key]);
        else if (object instanceof WbTextureTransform)
          object.translation = convertStringToVec2(update[key]);
      } else if (key === 'rotation') {
        if (object instanceof WbTextureTransform)
          object.rotation = parseFloat(update[key]);
        else {
          const quaternion = convertStringToQuaternion(update[key]);
          if (object instanceof WbTrackWheel)
            object.updateRotation(quaternion);
          else
            object.rotation = quaternion;
        }
      } else if (key === 'scale') {
        if (object instanceof WbPose)
          object.scale = convertStringToVec3(update[key]);
        else if (object instanceof WbTextureTransform)
          object.scale = convertStringToVec2(update[key]);
      } else if (key === 'center') {
        if (object instanceof WbTextureTransform)
          object.center = convertStringToVec2(update[key]);
      } else if (key === 'castShadows') {
        if (object instanceof WbLight || object instanceof WbCadShape)
          object.castShadows = update[key].toLowerCase() === 'true';
      } else if (key === 'isPickable') {
        if (object instanceof WbCadShape)
          object.isPickable = update[key].toLowerCase() === 'true';
      } else if (key === 'size') {
        if (object instanceof WbBox || object instanceof WbPlane)
          object.size = convertStringToVec3(update[key]);
      } else if (key === 'radius') {
        if (object instanceof WbCapsule || object instanceof WbSphere || object instanceof WbCylinder ||
          object instanceof WbSpotLight || object instanceof WbPointLight)
          object.radius = parseFloat(update[key]);
      } else if (key === 'subdivision') {
        if (object instanceof WbSphere || object instanceof WbCapsule || object instanceof WbCone ||
          object instanceof WbCylinder)
          object.subdivision = parseInt(update[key]);
      } else if (key === 'ico') {
        if (object instanceof WbSphere)
          object.ico = update[key].toLowerCase() === 'true';
      } else if (key === 'height') {
        if (object instanceof WbCapsule || object instanceof WbCone || object instanceof WbCylinder)
          object.height = parseFloat(update[key]);
        else if (object instanceof WbElevationGrid)
          // Filter is used to remove Nan elements
          object.height = convertStringToFloatArray(update[key]).filter(e => e);
        else if (object instanceof WbAbstractCamera)
          object.height = parseInt(update[key]);
      } else if (key === 'bottom') {
        if (object instanceof WbCapsule || object instanceof WbCone || object instanceof WbCylinder)
          object.bottom = update[key].toLowerCase() === 'true';
      } else if (key === 'top') {
        if (object instanceof WbCapsule || object instanceof WbCylinder)
          object.top = update[key].toLowerCase() === 'true';
      } else if (key === 'side') {
        if (object instanceof WbCapsule || object instanceof WbCone || object instanceof WbCylinder)
          object.side = update[key].toLowerCase() === 'true';
      } else if (key === 'bottomRadius') {
        if (object instanceof WbCone)
          object.bottomRadius = parseFloat(update[key]);
      } else if (key === 'ccw') {
        if (object instanceof WbMesh || object instanceof WbIndexedFaceSet || object instanceof WbCadShape)
          object.ccw = update[key].toLowerCase() === 'true';
      } else if (key === 'direction') {
        if (object instanceof WbSpotLight || WbDirectionalLight)
          object.direction = convertStringToVec3(update[key]);
      } else if (key === 'color') {
        if (object instanceof WbLight || object instanceof WbFog)
          object.color = convertStringToVec3(update[key]);
        else if (object instanceof WbColor)
          object.color = convertStringToVec3Array(update[key]);
      } else if (key === 'name') {
        if (object instanceof WbMesh)
          object.name = update[key];
      } else if (key === 'materialIndex') {
        if (object instanceof WbMesh)
          object.materialIndex = parseInt(update[key]);
      } else if (key === 'url') {
        if (object instanceof WbMesh || object instanceof WbCadShape || object instanceof WbImageTexture) {
          let urlString = update[key];
          if (urlString === '[]') {
            object.url = undefined;
            return;
          }
          if (urlString.startsWith('[') && urlString.endsWith(']'))
            urlString = urlString.substring(1, urlString.length - 1);
          object.url = urlString.split('"').filter(element => { if (element !== ' ') return element; })[0];
        }
      } else if (key === 'point') {
        if (object instanceof WbCoordinate)
          object.point = convertStringToVec3Array(update[key]);
        if (object instanceof WbTextureCoordinate)
          object.point = convertStringToVec2Array(update[key]);
      } else if (key === 'vector') {
        if (object instanceof WbNormal)
          object.vector = convertStringToVec3Array(update[key]);
      } else if (key === 'coordIndex') {
        if (object instanceof WbIndexedLineSet || object instanceof WbIndexedFaceSet)
          object.coordIndex = update[key];
      } else if (key === 'attenuation') {
        if (object instanceof WbSpotLight || object instanceof WbPointLight)
          object.attenuation = convertStringToVec3(update[key]);
      } else if (key === 'location') {
        if (object instanceof WbSpotLight || object instanceof WbPointLight)
          object.location = convertStringToVec3(update[key]);
      } else if (object instanceof WbFog) {
        if (key === 'visibilityRange')
          object.visibilityRange = parseFloat(update[key]);
        else if (key === 'fogType')
          object.fogType = update[key];
      } else if (object instanceof WbSpotLight) {
        if (key === 'beamWidth')
          object.beamWidth = parseFloat(update[key]);
        else if (key === 'cutOffAngle')
          object.cutOffAngle = parseFloat(update[key]);
      } else if (object instanceof WbIndexedFaceSet) {
        if (key === 'normalPerVertex')
          object.normalPerVertex = update[key].toLowerCase() === 'true';
        else if (key === 'normalIndex')
          object.normalIndex = update[key];
        else if (key === 'texCoordIndex')
          object.texCoordIndex = update[key];
        else if (key === 'creaseAngle')
          object.creaseAngle = parseFloat(update[key]);
      } else if (object instanceof WbElevationGrid) {
        if (key === 'xDimension')
          object.xDimension = update[key];
        else if (key === 'xSpacing')
          object.xSpacing = update[key];
        else if (key === 'yDimension')
          object.yDimension = update[key];
        else if (key === 'ySpacing')
          object.ySpacing = update[key];
        else if (key === 'thickness')
          object.thickness = update[key];
      } else if (object instanceof WbPbrAppearance || object instanceof WbMaterial) {
        if (key === 'baseColor')
          object.baseColor = convertStringToVec3(update[key]);
        else if (key === 'diffuseColor')
          object.diffuseColor = convertStringToVec3(update[key]);
        else if (key === 'emissiveColor')
          object.emissiveColor = convertStringToVec3(update[key]);
        else if (key === 'roughness')
          object.roughness = parseFloat(update[key]);
        else if (key === 'metalness')
          object.metalness = parseFloat(update[key]);
        else if (key === 'IBLStrength')
          object.IBLStrength = parseFloat(update[key]);
        else if (key === 'normalMapFactor')
          object.normalMapFactor = parseFloat(update[key]);
        else if (key === 'occlusionMapStrength')
          object.occlusionMapStrength = parseFloat(update[key]);
        else if (key === 'emissiveIntensity')
          object.emissiveIntensity = parseFloat(update[key]);
        else if (key === 'transparency')
          object.transparency = parseFloat(update[key]);
        else if (key === 'ambientIntensity')
          object.ambientIntensity = parseFloat(update[key]);
        else if (key === 'shininess')
          object.shininess = parseFloat(update[key]);
        else if (key === 'specularColor')
          object.specularColor = convertStringToVec3(update[key]);
      } else if (object instanceof WbImageTexture) {
        if (key === 'repeatS')
          object.repeatS = update[key].toLowerCase() === 'true';
        else if (key === 'repeatT')
          object.repeatT = update[key].toLowerCase() === 'true';
        else if (key === 'filtering')
          object.filtering = parseInt(update[key]);
      } else if (object instanceof WbCamera) {
        if (key === 'far')
          object.far = parseFloat(update[key]);
        else if (key === 'near')
          object.near = parseFloat(update[key]);
      } else if (object instanceof WbRangeFinder) {
        if (key === 'maxRange')
          object.maxRange = parseFloat(update[key]);
        else if (key === 'minRange')
          object.minRange = parseFloat(update[key]);
      } else if (object instanceof WbLidar) {
        if (key === 'maxRange')
          object.maxRange = parseFloat(update[key]);
        else if (key === 'minRange')
          object.minRange = parseFloat(update[key]);
        else if (key === 'horizontalResolution')
          object.horizontalResolution = parseInt(update[key]);
        else if (key === 'numberOfLayers')
          object.numberOfLayers = parseInt(update[key]);
        else if (key === 'tiltAngle')
          object.tiltAngle = parseFloat(update[key]);
        else if (key === 'verticalFieldOfView')
          object.verticalFieldOfView = parseFloat(update[key]);
      } else if (object instanceof WbRadar) {
        if (key === 'maxRange')
          object.maxRange = parseFloat(update[key]);
        else if (key === 'minRange')
          object.minRange = parseFloat(update[key]);
        else if (key === 'verticalFieldOfView')
          object.verticalFieldOfView = parseFloat(update[key]);
        else if (key === 'horizontalFieldOfView')
          object.horizontalFieldOfView = parseFloat(update[key]);
      } else if (object instanceof WbPen) {
        if (key === 'write')
          object.write = update[key].toLowerCase() === 'true';
      } else if (object instanceof WbDistanceSensor) {
        if (key === 'numberOfRays')
          object.numberOfRays = parseInt(update[key]);
        else if (key === 'aperture')
          object.aperture = parseFloat(update[key]);
        else if (key === 'lookupTable') {
          let lookupString = update[key];
          if (lookupString.startsWith('['))
            lookupString = lookupString.substring(1);
          if (lookupString.startsWith(']'))
            lookupString = lookupString.substring(0, lookupString.size - 1);
          const lookupTableArray = convertStringToFloatArray(lookupString);
          const lookupTable = [];
          if (lookupTableArray.length % 3 === 0) {
            for (let i = 0; i < lookupTableArray.length; i = i + 3)
              lookupTable.push(new WbVector3(lookupTableArray[i], lookupTableArray[i + 1], lookupTableArray[i + 2]));
          }
          object.lookupTable = lookupTable;
        }
      } else if (object instanceof WbConnector) {
        if (key === 'numberOfRotations')
          object.numberOfRotations = parseInt(update[key]);
      }

      if (object instanceof WbLight) {
        if (key === 'on')
          object.on = update[key].toLowerCase() === 'true';
        else if (key === 'ambientIntensity')
          object.ambientIntensity = parseFloat(update[key]);
        else if (key === 'intensity')
          object.intensity = parseFloat(update[key]);
      } else if (object instanceof WbAbstractCamera) {
        if (key === 'width')
          object.width = parseInt(update[key]);
        else if (key === 'fieldOfView')
          object.fieldOfView = parseFloat(update[key]);
      }
    }

    if (typeof object.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(object.parent);
      if (typeof parent !== 'undefined' && parent instanceof WbPropeller && parent.currentHelix !== object.id &&
        WbWorld.instance.readyForUpdates)
        parent.switchHelix(object.id);
    }
  }

  applyLabel(label, view) {
    view.setLabel({
      id: label.id,
      text: label.text,
      font: label.font,
      color: label.rgba,
      size: label.size,
      x: label.x,
      y: label.y
    });
  }

  processServerMessage(data, view) {
    if (data.startsWith('application/json:')) {
      if (typeof view.time !== 'undefined') { // otherwise ignore late updates until the scene loading is completed
        data = data.substring(data.indexOf(':') + 1);
        const frame = JSON.parse(data);
        view.time = frame.time;
        if (document.getElementById('webots-clock'))
          document.getElementById('webots-clock').innerHTML = webots.parseMillisecondsIntoReadableTime(frame.time);

        if (frame.hasOwnProperty('updates')) {
          for (let i = 0; i < frame.updates.length; i++)
            this.applyUpdate(frame.updates[i]);
          WbWorld.instance.tracks.forEach(track => {
            if (track.linearSpeed !== 0) {
              track.animateMesh();
              track.linearSpeed = 0;
            }
          });
        }

        if (frame.hasOwnProperty('labels')) {
          for (let i = 0; i < frame.labels.length; i++)
            this.applyLabel(frame.labels[i], view);
        }

        WbWorld.instance?.viewpoint.updateFollowUp(view.time);
        this.render();
      } else { // parse the labels even so the scene loading is not completed
        data = data.substring(data.indexOf(':') + 1);
        let frame = JSON.parse(data);
        if (frame.hasOwnProperty('labels')) {
          for (let i = 0; i < frame.labels.length; i++)
            this.applyLabel(frame.labels[i], view);
        }
      }
    } else if (data.startsWith('node:')) {
      data = data.substring(data.indexOf(':') + 1);
      const parentId = data.split(':')[0];
      data = data.substring(data.indexOf(':') + 1);
      this.loadObject(data, parentId === '0' ? WbWorld.instance.root.id.replace('n', '') : parentId);
    } else if (data.startsWith('delete:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      this.#deleteObject(data);
    } else if (data.startsWith('model:')) {
      this.destroyWorld();
      view.removeLabels();
      data = data.substring(data.indexOf(':') + 1).trim();
      if (!data) // received an empty model case: just destroy the view
        return true;
      view.stream.socket.send('pause');
      view.progress.setProgressBar('block', 'same', 0, 'Loading 3D scene...');
      this.loadRawWorldFile(data, view.onready);
    } else
      return false;
    return true;
  }

  resetViewpoint() {
    WbWorld.instance.viewpoint.resetViewpoint();
    this.render();
  }
}

function convertStringToVec3Array(string) {
  const vecArray = [];
  string = string.trim();
  string = string.slice(1, -1).trim();
  const floatArray = convertStringToFloatArray(string);
  if (typeof floatArray !== 'undefined') {
    for (let i = 0; i < floatArray.length; i += 3)
      vecArray.push(new WbVector3(floatArray[i], floatArray[i + 1], floatArray[i + 2]));
  }

  return vecArray;
}

function convertStringToVec2Array(string) {
  const vecArray = [];
  string = string.trim();
  string = string.slice(1, -1).trim();
  const floatArray = convertStringToFloatArray(string);
  if (typeof floatArray !== 'undefined') {
    for (let i = 0; i < floatArray.length; i += 2)
      vecArray.push(new WbVector2(floatArray[i], floatArray[i + 1]));
  }

  return vecArray;
}
