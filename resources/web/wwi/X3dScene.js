import Parser, {convertStringToVec3, convertStringToQuaternion, convertStringToFloatArray} from './Parser.js';
import {webots} from './webots.js';
import WrenRenderer from './WrenRenderer.js';

import WbBox from './nodes/WbBox.js';
import WbCapsule from './nodes/WbCapsule.js';
import WbCone from './nodes/WbCone.js';
import WbCoordinate from './nodes/WbCoordinate.js';
import WbCylinder from './nodes/WbCylinder.js';
import WbElevationGrid from './nodes/WbElevationGrid.js';
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
import WbColor from './nodes/WbColor.js';
import WbSphere from './nodes/WbSphere.js';
import WbTextureCoordinate from './nodes/WbTextureCoordinate.js';
import WbTextureTransform from './nodes/WbTextureTransform.js';
import WbTrackWheel from './nodes/WbTrackWheel.js';
import WbTransform from './nodes/WbTransform.js';
import WbWorld from './nodes/WbWorld.js';

import {getAncestor} from './nodes/utils/utils.js';
import WbVector2 from './nodes/utils/WbVector2.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbNormal from './nodes/WbNormal.js';
import WbSpotLight from './nodes/WbSpotLight.js';
import WbDirectionalLight from './nodes/WbDirectionalLight.js';

export default class X3dScene {
  #loader;
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

    if (typeof WbWorld.instance.scene !== 'undefined')
      WbWorld.instance.scene.updateFrameBuffer();

    if (typeof WbWorld.instance.viewpoint !== 'undefined')
      WbWorld.instance.viewpoint.updatePostProcessingEffects();

    this.render();
  }

  destroyWorld() {
    if (typeof document.getElementsByTagName('webots-view')[0] !== 'undefined' &&
      typeof document.getElementsByTagName('webots-view')[0].toolbar !== 'undefined') {
      const toolbar = document.getElementsByTagName('webots-view')[0].toolbar;
      toolbar.removeRobotWindows();
      if (typeof toolbar.terminal !== 'undefined')
        toolbar.terminal.clear();
    }

    if (typeof WbWorld.instance !== 'undefined') {
      let index = WbWorld.instance.sceneTree.length - 1;
      while (index >= 0) {
        WbWorld.instance.sceneTree[index].delete();
        --index;
      }

      if (typeof WbWorld.instance.viewpoint !== 'undefined')
        WbWorld.instance.viewpoint.delete();

      if (typeof WbWorld.instance.scene !== 'undefined')
        WbWorld.instance.scene.destroy();

      WbWorld.instance = undefined;
    }

    this.renderMinimal();
    clearTimeout(this.#renderingTimeout);
    this.#loader = undefined;
  }

  #deleteObject(id) {
    const object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object === 'undefined')
      return;

    object.delete();

    WbWorld.instance.robots.forEach((robot, i) => {
      if (robot.id === 'n' + id)
        WbWorld.instance.robots.splice(i, 1);
    });

    this.render();
  }

  async loadRawWorldFile(raw, onLoad, progress) {
    const prefix = webots.currentView.prefix;
    this.#loader = new Parser(prefix);
    await this.#loader.parse(raw, this.renderer);
    onLoad();
  }

  loadWorldFile(url, onLoad, progress) {
    const prefix = webots.currentView.prefix;
    const renderer = this.renderer;
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = () => {
      // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) {
        this.#loader = new Parser(prefix);
        this.#loader.parse(xmlhttp.responseText, renderer).then(onLoad());
      } else if (xmlhttp.status === 404)
        progress.setProgressBar('block', 'Loading world file...', 5, '(error) File not found: ' + url);
    };
    xmlhttp.onerror = () => {
      progress.setProgressBar('block', 'Loading world file...', 5, 'An unknown error occurred during the loading...');
    };
    xmlhttp.send();
  }

  loadObject(x3dObject, parentId, callback) {
    let parentNode;
    if (typeof parentId !== 'undefined' && parentId > 0) {
      parentNode = WbWorld.instance.nodes.get('n' + parentId);
      const ancestor = getAncestor(parentNode);
      ancestor.isPreFinalizedCalled = false;
      ancestor.wrenObjectsCreatedCalled = false;
      ancestor.isPostFinalizedCalled = false;
    }

    if (typeof this.#loader === 'undefined')
      this.#loader = new Parser(webots.currentView.prefix);
    else
      this.#loader.prefix = webots.currentView.prefix;
    this.#loader.parse(x3dObject, this.renderer, parentNode, callback);

    this.render();
  }

  applyPose(pose) {
    const id = pose.id;
    if (typeof WbWorld.instance === 'undefined')
      return;

    const object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object === 'undefined')
      return;

    this.#applyPoseToObject(pose, object);

    // Update the related USE nodes
    let length = object.useList.length - 1;
    while (length >= 0) {
      const use = WbWorld.instance.nodes.get(object.useList[length]);
      if (typeof use === 'undefined') {
        // remove a USE node from the list if it has been deleted
        const index = object.useList.indexOf(length);
        this.useList.splice(index, 1);
      } else
        this.#applyPoseToObject(pose, use);

      --length;
    }
  }

  #applyPoseToObject(pose, object) {
    for (let key in pose) {
      if (key === 'id')
        continue;

      if (key === 'translation') {
        const translation = convertStringToVec3(pose[key]);

        if (object instanceof WbTransform) {
          if (typeof WbWorld.instance.viewpoint.followedId !== 'undefined' &&
            WbWorld.instance.viewpoint.followedId === object.id)
            WbWorld.instance.viewpoint.setFollowedObjectDeltaPosition(translation, object.translation);

          object.translation = translation;
          if (WbWorld.instance.readyForUpdates)
            object.applyTranslationToWren();
        } else if (object instanceof WbTextureTransform) {
          object.translation = translation;
          if (WbWorld.instance.readyForUpdates) {
            let appearance = WbWorld.instance.nodes.get(object.parent);
            if (typeof appearance !== 'undefined') {
              let shape = WbWorld.instance.nodes.get(appearance.parent);
              if (typeof shape !== 'undefined')
                shape.updateAppearance();
            }
          }
        }
      } else if (key === 'rotation') {
        const quaternion = convertStringToQuaternion(pose[key]);
        if (object instanceof WbTrackWheel)
          object.updateRotation(quaternion);
        else {
          object.rotation = quaternion;
          if (WbWorld.instance.readyForUpdates)
            object.applyRotationToWren();
        }
      } else if (key === 'scale') {
        if (object instanceof WbTransform) {
          object.scale = convertStringToVec3(pose[key]);
          if (WbWorld.instance.readyForUpdates)
            object.applyScaleToWren();
        }
      } else if (key === 'size') {
        if (object instanceof WbBox || object instanceof WbPlane)
          object.size = convertStringToVec3(pose[key]);
      } else if (key === 'radius') {
        if (object instanceof WbCapsule || object instanceof WbSphere || object instanceof WbCylinder ||
          object instanceof WbSpotLight || object instanceof WbPointLight)
          object.radius = parseFloat(pose[key]);
      } else if (key === 'subdivision') {
        if (object instanceof WbSphere || object instanceof WbCapsule || object instanceof WbCone ||
           object instanceof WbCylinder)
          object.subdivision = parseInt(pose[key]);
      } else if (key === 'ico') {
        if (object instanceof WbSphere)
          object.ico = pose[key].toLowerCase() === 'true';
      } else if (key === 'height') {
        if (object instanceof WbCapsule || object instanceof WbCone || object instanceof WbCylinder)
          object.height = parseFloat(pose[key]);
        else if (object instanceof WbElevationGrid)
          // Filter is used to remove Nan elements
          object.height = convertStringToFloatArray(pose[key]).filter(e => e);
      } else if (key === 'bottom') {
        if (object instanceof WbCapsule || object instanceof WbCone || object instanceof WbCylinder)
          object.bottom = pose[key].toLowerCase() === 'true';
      } else if (key === 'top') {
        if (object instanceof WbCapsule || object instanceof WbCylinder)
          object.top = pose[key].toLowerCase() === 'true';
      } else if (key === 'side') {
        if (object instanceof WbCapsule || object instanceof WbCone || object instanceof WbCylinder)
          object.side = pose[key].toLowerCase() === 'true';
      } else if (key === 'bottomRadius') {
        if (object instanceof WbCone)
          object.bottomRadius = parseFloat(pose[key]);
      } else if (key === 'ccw') {
        if (object instanceof WbMesh || object instanceof WbIndexedFaceSet)
          object.ccw = pose[key].toLowerCase() === 'true';
      } else if (key === 'direction') {
        if (object instanceof WbSpotLight || WbDirectionalLight)
          object.direction = convertStringToVec3(pose[key]);
      } else if (key === 'color') {
        if (object instanceof WbLight)
          object.color = convertStringToVec3(pose[key]);
        else if (object instanceof WbColor)
          object.color = convertStringToVec3Array(pose[key]);
      } else if (key === 'name') {
        if (object instanceof WbMesh)
          object.name = pose[key];
      } else if (key === 'materialIndex') {
        if (object instanceof WbMesh)
          object.materialIndex = parseInt(pose[key]);
      } else if (key === 'url') {
        if (object instanceof WbMesh)
          object.url = pose[key];
        else if (object instanceof WbImageTexture)
          object.url = pose[key][0];
      } else if (key === 'point') {
        if (object instanceof WbCoordinate)
          object.point = convertStringToVec3Array(pose[key]);
        if (object instanceof WbTextureCoordinate)
          object.point = convertStringToVec2Array(pose[key]);
      } else if (key === 'vector') {
        if (object instanceof WbNormal)
          object.vector = convertStringToVec3Array(pose[key]);
      } else if (key === 'coordIndex') {
        if (object instanceof WbIndexedLineSet || object instanceof WbIndexedFaceSet)
          object.coordIndex = pose[key];
      } else if (key === 'attenuation') {
        if (object instanceof WbSpotLight || object instanceof WbPointLight)
          object.attenuation = convertStringToVec3(pose[key]);
      } else if (key === 'location') {
        if (object instanceof WbSpotLight || object instanceof WbPointLight)
          object.location = convertStringToVec3(pose[key]);
      } else if (object instanceof WbSpotLight) {
        if (key === 'beamWidth')
          object.beamWidth = parseFloat(pose[key]);
        else if (key === 'cutOffAngle')
          object.cutOffAngle = parseFloat(pose[key]);
      } else if (object instanceof WbLight) {
        if (key === 'on')
          object.on = pose[key].toLowerCase() === 'true';
        else if (key === 'ambientIntensity')
          object.ambientIntensity = parseFloat(pose[key]);
        else if (key === 'intensity')
          object.intensity = parseFloat(pose[key]);
        else if (key === 'castShadows')
          object.castShadows = pose[key].toLowerCase() === 'true';
      } else if (object instanceof WbIndexedFaceSet) {
        if (key === 'normalPerVertex')
          object.normalPerVertex = pose[key].toLowerCase() === 'true';
        else if (key === 'normalIndex')
          object.normalIndex = pose[key];
        else if (key === 'texCoordIndex')
          object.texCoordIndex = pose[key];
        else if (key === 'creaseAngle')
          object.creaseAngle = parseFloat(pose[key]);
      } else if (object instanceof WbElevationGrid) {
        if (key === 'xDimension')
          object.xDimension = pose[key];
        else if (key === 'xSpacing')
          object.xSpacing = pose[key];
        else if (key === 'yDimension')
          object.yDimension = pose[key];
        else if (key === 'ySpacing')
          object.ySpacing = pose[key];
        else if (key === 'thickness')
          object.thickness = pose[key];
      } else if (object instanceof WbPbrAppearance || object instanceof WbMaterial) {
        if (key === 'baseColor')
          object.baseColor = convertStringToVec3(pose[key]);
        else if (key === 'diffuseColor')
          object.diffuseColor = convertStringToVec3(pose[key]);
        else if (key === 'emissiveColor')
          object.emissiveColor = convertStringToVec3(pose[key]);
        else if (key === 'roughness')
          object.roughness = parseFloat(pose[key]);
        else if (key === 'metalness')
          object.metalness = parseFloat(pose[key]);
        else if (key === 'IBLStrength')
          object.IBLStrength = parseFloat(pose[key]);
        else if (key === 'normalMapFactor')
          object.normalMapFactor = parseFloat(pose[key]);
        else if (key === 'occlusionMapStrength')
          object.occlusionMapStrength = parseFloat(pose[key]);
        else if (key === 'emissiveIntensity')
          object.emissiveIntensity = parseFloat(pose[key]);
        else if (key === 'transparency')
          object.transparency = parseFloat(pose[key]);
        else if (key === 'ambientIntensity')
          object.ambientIntensity = parseFloat(pose[key]);
        else if (key === 'shininess')
          object.shininess = parseFloat(pose[key]);
        else if (key === 'specularColor')
          object.specularColor = convertStringToVec3(pose[key]);
      } else if (object instanceof WbImageTexture) {
        if (key === 'repeatS')
          object.repeatS = pose[key].toLowerCase() === 'true';
        else if (key === 'repeatT')
          object.repeatT = pose[key].toLowerCase() === 'true';
        else if (key === 'filtering')
          object.filtering = parseInt(pose[key]);
      }
    }

    if (typeof object.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(object.parent);
      if (typeof parent !== 'undefined' && parent instanceof WbGroup && parent.isPropeller &&
        parent.currentHelix !== object.id && WbWorld.instance.readyForUpdates)
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

        if (frame.hasOwnProperty('poses')) {
          for (let i = 0; i < frame.poses.length; i++)
            this.applyPose(frame.poses[i]);
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
        if (typeof WbWorld.instance !== 'undefined' && typeof WbWorld.instance.viewpoint !== 'undefined')
          WbWorld.instance.viewpoint.updateFollowUp(view.time);
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
      this.loadObject(data, parentId);
    } else if (data.startsWith('delete:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      this.#deleteObject(data);
    } else if (data.startsWith('model:')) {
      view.progress.setProgressBar('block', 'same', 60 + 0.1 * 17, 'Loading 3D scene...');
      this.destroyWorld();
      view.removeLabels();
      data = data.substring(data.indexOf(':') + 1).trim();
      if (!data) // received an empty model case: just destroy the view
        return true;
      view.stream.socket.send('pause');
      view.progress.setProgressBar('block', 'same', 60 + 0.1 * 23, 'Loading object...');
      this.loadObject(data, 0, view.onready);
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
  const colorArray = convertStringToFloatArray(string);
  if (typeof colorArray !== 'undefined') {
    for (let i = 0; i < colorArray.length; i += 3)
      vecArray.push(new WbVector3(colorArray[i], colorArray[i + 1], colorArray[i + 2]));
  }

  return vecArray;
}

function convertStringToVec2Array(string) {
  const vecArray = [];
  string = string.trim();
  string = string.slice(1, -1).trim();
  const colorArray = convertStringToFloatArray(string);
  if (typeof colorArray !== 'undefined') {
    for (let i = 0; i < colorArray.length; i += 2)
      vecArray.push(new WbVector2(colorArray[i], colorArray[i + 1]));
  }

  return vecArray;
}
