import Parser, {convertStringToVec3, convertStringToQuaternion} from './Parser.js';
import {webots} from './webots.js';
import WrenRenderer from './WrenRenderer.js';

import {getAncestor} from './nodes/utils/utils.js';
import WbGroup from './nodes/WbGroup.js';
import WbLight from './nodes/WbLight.js';
import WbMaterial from './nodes/WbMaterial.js';
import WbPbrAppearance from './nodes/WbPbrAppearance.js';
import WbTextureTransform from './nodes/WbTextureTransform.js';
import WbTrackWheel from './nodes/WbTrackWheel.js';
import WbTransform from './nodes/WbTransform.js';
import WbWorld from './nodes/WbWorld.js';

export default class X3dScene {
  constructor(domElement) {
    this.domElement = domElement;
    this._loader = new Parser(this.prefix);
    // Each time a render is needed, we ensure that there will be 10 additional renderings to avoid gtao artifacts
    this.remainingRenderings = 10;
  }

  init(texturePathPrefix = '') {
    this.prefix = texturePathPrefix;
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
    if (this._nextRenderingTime && this._nextRenderingTime > currentTime) {
      if (!this._renderingTimeout)
        this._renderingTimeout = setTimeout(() => this.render(toRemoveGtaoArtifact), this._nextRenderingTime - currentTime);
      return;
    }

    this.renderer.render();

    this._nextRenderingTime = (new Date()).getTime() + renderingMinTimeStep;
    clearTimeout(this._renderingTimeout);
    this._renderingTimeout = null;

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
      typeof document.getElementsByTagName('webots-view')[0].toolbar !== 'undefined')
      document.getElementsByTagName('webots-view')[0].toolbar.removeRobotWindows();

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
    clearTimeout(this._renderingTimeout);
    this._loader = undefined;
  }

  _deleteObject(id) {
    const object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object === 'undefined')
      return;

    object.delete();

    WbWorld.instance.robots.forEach((robot, i) => {
      if (robot.id === 'n' + id)
        WbWorld.instance.robots.splice(i, 1);
    });

    if (document.getElementById('robot-window-button') !== null)
      document.getElementsByTagName('webots-view')[0].toolbar.loadRobotWindows();

    this.render();
  }

  loadWorldFile(url, onLoad, progress) {
    const prefix = this.prefix;
    const renderer = this.renderer;
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async function() {
      // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) {
        const loader = new Parser(prefix);
        await loader.parse(xmlhttp.responseText, renderer);
        onLoad();
      } else if (xmlhttp.status === 404)
        progress.setProgressBar('block', 'Loading world file...', 5, '(error) File not found: ' + url);
    };
    xmlhttp.onerror = () => {
      progress.setProgressBar('block', 'Loading world file...', 5, 'An unknown error occurred during the loading...');
    };
    xmlhttp.send();
  }

  _loadObject(x3dObject, parentId, callback) {
    let parentNode;
    if (typeof parentId !== 'undefined' && parentId > 0) {
      parentNode = WbWorld.instance.nodes.get('n' + parentId);
      const ancestor = getAncestor(parentNode);
      ancestor.isPreFinalizeCalled = false;
      ancestor.wrenObjectsCreatedCalled = false;
      ancestor.isPostFinalizeCalled = false;
    }

    if (typeof this._loader === 'undefined')
      this._loader = new Parser(this.prefix);

    this._loader.parse(x3dObject, this.renderer, parentNode, callback);

    this.render();
  }

  applyPose(pose) {
    const id = pose.id;
    if (typeof WbWorld.instance === 'undefined')
      return;

    const object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object === 'undefined')
      return;

    this._applyPoseToObject(pose, object);

    // Update the related USE nodes
    let length = object.useList.length - 1;
    while (length >= 0) {
      const use = WbWorld.instance.nodes.get(object.useList[length]);
      if (typeof use === 'undefined') {
        // remove a USE node from the list if it has been deleted
        const index = object.useList.indexOf(length);
        this.useList.splice(index, 1);
      } else
        this._applyPoseToObject(pose, use);

      --length;
    }
  }

  _applyPoseToObject(pose, object) {
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
      } else if (object instanceof WbPbrAppearance || object instanceof WbMaterial) {
        if (key === 'baseColor')
          object.baseColor = convertStringToVec3(pose[key]);
        else if (key === 'diffuseColor')
          object.diffuseColor = convertStringToVec3(pose[key]);
        else if (key === 'emissiveColor')
          object.emissiveColor = convertStringToVec3(pose[key]);

        if (object instanceof WbMaterial) {
          if (WbWorld.instance.readyForUpdates) {
            let appearance = WbWorld.instance.nodes.get(object.parent);
            if (typeof appearance !== 'undefined') {
              let shape = WbWorld.instance.nodes.get(appearance.parent);
              if (typeof shape !== 'undefined')
                shape.updateAppearance();
            }
          }
        } else {
          if (WbWorld.instance.readyForUpdates) {
            let shape = WbWorld.instance.nodes.get(object.parent);
            if (typeof shape !== 'undefined')
              shape.updateAppearance();
          }
        }
      } else if (object instanceof WbLight) {
        if (key === 'color') {
          object.color = convertStringToVec3(pose[key]);
          object.updateColor();
        } else if (key === 'on') {
          object.on = pose[key].toLowerCase() === 'true';
          object.updateOn();
        }
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
      this._loadObject(data, parentId);
    } else if (data.startsWith('delete:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      this._deleteObject(data);
    } else if (data.startsWith('model:')) {
      view.progress.setProgressBar('block', 'same', 60 + 0.1 * 17, 'Loading 3D scene...');
      this.destroyWorld();
      view.removeLabels();
      data = data.substring(data.indexOf(':') + 1).trim();
      if (!data) // received an empty model case: just destroy the view
        return true;
      view.stream.socket.send('pause');
      view.progress.setProgressBar('block', 'same', 60 + 0.1 * 23, 'Loading object...');
      this._loadObject(data, 0, view.onready);
    } else
      return false;
    return true;
  }

  resetViewpoint() {
    WbWorld.instance.viewpoint.resetViewpoint();
    this.render();
  }
}
