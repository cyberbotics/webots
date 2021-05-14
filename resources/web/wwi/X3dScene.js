import Parser, {convertStringToVec3, convertStringToQuaternion} from './Parser.js';
import {webots} from './webots.js';
import WrenRenderer from './WrenRenderer.js';

import {getAncestor} from './nodes/utils/utils.js';
import WbGroup from './nodes/WbGroup.js';
import WbPBRAppearance from './nodes/WbPBRAppearance.js';
import WbMaterial from './nodes/WbMaterial.js';
import WbTransform from './nodes/WbTransform.js';
import WbWorld from './nodes/WbWorld.js';

export default class X3dScene {
  constructor(domElement) {
    this.domElement = domElement;
    this.loader = new Parser(this.prefix);
  }

  init(texturePathPrefix = '') {
    this.prefix = texturePathPrefix;
    this.renderer = new WrenRenderer();

    this.resize();

    this.destroyWorld();
  }

  render() {
    // Set maximum rendering frequency.
    // To avoid slowing down the simulation rendering the scene too often, the last rendering time is checked
    // and the rendering is performed only at a given maximum frequency.
    // To be sure that no rendering request is lost, a timeout is set.
    const renderingMinTimeStep = 40; // Rendering maximum frequency: every 40 ms.
    const currentTime = (new Date()).getTime();
    if (this.nextRenderingTime && this.nextRenderingTime > currentTime) {
      if (!this.renderingTimeout)
        this.renderingTimeout = setTimeout(() => this.render(), this.nextRenderingTime - currentTime);
      return;
    }

    this.renderer.render();

    this.nextRenderingTime = (new Date()).getTime() + renderingMinTimeStep;
    clearTimeout(this.renderingTimeout);
    this.renderingTimeout = null;
  }

  renderMinimal() {
    this.renderer.renderMinimal();
  }

  resize() {
    const width = this.domElement.clientWidth;
    const height = this.domElement.clientHeight;

    this.renderer.setSize(width, height);

    if (typeof WbWorld.instance.scene !== 'undefined')
      WbWorld.instance.scene.updateFrameBuffer();

    if (typeof WbWorld.instance.viewpoint !== 'undefined')
      WbWorld.instance.viewpoint.updatePostProcessingEffects();

    this.render();
  }

  destroyWorld() {
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
    this.loader = undefined;
    webots.currentView.runOnLoad = false;
  }

  deleteObject(id) {
    const object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object === 'undefined')
      return;

    object.delete();

    this.render();
  }

  loadWorldFile(url, onLoad) {
    const prefix = this.prefix;
    const renderer = this.renderer;
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async function() {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) { // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
        const loader = new Parser(prefix);
        await loader.parse(xmlhttp.responseText, renderer);
        onLoad();
      }
    };
    xmlhttp.send();
  }

  loadObject(x3dObject, parentId, callback) {
    let parentNode;
    if (typeof parentId !== 'undefined' && parentId > 0) {
      parentNode = WbWorld.instance.nodes.get('n' + parentId);
      const ancestor = getAncestor(parentNode);
      ancestor.isPreFinalizeCalled = false;
      ancestor.wrenObjectsCreatedCalled = false;
      ancestor.isPostFinalizeCalled = false;
    }

    if (typeof this.loader === 'undefined')
      this.loader = new Parser(this.prefix);

    this.loader.parse(x3dObject, this.renderer, parentNode, callback);

    this.render();
  }

  applyPose(pose, time, appliedFields = [], automaticMove) {
    const id = pose.id;
    if (typeof WbWorld.instance === 'undefined')
      return appliedFields;

    const object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object === 'undefined')
      return;

    let fields = [...appliedFields];

    fields = this.applyPoseToObject(pose, object, time, fields);

    // Update the related USE nodes
    let length = object.useList.length - 1;
    while (length >= 0) {
      const use = WbWorld.instance.nodes.get(object.useList[length]);
      if (typeof use === 'undefined') {
        // remove a USE node from the list if it has been deleted
        const index = object.useList.indexOf(length);
        this.useList.splice(index, 1);
      } else {
        fields = [...appliedFields];
        fields = this.applyPoseToObject(pose, use, time, fields);
      }

      --length;
    }

    return fields;
  }

  applyPoseToObject(pose, object, time, fields, automaticMove) {
    for (let key in pose) {
      if (key === 'id')
        continue;

      if (fields.indexOf(key) !== -1)
        continue;

      let valid = true;
      if (key === 'translation' && object instanceof WbTransform) {
        const translation = convertStringToVec3(pose[key]);

        if (typeof WbWorld.instance.viewpoint.followedId !== 'undefined' && WbWorld.instance.viewpoint.followedId === object.id) {
          WbWorld.instance.viewpoint.setFollowedObjectDeltaPosition(translation, object.translation);
          WbWorld.instance.viewpoint.updateFollowUp(time);
        }

        object.translation = translation;
        if (WbWorld.instance.readyForUpdates)
          object.applyTranslationToWren();
      } else if (key === 'rotation') {
        const quaternion = convertStringToQuaternion(pose[key]);
        object.rotation = quaternion;
        if (WbWorld.instance.readyForUpdates)
          object.applyRotationToWren();
      } else if (object instanceof WbPBRAppearance || object instanceof WbMaterial) {
        if (key === 'baseColor')
          object.baseColor = convertStringToVec3(pose[key]);
        else if (key === 'diffuseColor')
          object.diffuseColor = convertStringToVec3(pose[key]);
        else if (key === 'emissiveColor')
          object.emissiveColor = convertStringToVec3(pose[key]);

        if (object instanceof WbMaterial) {
          if (WbWorld.instance.readyForUpdates)
            WbWorld.instance.nodes.get(WbWorld.instance.nodes.get(object.parent).parent).updateAppearance();
        } else {
          if (WbWorld.instance.readyForUpdates)
            WbWorld.instance.nodes.get(object.parent).updateAppearance();
        }
      } else
        valid = false;

      if (valid)
        fields.push(key);
    }

    if (typeof object.parent !== 'undefined') {
      const parent = WbWorld.instance.nodes.get(object.parent);
      if (typeof parent !== 'undefined' && parent instanceof WbGroup && parent.isPropeller && parent.currentHelix !== object.id && WbWorld.instance.readyForUpdates)
        parent.switchHelix(object.id);
    }

    return fields;
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
        if (document.getElementById('webotsClock'))
          document.getElementById('webotsClock').innerHTML = webots.parseMillisecondsIntoReadableTime(frame.time);

        if (frame.hasOwnProperty('poses')) {
          for (let i = 0; i < frame.poses.length; i++)
            this.applyPose(frame.poses[i], view.time);
        }

        if (frame.hasOwnProperty('labels')) {
          for (let i = 0; i < frame.labels.length; i++)
            this.applyLabel(frame.labels[i], view);
        }

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
      this.deleteObject(data);
    } else if (data.startsWith('model:')) {
      if (view.toolBar)
        view.toolBar.enableToolBarButtons(false);

      if (document.getElementById('webotsProgressMessage'))
        document.getElementById('webotsProgressMessage').innerHTML = 'Loading 3D scene...';
      if (document.getElementById('webotsProgressPercent'))
        document.getElementById('webotsProgressPercent').innerHTML = '';
      if (document.getElementById('webotsProgress'))
        document.getElementById('webotsProgress').style.display = 'block';
      this.destroyWorld();
      view.removeLabels();
      data = data.substring(data.indexOf(':') + 1).trim();
      if (!data) // received an empty model case: just destroy the view
        return true;
      view.stream.socket.send('pause');
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
