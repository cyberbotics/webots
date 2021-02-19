import {Parser, convertStringToVec3, convertStringToQuaternion} from './parser.js';
import {webots} from './webots.js';
import {WrenRenderer} from './wrenRenderer.js';

import {getAncestor} from './nodes/wbUtils.js';
import {WbGroup} from './nodes/wbGroup.js';
import {WbPBRAppearance} from './nodes/wbPBRAppearance.js';
import {WbMaterial} from './nodes/wbMaterial.js';
import {WbTransform} from './nodes/wbTransform.js';
import {WbWorld} from './nodes/wbWorld.js';

class X3dScene { // eslint-disable-line no-unused-vars
  constructor(domElement) {
    this.domElement = domElement;
    this.loader = new Parser(this.prefix);
  }

  init(texturePathPrefix = '') {
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
    let currentTime = (new Date()).getTime();
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

    this.renderer.renderMinimal();

    this.nextRenderingTime = (new Date()).getTime() + renderingMinTimeStep;
    clearTimeout(this.renderingTimeout);
    this.renderingTimeout = null;
  }

  resize() {
    let width = this.domElement.clientWidth;
    let height = this.domElement.clientHeight;

    this.renderer.setSize(width, height);

    if (typeof WbWorld.instance.scene !== 'undefined')
      WbWorld.instance.scene.updateFrameBuffer();

    if (typeof WbWorld.instance.viewpoint !== 'undefined')
      WbWorld.instance.viewpoint.updatePostProcessingEffects();

    this.render();
  }

  onSceneUpdate() {
    this.sceneModified = true;
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
  }

  deleteObject(id) {
    console.log(id);
    let object = WbWorld.instance.nodes.get('n' + id);
    if (typeof object === 'undefined')
      return;

    object.delete();

    this.onSceneUpdate();
    console.log(WbWorld.instance);
  }

  loadWorldFile(url, onLoad) {
    let request = new XMLHttpRequest();
    request.open('GET', url, true);
    request.onload = async function() {
      let doc = request.responseXML;

      if (typeof this.loader === 'undefined')
        this.loader = new Parser(this.prefix);

      this.loader.prefix = '';
      console.log(doc.getElementsByTagName('Scene')[0]);
      await this.loader.parseFile(doc);
      onLoad();
    };

    request.send(null);
  }

  loadObject(x3dObject, parentId) {
    let parentNode;
    if (typeof parentId !== 'undefined' && parentId > 0) {
      parentNode = WbWorld.instance.nodes.get('n' + parentId);
      let ancestor = getAncestor(parentNode);
      ancestor.isPreFinalizeCalled = false;
      ancestor.wrenObjectsCreatedCalled = false;
      ancestor.isPostFinalizeCalled = false;
    }

    if (typeof this.loader === 'undefined')
      this.loader = new Parser(this.prefix);

    this.loader.prefix = 'http://localhost:1234/';
    this.loader.parse(x3dObject, this.renderer, parentNode);

    this.onSceneUpdate();
  }

  applyPose(pose) {
    let id = pose.id;
    let object = WbWorld.instance.nodes.get('n' + id);

    if (typeof object === 'undefined')
      return;

    this.applyPoseToObject(pose, object);

    // Update the related USE nodes
    let length = object.useList.length - 1;
    while (length >= 0) {
      let use = WbWorld.instance.nodes.get(object.useList[length]);
      if (typeof use === 'undefined') {
        // remove a USE node from the list if it has been deleted
        let index = object.useList.indexOf(length);
        this.useList.splice(index, 1);
      } else
        this.applyPoseToObject(pose, use);

      --length;
    }
  }

  applyPoseToObject(pose, object) {
    let fields = [];

    for (let key in pose) {
      if (key === 'id')
        continue;
      if (fields.indexOf(key) !== -1)
        continue;

      if (key === 'translation' && object instanceof WbTransform) {
        let translation = convertStringToVec3(pose[key]);
        object.translation = translation;
        object.applyTranslationToWren();
        fields.push(key);
      } else if (key === 'rotation') {
        let quaternion = convertStringToQuaternion(pose[key]);
        object.rotation = quaternion;
        object.applyRotationToWren();
        fields.push(key);
      } else if (object instanceof WbPBRAppearance || object instanceof WbMaterial) {
        if (key === 'baseColor')
          object.baseColor = convertStringToVec3(pose[key]);
        else if (key === 'diffuseColor')
          object.diffuseColor = convertStringToVec3(pose[key]);
        else if (key === 'emissiveColor')
          object.emissiveColor = convertStringToVec3(pose[key]);

        if (object instanceof WbMaterial)
          WbWorld.instance.nodes.get(WbWorld.instance.nodes.get(object.parent).parent).updateAppearance();
        else
          WbWorld.instance.nodes.get(object.parent).updateAppearance();
      }
    }

    if (typeof object.parent !== 'undefined') {
      let parent = WbWorld.instance.nodes.get(object.parent);
      if (typeof parent !== 'undefined' && parent instanceof WbGroup && parent.isPropeller && parent.currentHelix !== object.id)
        parent.switchHelix(object.id);
    }

    if (typeof WbWorld.instance.viewpoint.followedId !== 'undefined' && WbWorld.instance.viewpoint.followedId === object.id)
      WbWorld.instance.viewpoint.updateFollowUp();
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
      view.stream.socket.send('pause');
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
}

export {X3dScene};
