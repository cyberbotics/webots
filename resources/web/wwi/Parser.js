import {M_PI_4} from './nodes/utils/constants.js';
import WbAbstractAppearance from './nodes/WbAbstractAppearance.js';
import WbAppearance from './nodes/WbAppearance.js';
import WbBackground from './nodes/WbBackground.js';
import WbBillboard from './nodes/WbBillboard.js';
import WbBox from './nodes/WbBox.js';
import WbCapsule from './nodes/WbCapsule.js';
import WbCone from './nodes/WbCone.js';
import WbCylinder from './nodes/WbCylinder.js';
import WbDirectionalLight from './nodes/WbDirectionalLight.js';
import WbElevationGrid from './nodes/WbElevationGrid.js';
import WbFog from './nodes/WbFog.js';
import WbGeometry from './nodes/WbGeometry.js';
import WbGroup from './nodes/WbGroup.js';
import WbImage from './nodes/WbImage.js';
import WbImageTexture from './nodes/WbImageTexture.js';
import WbIndexedFaceSet from './nodes/WbIndexedFaceSet.js';
import WbIndexedLineSet from './nodes/WbIndexedLineSet.js';
import WbLight from './nodes/WbLight.js';
import WbMaterial from './nodes/WbMaterial.js';
import WbPBRAppearance from './nodes/WbPBRAppearance.js';
import WbPlane from './nodes/WbPlane.js';
import WbPointLight from './nodes/WbPointLight.js';
import WbPointSet from './nodes/WbPointSet.js';
import WbScene from './nodes/WbScene.js';
import WbShape from './nodes/WbShape.js';
import WbSphere from './nodes/WbSphere.js';
import WbSpotLight from './nodes/WbSpotLight.js';
import WbTextureTransform from './nodes/WbTextureTransform.js';
import WbTransform from './nodes/WbTransform.js';
import WbVector2 from './nodes/utils/WbVector2.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbVector4 from './nodes/utils/WbVector4.js';
import WbViewpoint from './nodes/WbViewpoint.js';
import WbWorld from './nodes/WbWorld.js';
import WbWrenPostProcessingEffects from './wren/WbWrenPostProcessingEffects.js';

import {getAnId, arrayXPointer} from './nodes/utils/utils.js';

import DefaultUrl from './DefaultUrl.js';
import loadHdr from './hdr_loader.js';
import {webots} from './webots.js';

/*
  This module takes an x3d world, parse it and populate the scene.
*/
export default class Parser {
  constructor(prefix = '') {
    this._prefix = prefix;
    this._downloadingImage = new Set();
    this._promises = [];
    WbWorld.init();
  }

  parse(text, renderer, parent, callback) {
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
        else
          this._parseChildren(node, parent);
      } else
        this._parseNode(scene);
    }

    if (document.getElementById('webotsProgressMessage'))
      document.getElementById('webotsProgressMessage').innerHTML = 'Finalizing...';

    return Promise.all(this._promises).then(() => {
      this._promises = [];
      this._downloadingImage.clear();
      if (typeof this.smaaAreaTexture !== 'undefined' && typeof this.smaaSearchTexture !== 'undefined' && typeof this.gtaoNoiseTexture !== 'undefined') {
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
      WbWorld.instance.sceneTree.forEach(node => {
        node.finalize();
      });

      WbWorld.instance.readyForUpdates = true;

      webots.currentView.x3dScene.resize();
      renderer.render();
      if (document.getElementById('webotsProgress'))
        document.getElementById('webotsProgress').style.display = 'none';
      if (webots.currentView.toolBar) {
        webots.currentView.toolBar.enableToolBarButtons(true);
        if (webots.currentView.runOnLoad === 'real-time')
          webots.currentView.toolBar.realTime(true);
        else if (webots.currentView.runOnLoad === 'run' || webots.currentView.runOnLoad === 'fast')
          webots.currentView.toolBar.run(true);
      }

      if (typeof callback === 'function')
        callback();

      console.timeEnd('Loaded in: ');
    });
  }

  _parseNode(node, parentNode, isBoundingObject) {
    if (typeof WbWorld.instance === 'undefined')
      WbWorld.init();

    let result;
    if (node.tagName === 'Scene') {
      this._parseScene(node);
      this._parseChildren(node, parentNode);
    } else if (node.tagName === 'WorldInfo')
      this._parseWorldInfo(node);
    else if (node.tagName === 'Viewpoint')
      WbWorld.instance.viewpoint = this._parseViewpoint(node);
    else if (node.tagName === 'Background')
      result = this._parseBackground(node);
    else if (node.tagName === 'Transform')
      result = this._parseTransform(node, parentNode, isBoundingObject);
    else if (node.tagName === 'Billboard')
      result = this._parseBillboard(node, parentNode);
    else if (node.tagName === 'Group')
      result = this._parseGroup(node, parentNode, isBoundingObject);
    else if (node.tagName === 'Shape')
      result = this._parseShape(node, parentNode, isBoundingObject);
    else if (node.tagName === 'Switch')
      result = this._parseSwitch(node, parentNode);
    else if (node.tagName === 'DirectionalLight')
      result = this._parseDirectionalLight(node, parentNode);
    else if (node.tagName === 'PointLight')
      result = this._parsePointLight(node, parentNode);
    else if (node.tagName === 'SpotLight')
      result = this._parseSpotLight(node, parentNode);
    else if (node.tagName === 'Fog') {
      if (!WbWorld.instance.hasFog)
        result = this._parseFog(node);
      else
        console.error('This world already has a fog.');
    } else {
      // Either it is a node added after the whole scene, or it is an unknown node
      let id;
      if (typeof parentNode !== 'undefined')
        id = parentNode.id;
      result = this._parseGeometry(node, id);

      // We are forced to check if the result correspond to the class we expect because of the case of a USE
      if (typeof result !== 'undefined' && result instanceof WbGeometry) {
        if (typeof parentNode !== 'undefined' && parentNode instanceof WbShape) {
          if (typeof parentNode.geometry !== 'undefined')
            parentNode.geometry.delete();
          parentNode.geometry = result;
        }
      } else if (node.tagName === 'PBRAppearance') {
        if (typeof parentNode !== 'undefined' && parentNode instanceof WbShape) {
          if (typeof parentNode.appearance !== 'undefined')
            parentNode.appearance.delete();
          result = this._parsePBRAppearance(node, id);
          parentNode.appearance = result;
        }
      } else if (node.tagName === 'Appearance') {
        if (typeof parentNode !== 'undefined' && parentNode instanceof WbShape) {
          if (typeof parentNode.appearance !== 'undefined')
            parentNode.appearance.delete();
          result = this._parseAppearance(node, id);
          parentNode.appearance = result;
        }
      } else if (node.tagName === 'Material') {
        result = this._parseMaterial(node, id);
        if (typeof result !== 'undefined') {
          if (typeof parentNode !== 'undefined' && parentNode instanceof WbAppearance) {
            if (typeof parentNode.material !== 'undefined')
              parentNode.material.delete();
            parentNode.material = result;
          }
        }
      } else if (node.tagName === 'ImageTexture') {
        result = this._parseImageTexture(node, id);
        if (typeof result !== 'undefined') {
          if (typeof parentNode !== 'undefined' && parentNode instanceof WbAppearance) {
            if (typeof parentNode.material !== 'undefined')
              parentNode.texture.delete();
            parentNode.texture = result;
          }
        }
      } else if (node.tagName === 'TextureTransform') {
        result = this._parseTextureTransform(node, id);
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

    return result;
  }

  _parseChildren(node, parentNode, isBoundingObject) {
    for (let i = 0; i < node.childNodes.length; i++) {
      const child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined')
        this._parseNode(child, parentNode, isBoundingObject);
    }
    return 1;
  }

  _parseScene(node) {
    const prefix = DefaultUrl.wrenImagesUrl();
    this._promises.push(loadTextureData(prefix, 'smaa_area_texture.png').then(image => {
      this.smaaAreaTexture = image;
      this.smaaAreaTexture.isTranslucent = false;
    }));
    this._promises.push(loadTextureData(prefix, 'smaa_search_texture.png').then(image => {
      this.smaaSearchTexture = image;
      this.smaaSearchTexture.isTranslucent = false;
    }));
    this._promises.push(loadTextureData(prefix, 'gtao_noise_texture.png').then(image => {
      this.gtaoNoiseTexture = image;
      this.gtaoNoiseTexture.isTranslucent = true;
    }));
    WbWorld.instance.scene = new WbScene();
  }

  _parseWorldInfo(node) {
    WbWorld.instance.coordinateSystem = getNodeAttribute(node, 'coordinateSystem', 'ENU');
    WbWorld.computeUpVector();
  }

  _parseId(node) {
    if (typeof node === 'undefined')
      return;

    let id = getNodeAttribute(node, 'id');
    if (typeof id === 'undefined')
      id = getAnId();

    return id;
  }

  _parseViewpoint(node) {
    const id = this._parseId(node);
    const fieldOfView = parseFloat(getNodeAttribute(node, 'fieldOfView', M_PI_4));
    const orientation = convertStringToQuaternion(getNodeAttribute(node, 'orientation', '0 1 0 0'));
    const position = convertStringToVec3(getNodeAttribute(node, 'position', '0 0 10'));
    const exposure = parseFloat(getNodeAttribute(node, 'exposure', '1.0'));
    const bloomThreshold = parseFloat(getNodeAttribute(node, 'bloomThreshold', 21));
    const far = parseFloat(getNodeAttribute(node, 'zFar', '2000'));
    const near = parseFloat(getNodeAttribute(node, 'zNear', '0.1'));
    const followSmoothness = parseFloat(getNodeAttribute(node, 'followSmoothness'));
    const followedId = getNodeAttribute(node, 'followedId');
    const ambientOcclusionRadius = parseFloat(getNodeAttribute(node, 'ambientOcclusionRadius', 2));

    return new WbViewpoint(id, fieldOfView, orientation, position, exposure, bloomThreshold, near, far, followSmoothness, followedId, ambientOcclusionRadius);
  }

  _parseBackground(node) {
    const id = this._parseId(node);
    const skyColor = convertStringToVec3(getNodeAttribute(node, 'skyColor', '0 0 0'));
    const luminosity = parseFloat(getNodeAttribute(node, 'luminosity', '1'));

    let backUrl = getNodeAttribute(node, 'backUrl');
    let bottomUrl = getNodeAttribute(node, 'bottomUrl');
    let frontUrl = getNodeAttribute(node, 'frontUrl');
    let leftUrl = getNodeAttribute(node, 'leftUrl');
    let rightUrl = getNodeAttribute(node, 'rightUrl');
    let topUrl = getNodeAttribute(node, 'topUrl');

    this.cubeImages = [];
    if (typeof backUrl !== 'undefined' && typeof bottomUrl !== 'undefined' && typeof frontUrl !== 'undefined' && typeof leftUrl !== 'undefined' && typeof rightUrl !== 'undefined' && typeof topUrl !== 'undefined') {
      backUrl = backUrl.slice(1, backUrl.length - 1);
      bottomUrl = bottomUrl.slice(1, bottomUrl.length - 1);
      frontUrl = frontUrl.slice(1, frontUrl.length - 1);
      leftUrl = leftUrl.slice(1, leftUrl.length - 1);
      rightUrl = rightUrl.slice(1, rightUrl.length - 1);
      topUrl = topUrl.slice(1, topUrl.length - 1);

      if (WbWorld.instance.coordinateSystem === 'ENU') {
        this._promises.push(loadTextureData(this._prefix, backUrl, false, 90).then(image => { this.cubeImages[0] = image; }));
        this._promises.push(loadTextureData(this._prefix, bottomUrl, false, -90).then(image => { this.cubeImages[4] = image; }));
        this._promises.push(loadTextureData(this._prefix, frontUrl, false, -90).then(image => { this.cubeImages[1] = image; }));
        this._promises.push(loadTextureData(this._prefix, leftUrl, false, 180).then(image => { this.cubeImages[3] = image; }));
        this._promises.push(loadTextureData(this._prefix, rightUrl).then(image => { this.cubeImages[2] = image; }));
        this._promises.push(loadTextureData(this._prefix, topUrl, false, -90).then(image => { this.cubeImages[5] = image; }));
      } else {
        this._promises.push(loadTextureData(this._prefix, topUrl).then(image => { this.cubeImages[2] = image; }));
        this._promises.push(loadTextureData(this._prefix, backUrl).then(image => { this.cubeImages[5] = image; }));
        this._promises.push(loadTextureData(this._prefix, bottomUrl).then(image => { this.cubeImages[3] = image; }));
        this._promises.push(loadTextureData(this._prefix, frontUrl).then(image => { this.cubeImages[4] = image; }));
        this._promises.push(loadTextureData(this._prefix, leftUrl).then(image => { this.cubeImages[1] = image; }));
        this._promises.push(loadTextureData(this._prefix, rightUrl).then(image => { this.cubeImages[0] = image; }));
      }
    }

    let backIrradianceUrl = getNodeAttribute(node, 'backIrradianceUrl');
    let bottomIrradianceUrl = getNodeAttribute(node, 'bottomIrradianceUrl');
    let frontIrradianceUrl = getNodeAttribute(node, 'frontIrradianceUrl');
    let leftIrradianceUrl = getNodeAttribute(node, 'leftIrradianceUrl');
    let rightIrradianceUrl = getNodeAttribute(node, 'rightIrradianceUrl');
    let topIrradianceUrl = getNodeAttribute(node, 'topIrradianceUrl');

    this.irradianceCubeURL = [];
    if (typeof backIrradianceUrl !== 'undefined' && typeof bottomIrradianceUrl !== 'undefined' && typeof frontIrradianceUrl !== 'undefined' && typeof leftIrradianceUrl !== 'undefined' && typeof rightIrradianceUrl !== 'undefined' && typeof topIrradianceUrl !== 'undefined') {
      backIrradianceUrl = backIrradianceUrl.slice(1, backIrradianceUrl.length - 1);
      bottomIrradianceUrl = bottomIrradianceUrl.slice(1, bottomIrradianceUrl.length - 1);
      frontIrradianceUrl = frontIrradianceUrl.slice(1, frontIrradianceUrl.length - 1);
      leftIrradianceUrl = leftIrradianceUrl.slice(1, leftIrradianceUrl.length - 1);
      rightIrradianceUrl = rightIrradianceUrl.slice(1, rightIrradianceUrl.length - 1);
      topIrradianceUrl = topIrradianceUrl.slice(1, topIrradianceUrl.length - 1);

      if (WbWorld.instance.coordinateSystem === 'ENU') {
        this._promises.push(loadTextureData(this._prefix, backIrradianceUrl, true, 90).then(image => { this.irradianceCubeURL[0] = image; }));
        this._promises.push(loadTextureData(this._prefix, bottomIrradianceUrl, true, -90).then(image => { this.irradianceCubeURL[4] = image; }));
        this._promises.push(loadTextureData(this._prefix, frontIrradianceUrl, true, -90).then(image => { this.irradianceCubeURL[1] = image; }));
        this._promises.push(loadTextureData(this._prefix, leftIrradianceUrl, true, 180).then(image => { this.irradianceCubeURL[3] = image; }));
        this._promises.push(loadTextureData(this._prefix, rightIrradianceUrl, true).then(image => { this.irradianceCubeURL[2] = image; }));
        this._promises.push(loadTextureData(this._prefix, topIrradianceUrl, true, -90).then(image => { this.irradianceCubeURL[5] = image; }));
      } else {
        this._promises.push(loadTextureData(this._prefix, topIrradianceUrl, true).then(image => { this.irradianceCubeURL[2] = image; }));
        this._promises.push(loadTextureData(this._prefix, backIrradianceUrl, true).then(image => { this.irradianceCubeURL[5] = image; }));
        this._promises.push(loadTextureData(this._prefix, bottomIrradianceUrl, true).then(image => { this.irradianceCubeURL[3] = image; }));
        this._promises.push(loadTextureData(this._prefix, frontIrradianceUrl, true).then(image => { this.irradianceCubeURL[4] = image; }));
        this._promises.push(loadTextureData(this._prefix, leftIrradianceUrl, true).then(image => { this.irradianceCubeURL[1] = image; }));
        this._promises.push(loadTextureData(this._prefix, rightIrradianceUrl, true).then(image => { this.irradianceCubeURL[0] = image; }));
      }
    }

    const background = new WbBackground(id, skyColor, luminosity);
    WbBackground.instance = background;

    WbWorld.instance.nodes.set(background.id, background);

    return background;
  }

  _checkUse(node, parentNode) {
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
    const id = this._parseId(node);

    const useNode = result.clone(id);
    if (typeof parentNode !== 'undefined') {
      useNode.parent = parentNode.id;
      if (result instanceof WbShape || result instanceof WbGroup || result instanceof WbLight)
        parentNode.children.push(useNode);
    }

    WbWorld.instance.nodes.set(id, useNode);
    return useNode;
  }

  _parseTransform(node, parentNode, isBoundingObject) {
    const use = this._checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);

    const isSolid = getNodeAttribute(node, 'solid', 'false').toLowerCase() === 'true';
    const translation = convertStringToVec3(getNodeAttribute(node, 'translation', '0 0 0'));
    const scale = convertStringToVec3(getNodeAttribute(node, 'scale', '1 1 1'));
    const rotation = convertStringToQuaternion(getNodeAttribute(node, 'rotation', '0 1 0 0'));

    const transform = new WbTransform(id, isSolid, translation, scale, rotation);

    WbWorld.instance.nodes.set(transform.id, transform);

    this._parseChildren(node, transform, isBoundingObject);

    if (typeof parentNode !== 'undefined') {
      transform.parent = parentNode.id;
      parentNode.children.push(transform);
    }

    return transform;
  }

  _parseGroup(node, parentNode, isBoundingObject) {
    const use = this._checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);

    const isPropeller = getNodeAttribute(node, 'isPropeller', 'false').toLowerCase() === 'true';

    const group = new WbGroup(id, isPropeller);

    WbWorld.instance.nodes.set(group.id, group);
    this._parseChildren(node, group, isBoundingObject);

    if (typeof parentNode !== 'undefined') {
      group.parent = parentNode.id;
      parentNode.children.push(group);
    }

    return group;
  }

  _parseShape(node, parentNode, isBoundingObject) {
    const use = this._checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);

    const castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';
    const isPickable = getNodeAttribute(node, 'isPickable', 'true').toLowerCase() === 'true';
    let geometry;
    let appearance;

    for (let i = 0; i < node.childNodes.length; i++) {
      const child = node.childNodes[i];
      if (typeof child.tagName === 'undefined')
        continue;

      if (typeof appearance === 'undefined') {
        if (child.tagName === 'Appearance') {
          if (isBoundingObject)
            continue;
          // If a sibling PBRAppearance is detected, prefer it.
          let pbrAppearanceChild = false;
          for (let j = 0; j < node.childNodes.length; j++) {
            const child0 = node.childNodes[j];
            if (child0.tagName === 'PBRAppearance') {
              pbrAppearanceChild = true;
              break;
            }
          }
          if (pbrAppearanceChild)
            continue;
          appearance = this._parseAppearance(child);
        } else if (child.tagName === 'PBRAppearance')
          appearance = this._parsePBRAppearance(child);
        if (typeof appearance !== 'undefined')
          continue;
      }

      if (typeof geometry === 'undefined') {
        geometry = this._parseGeometry(child, id);
        if (typeof geometry !== 'undefined')
          continue;
      }

      console.log('X3dLoader: Unknown node: ' + child.tagName);
    }

    if (isBoundingObject)
      appearance = undefined;

    const shape = new WbShape(id, castShadows, isPickable, geometry, appearance);

    if (typeof parentNode !== 'undefined') {
      parentNode.children.push(shape);
      shape.parent = parentNode.id;
    }

    if (typeof appearance !== 'undefined')
      appearance.parent = shape.id;

    WbWorld.instance.nodes.set(shape.id, shape);

    return shape;
  }

  _parseBillboard(node, parentNode) {
    const id = this._parseId(node);

    const billboard = new WbBillboard(id);

    WbWorld.instance.nodes.set(billboard.id, billboard);
    this._parseChildren(node, billboard);

    return billboard;
  }

  _parseDirectionalLight(node, parentNode) {
    const use = this._checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);
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

  _parsePointLight(node, parentNode) {
    const use = this._checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);
    const on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    const attenuation = convertStringToVec3(getNodeAttribute(node, 'attenuation', '1 0 0'));
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    const location = convertStringToVec3(getNodeAttribute(node, 'location', '0 0 0'));
    const radius = parseFloat(getNodeAttribute(node, 'radius', '100'));
    const ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    const castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    const pointLight = new WbPointLight(id, on, attenuation, color, intensity, location, radius, ambientIntensity, castShadows, parentNode);

    if (typeof parentNode !== 'undefined' && typeof pointLight !== 'undefined')
      parentNode.children.push(pointLight);

    WbWorld.instance.nodes.set(pointLight.id, pointLight);

    return pointLight;
  }

  _parseSpotLight(node, parentNode) {
    const use = this._checkUse(node, parentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);
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

    const spotLight = new WbSpotLight(id, on, attenuation, beamWidth, color, cutOffAngle, direction, intensity, location, radius, ambientIntensity, castShadows, parentNode);

    if (typeof parentNode !== 'undefined' && typeof spotLight !== 'undefined')
      parentNode.children.push(spotLight);

    WbWorld.instance.nodes.set(spotLight.id, spotLight);

    return spotLight;
  }

  _parseFog(node) {
    const id = this._parseId(node);
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const visibilityRange = parseFloat(getNodeAttribute(node, 'visibilityRange', '0'));
    const fogType = getNodeAttribute(node, 'fogType', 'LINEAR');

    const fog = new WbFog(id, color, visibilityRange, fogType);

    WbWorld.instance.nodes.set(fog.id, fog);

    if (typeof fog !== 'undefined')
      WbWorld.instance.hasFog = true;

    return fog;
  }

  _parseGeometry(node, parentId) {
    const use = this._checkUse(node);
    if (typeof use !== 'undefined') {
      use.parent = parentId;
      return use;
    }

    const id = this._parseId(node);

    let geometry;
    if (node.tagName === 'Box')
      geometry = this._parseBox(node, id);
    else if (node.tagName === 'Sphere')
      geometry = this._parseSphere(node, id);
    else if (node.tagName === 'Cone')
      geometry = this._parseCone(node, id);
    else if (node.tagName === 'Plane')
      geometry = this._parsePlane(node, id);
    else if (node.tagName === 'Cylinder')
      geometry = this._parseCylinder(node, id);
    else if (node.tagName === 'Capsule')
      geometry = this._parseCapsule(node, id);
    else if (node.tagName === 'IndexedFaceSet')
      geometry = this._parseIndexedFaceSet(node, id);
    else if (node.tagName === 'IndexedLineSet')
      geometry = this._parseIndexedLineSet(node, id);
    else if (node.tagName === 'ElevationGrid')
      geometry = this._parseElevationGrid(node, id);
    else if (node.tagName === 'PointSet')
      geometry = this._parsePointSet(node, id);
    else
      console.log('Not a recognized geometry: ' + node.tagName);

    if (typeof parentId !== 'undefined' && typeof geometry !== 'undefined')
      geometry.parent = parentId;

    return geometry;
  }

  _parseBox(node, id) {
    const size = convertStringToVec3(getNodeAttribute(node, 'size', '2 2 2'));

    const box = new WbBox(id, size);
    WbWorld.instance.nodes.set(box.id, box);
    return box;
  }

  _parseSphere(node, id) {
    const radius = parseFloat(getNodeAttribute(node, 'radius', '1'));
    const ico = getNodeAttribute(node, 'ico', 'false').toLowerCase() === 'true';
    const subdivision = parseInt(getNodeAttribute(node, 'subdivision', '1,1'));

    const sphere = new WbSphere(id, radius, ico, subdivision);

    WbWorld.instance.nodes.set(sphere.id, sphere);

    return sphere;
  }

  _parseCone(node, id) {
    const bottomRadius = getNodeAttribute(node, 'bottomRadius', '1');
    const height = getNodeAttribute(node, 'height', '2');
    const subdivision = getNodeAttribute(node, 'subdivision', '32');
    const side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    const bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';

    const cone = new WbCone(id, bottomRadius, height, subdivision, side, bottom);

    WbWorld.instance.nodes.set(cone.id, cone);

    return cone;
  }

  _parseCylinder(node, id) {
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

  _parsePlane(node, id) {
    const size = convertStringToVec2(getNodeAttribute(node, 'size', '1,1'));

    const plane = new WbPlane(id, size);

    WbWorld.instance.nodes.set(plane.id, plane);

    return plane;
  }

  _parseCapsule(node, id) {
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

  _parseIndexedFaceSet(node, id) {
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

    const ccw = parseFloat(getNodeAttribute(node, 'ccw', '1'));

    const ifs = new WbIndexedFaceSet(id, coordIndex, normalIndex, texCoordIndex, coordArray, texCoordArray, normalArray, ccw);
    WbWorld.instance.nodes.set(ifs.id, ifs);

    return ifs;
  }

  _parseIndexedLineSet(node, id) {
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

  _parseElevationGrid(node, id) {
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

  _parsePointSet(node, id) {
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

  _parseSwitch(node, parent) {
    if (typeof parent === 'undefined')
      return;

    const child = node.childNodes[0];

    let boundingObject;
    if (child.tagName === 'Shape')
      boundingObject = this._parseShape(child, undefined, true);
    else if (child.tagName === 'Transform')
      boundingObject = this._parseTransform(child, undefined, true);
    else if (child.tagName === 'Group')
      boundingObject = this._parseGroup(child, undefined, true);
    else
      console.error('Unknown boundingObject: ' + child.tagName);

    if (typeof boundingObject === 'undefined')
      return;

    boundingObject.parent = parent.id;
    parent.boundingObject = boundingObject;

    return boundingObject;
  }

  _parseAppearance(node, parentId) {
    const use = this._checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);

    // Get the Material tag.
    const materialNode = node.getElementsByTagName('Material')[0];
    let material;
    if (typeof materialNode !== 'undefined')
      material = this._parseMaterial(materialNode);

    // Check to see if there is a texture.
    const imageTexture = node.getElementsByTagName('ImageTexture')[0];
    let texture;
    if (typeof imageTexture !== 'undefined')
      texture = this._parseImageTexture(imageTexture);

    // Check to see if there is a textureTransform.
    const textureTransform = node.getElementsByTagName('TextureTransform')[0];
    let transform;
    if (typeof textureTransform !== 'undefined')
      transform = this._parseTextureTransform(textureTransform);

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

  _parseMaterial(node, parentId) {
    const use = this._checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);

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

  _parseImageTexture(node, parentId) {
    const use = this._checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);
    let url = getNodeAttribute(node, 'url', '');
    url = url.slice(1, url.length - 1);
    const isTransparent = getNodeAttribute(node, 'isTransparent', 'false').toLowerCase() === 'true';
    const s = getNodeAttribute(node, 'repeatS', 'true').toLowerCase() === 'true';
    const t = getNodeAttribute(node, 'repeatT', 'true').toLowerCase() === 'true';
    const filtering = parseFloat(getNodeAttribute(node, 'filtering', '4'));

    let imageTexture;
    if (typeof url !== 'undefined' && url !== '') {
      imageTexture = new WbImageTexture(id, url, isTransparent, s, t, filtering);
      if (!this._downloadingImage.has(url)) {
        this._downloadingImage.add(url);
        // Load the texture in WREN
        this._promises.push(loadImageTextureInWren(this._prefix, url, isTransparent));
      }
    }

    if (typeof imageTexture !== 'undefined') {
      if (typeof parentId !== 'undefined')
        imageTexture.parent = parentId;

      WbWorld.instance.nodes.set(imageTexture.id, imageTexture);
    }

    return imageTexture;
  }

  _parsePBRAppearance(node, parentId) {
    const use = this._checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);

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
      transform = this._parseTextureTransform(textureTransform);

    const imageTextures = node.getElementsByTagName('ImageTexture');
    let baseColorMap, roughnessMap, metalnessMap, normalMap, occlusionMap, emissiveColorMap;
    for (let i = 0; i < imageTextures.length; i++) {
      const imageTexture = imageTextures[i];
      const type = getNodeAttribute(imageTexture, 'type', undefined);
      if (type === 'baseColor') {
        baseColorMap = this._parseImageTexture(imageTexture);
        if (typeof baseColorMap !== 'undefined')
          baseColorMap.type = 'baseColorMap';
      } else if (type === 'roughness') {
        roughnessMap = this._parseImageTexture(imageTexture);
        if (typeof roughnessMap !== 'undefined')
          roughnessMap.type = 'roughnessMap';
      } else if (type === 'metalness') {
        metalnessMap = this._parseImageTexture(imageTexture);
        if (typeof metalnessMap !== 'undefined')
          metalnessMap.type = 'metalnessMap';
      } else if (type === 'normal') {
        normalMap = this._parseImageTexture(imageTexture);
        if (typeof normalMap !== 'undefined')
          normalMap.type = 'normalMap';
      } else if (type === 'occlusion') {
        occlusionMap = this._parseImageTexture(imageTexture);
        if (typeof occlusionMap !== 'undefined')
          occlusionMap.type = 'occlusionMap';
      } else if (type === 'emissiveColor') {
        emissiveColorMap = this._parseImageTexture(imageTexture);
        if (typeof emissiveColorMap !== 'undefined')
          emissiveColorMap.type = 'emissiveColorMap';
      }
    }

    const pbrAppearance = new WbPBRAppearance(id, baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness, metalnessMap, IBLStrength,
      normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor, emissiveColorMap, emissiveIntensity, transform);

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

  _parseTextureTransform(node, parentId) {
    const use = this._checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = this._parseId(node);
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

function loadTextureData(prefix, url, isHdr, rotation) {
  const canvas2 = document.createElement('canvas');
  const context = canvas2.getContext('2d');

  if (url.startsWith('webots://')) {
    if (typeof webots.currentView.repository === 'undefined')
      webots.currentView.repository = 'cyberbotics';
    if (typeof webots.currentView.branch === 'undefined' || webots.currentView.branch === '')
      webots.currentView.branch = 'released';
    url = url.replace('webots://', 'https://raw.githubusercontent.com/' + webots.currentView.repository + '/webots/' + webots.currentView.branch + '/');
  }
  if (typeof prefix !== 'undefined' && !url.startsWith('http'))
    url = prefix + url;
  if (isHdr) {
    return _loadHDRImage(url).then(img => {
      const image = new WbImage();
      image.bits = img.data;
      image.width = img.width;
      image.height = img.height;
      image.url = url;
      if (typeof rotation !== 'undefined')
        image.bits = rotateHDR(image, rotation);
      return image;
    });
  } else {
    return _loadImage(url).then(img => {
      const image = new WbImage();

      canvas2.width = img.width;
      canvas2.height = img.height;
      if (typeof rotation !== 'undefined') {
        context.save();
        context.translate(canvas2.width / 2, canvas2.height / 2);
        context.rotate(rotation * Math.PI / 180);
        context.drawImage(img, -canvas2.width / 2, -canvas2.height / 2);
        context.restore();
      } else
        context.drawImage(img, 0, 0);

      const dataBGRA = context.getImageData(0, 0, img.width, img.height).data;
      let data = new Uint8ClampedArray(dataBGRA.length);
      data = dataBGRA;

      image.bits = data;
      image.width = img.width;
      image.height = img.height;
      image.url = url;
      return image;
    });
  }
}

function _loadImage(src) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => {
      resolve(img);
    };
    img.onerror = () => console.log('Error in loading: ' + src);
    img.setAttribute('crossOrigin', '');
    img.src = src;
  });
}

function _loadHDRImage(src) {
  return new Promise((resolve, reject) => {
    loadHdr(src, function(img) { resolve(img); });
  });
}

function loadImageTextureInWren(prefix, url, isTransparent) {
  return loadTextureData(prefix, url).then((image) => {
    let texture = _wr_texture_2d_new();
    _wr_texture_set_size(texture, image.width, image.height);
    _wr_texture_set_translucent(texture, isTransparent);
    const bitsPointer = arrayXPointer(image.bits);
    _wr_texture_2d_set_data(texture, bitsPointer);
    Module.ccall('wr_texture_2d_set_file_path', null, ['number', 'string'], [texture, url]);
    _wr_texture_setup(texture);
    _free(bitsPointer);
  });
}

function getNodeAttribute(node, attributeName, defaultValue) {
  console.assert(node && node.attributes);
  if (attributeName in node.attributes)
    return node.attributes.getNamedItem(attributeName).value;
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

function rotateHDR(image, rotate) {
  let rotatedbits = [];
  if (rotate === 90) {
    for (let x = 0; x < image.width; x++) {
      for (let y = 0; y < image.height; y++) {
        const u = y * image.width * 3 + x * 3;
        const v = (image.width - 1 - x) * image.height * 3 + y * 3;
        for (let c = 0; c < 3; c++)
          rotatedbits[u + c] = image.bits[v + c];
      }
    }
    const swap = image.width;
    image.width = image.height;
    image.height = swap;
  } else if (rotate === -90) {
    for (let x = 0; x < image.width; x++) {
      for (let y = 0; y < image.height; y++) {
        const u = y * image.width * 3 + x * 3;
        const v = x * image.width * 3 + (image.height - 1 - y) * 3;
        for (let c = 0; c < 3; c++)
          rotatedbits[u + c] = image.bits[v + c];
      }
    }
    const swap = image.width;
    image.width = image.height;
    image.height = swap;
  } else if (rotate === 180) {
    for (let x = 0; x < image.width; x++) {
      for (let y = 0; y < image.height; y++) {
        const u = y * image.width * 3 + x * 3;
        const v = (image.height - 1 - y) * image.width * 3 + (image.width - 1 - x) * 3;
        for (let c = 0; c < 3; c++)
          rotatedbits[u + c] = image.bits[v + c];
      }
    }
  }
  return rotatedbits;
}

export {convertStringToVec3, convertStringToQuaternion, loadImageTextureInWren};
