// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
import {M_PI_4} from './nodes/wbConstants.js';
import {WbAbstractAppearance} from './nodes/wbAbstractAppearance.js';
import {WbAppearance} from './nodes/wbAppearance.js';
import {WbBackground} from './nodes/wbBackground.js';
import {WbBox} from './nodes/wbBox.js';
import {WbCapsule} from './nodes/wbCapsule.js';
import {WbCone} from './nodes/wbCone.js';
import {WbCylinder} from './nodes/wbCylinder.js';
import {WbDirectionalLight} from './nodes/wbDirectionalLight.js';
import {WbElevationGrid} from './nodes/wbElevationGrid.js';
import {WbFog} from './nodes/wbFog.js';
import {WbGeometry} from './nodes/wbGeometry.js';
import {WbGroup} from './nodes/wbGroup.js';
import {WbImage} from './nodes/wbImage.js';
import {WbImageTexture} from './nodes/wbImageTexture.js';
import {WbIndexedFaceSet} from './nodes/wbIndexedFaceSet.js';
import {WbIndexedLineSet} from './nodes/wbIndexedLineSet.js';
import {WbLight} from './nodes/wbLight.js';
import {WbMaterial} from './nodes/wbMaterial.js';
import {WbPBRAppearance} from './nodes/wbPBRAppearance.js';
import {WbPlane} from './nodes/wbPlane.js';
import {WbPointLight} from './nodes/wbPointLight.js';
import {WbPointSet} from './nodes/wbPointSet.js';
import {WbScene} from './nodes/wbScene.js';
import {WbShape} from './nodes/wbShape.js';
import {WbSphere} from './nodes/wbSphere.js';
import {WbSpotLight} from './nodes/wbSpotLight.js';
import {WbTextureTransform} from './nodes/wbTextureTransform.js';
import {WbTransform} from './nodes/wbTransform.js';
import {WbVector2} from './nodes/utils/wbVector2.js';
import {WbVector3} from './nodes/utils/wbVector3.js';
import {WbVector4} from './nodes/utils/wbVector4.js';
import {WbViewpoint} from './nodes/wbViewpoint.js';
import {WbWorld} from './nodes/wbWorld.js';

import {DefaultUrl} from './default_url.js';
import {RGBELoader} from './hdrLoader.js';

class Parser {
  constructor(prefix = '') {
    this.prefix = prefix;
    WbWorld.init();
  }

  async parse(text, renderer, parent, callback) {
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
        console.log(node);
        if (typeof node === 'undefined')
          console.error('Unknown content, nor Scene, nor Node');
        else
          await this.parseChildren(node, parent);
      } else {
        console.log(scene);
        await this.parseNode(scene);
      }
    }

    console.log(WbWorld.instance);
    $('#webotsProgressMessage').html('Finalizing...');
    // console.timeEnd('startID');
    console.log('FINALIZE');

    if (typeof WbWorld.instance.viewpoint === 'undefined')
      return;
    WbWorld.instance.viewpoint.finalize();

    WbWorld.instance.sceneTree.forEach(node => {
      node.finalize();
    });

    //  console.time('startID');
    renderer.render();
    console.timeEnd('startID');
    console.log('RENDER');
    $('#webotsProgress').hide();
    if (typeof callback === 'function')
      callback();
  }

  async parseNode(node, currentNode, isBoundingObject) {
    if (typeof WbWorld.instance === 'undefined')
      WbWorld.init();

    let result;
    if (node.tagName === 'Scene') {
      WbWorld.instance.scene = await this.parseScene(node);
      await this.parseChildren(node, currentNode);
    } else if (node.tagName === 'WorldInfo')
      this.parseWorldInfo(node);
    else if (node.tagName === 'Viewpoint')
      WbWorld.instance.viewpoint = this.parseViewpoint(node);
    else if (node.tagName === 'Background')
      result = await this.parseBackground(node);
    else if (node.tagName === 'Transform')
      result = await this.parseTransform(node, currentNode, isBoundingObject);
    else if (node.tagName === 'Group')
      result = await this.parseGroup(node, currentNode, isBoundingObject);
    else if (node.tagName === 'Shape')
      result = await this.parseShape(node, currentNode, isBoundingObject);
    else if (node.tagName === 'Switch')
      result = await this.parseSwitch(node, currentNode);
    else if (node.tagName === 'DirectionalLight')
      result = await this.parseDirectionalLight(node, currentNode);
    else if (node.tagName === 'PointLight')
      result = await this.parsePointLight(node, currentNode);
    else if (node.tagName === 'SpotLight')
      result = await this.parseSpotLight(node, currentNode);
    else if (node.tagName === 'Fog') {
      if (!WbWorld.instance.hasFog)
        result = await this.parseFog(node);
      else
        console.error('This world already has a fog.');
    } else {
      // Either it is a node added after the whole scene, or it is an unknown node
      let id;
      if (typeof currentNode !== 'undefined')
        id = currentNode.id;
      result = await this.parseGeometry(node, id);

      // We are forced to check if the result correspond to the class we except because of the case of a USE
      if (typeof result !== 'undefined' && result instanceof WbGeometry) {
        if (typeof currentNode !== 'undefined' && currentNode instanceof WbShape) {
          if (typeof currentNode.geometry !== 'undefined')
            currentNode.geometry.delete();
          currentNode.geometry = result;
        }
      } else if (node.tagName === 'PBRAppearance') {
        if (typeof currentNode !== 'undefined' && currentNode instanceof WbShape) {
          if (typeof currentNode.appearance !== 'undefined')
            currentNode.appearance.delete();
          result = await this.parsePBRAppearance(node, id);
          currentNode.appearance = result;
        }
      } else if (node.tagName === 'Appearance') {
        if (typeof currentNode !== 'undefined' && currentNode instanceof WbShape) {
          if (typeof currentNode.appearance !== 'undefined')
            currentNode.appearance.delete();

          result = await this.parseAppearance(node, id);
          currentNode.appearance = result;
        }
      } else if (node.tagName === 'Material') {
        result = await this.parseMaterial(node, id);
        if (typeof result !== 'undefined') {
          if (typeof currentNode !== 'undefined' && currentNode instanceof WbAppearance) {
            if (typeof currentNode.material !== 'undefined')
              currentNode.material.delete();
            currentNode.material = result;
          }
        }
      } else if (node.tagName === 'ImageTexture') {
        result = await this.parseImageTexture(node, id);
        if (typeof result !== 'undefined') {
          if (typeof currentNode !== 'undefined' && currentNode instanceof WbAppearance) {
            if (typeof currentNode.material !== 'undefined')
              currentNode.texture.delete();
            currentNode.texture = result;
          }
        }
      } else if (node.tagName === 'TextureTransform') {
        result = await this.parseTextureTransform(node, id);
        if (typeof result !== 'undefined') {
          if (typeof currentNode !== 'undefined' && currentNode instanceof WbAbstractAppearance) {
            if (typeof currentNode.textureTransform !== 'undefined')
              currentNode.textureTransform.delete();
            currentNode.textureTransform = result;
          }
        }
      } else {
        console.log(node.tagName);
        console.error("The parser doesn't support this type of node");
      }
    }

    // check if top-level nodes
    if (typeof result !== 'undefined' && typeof currentNode === 'undefined')
      WbWorld.instance.sceneTree.push(result);

    return result;
  }

  async parseChildren(node, currentNode, isBoundingObject) {
    for (let i = 0; i < node.childNodes.length; i++) {
      const child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined')
        await this.parseNode(child, currentNode, isBoundingObject);
    }
    return 1;
  }

  async parseScene(node) {
    const prefix = DefaultUrl.wrenImagesUrl();
    const id = getNodeAttribute(node, 'id');
    const smaaAreaTexture = await Parser.loadTextureData(prefix + 'smaa_area_texture.png');
    smaaAreaTexture.isTranslucent = false;
    const smaaSearchTexture = await Parser.loadTextureData(prefix + 'smaa_search_texture.png');
    smaaSearchTexture.isTranslucent = false;
    const gtaoNoiseTexture = await Parser.loadTextureData(prefix + 'gtao_noise_texture.png');
    gtaoNoiseTexture.isTranslucent = true;
    return new WbScene(id, smaaAreaTexture, smaaSearchTexture, gtaoNoiseTexture);
  }

  parseWorldInfo(node) {
    const basicTimeStep = parseInt(getNodeAttribute(node, 'basicTimeStep', '32'));
    WbWorld.instance.basicTimeStep = basicTimeStep;
    const coordinateSystem = getNodeAttribute(node, 'coordinateSystem', 'NUE');
    WbWorld.instance.coordinateSystem = coordinateSystem;
    WbWorld.computeUpVector();
  }

  parseViewpoint(node) {
    const id = getNodeAttribute(node, 'id');
    const fieldOfView = parseFloat(getNodeAttribute(node, 'fieldOfView', M_PI_4));
    const orientation = convertStringToQuaternion(getNodeAttribute(node, 'orientation', '0 1 0 0'));
    const position = convertStringToVec3(getNodeAttribute(node, 'position', '0 0 10'));
    const exposure = parseFloat(getNodeAttribute(node, 'exposure', '1.0'));
    const bloomThreshold = parseFloat(getNodeAttribute(node, 'bloomThreshold'));
    const far = parseFloat(getNodeAttribute(node, 'far', '2000'));
    const zNear = parseFloat(getNodeAttribute(node, 'zNear', '0.1'));
    const followSmoothness = parseFloat(getNodeAttribute(node, 'followSmoothness'));
    const followedId = getNodeAttribute(node, 'followedId');
    const ambientOcclusionRadius = parseFloat(getNodeAttribute(node, 'ambientOcclusionRadius', 2));

    return new WbViewpoint(id, fieldOfView, orientation, position, exposure, bloomThreshold, zNear, far, followSmoothness, followedId, ambientOcclusionRadius);
  }

  async parseBackground(node) {
    const id = getNodeAttribute(node, 'id');
    const skyColor = convertStringToVec3(getNodeAttribute(node, 'skyColor', '0 0 0'));
    const luminosity = parseFloat(getNodeAttribute(node, 'luminosity', '1'));

    let backUrl = getNodeAttribute(node, 'backUrl');
    let bottomUrl = getNodeAttribute(node, 'bottomUrl');
    let frontUrl = getNodeAttribute(node, 'frontUrl');
    let leftUrl = getNodeAttribute(node, 'leftUrl');
    let rightUrl = getNodeAttribute(node, 'rightUrl');
    let topUrl = getNodeAttribute(node, 'topUrl');

    const cubeImages = [];
    if (typeof backUrl !== 'undefined' && typeof bottomUrl !== 'undefined' && typeof frontUrl !== 'undefined' && typeof leftUrl !== 'undefined' && typeof rightUrl !== 'undefined' && typeof topUrl !== 'undefined') {
      console.log('load cubeimages');
      backUrl = backUrl.slice(1, backUrl.length - 1);
      bottomUrl = bottomUrl.slice(1, bottomUrl.length - 1);
      frontUrl = frontUrl.slice(1, frontUrl.length - 1);
      leftUrl = leftUrl.slice(1, leftUrl.length - 1);
      rightUrl = rightUrl.slice(1, rightUrl.length - 1);
      topUrl = topUrl.slice(1, topUrl.length - 1);

      if(WbWorld.instance.coordinateSystem === 'ENU'){
        cubeImages[0] = await Parser.loadTextureData(this.prefix + backUrl, false, 90);
        cubeImages[4] = await Parser.loadTextureData(this.prefix + bottomUrl, false, -90);
        cubeImages[1] = await Parser.loadTextureData(this.prefix + frontUrl, false, -90);
        cubeImages[3] = await Parser.loadTextureData(this.prefix + leftUrl, false, 180);
        cubeImages[2] = await Parser.loadTextureData(this.prefix + rightUrl);
        cubeImages[5] = await Parser.loadTextureData(this.prefix + topUrl, false, -90);
      } else{
        cubeImages[5] = await Parser.loadTextureData(this.prefix + backUrl);
        cubeImages[3] = await Parser.loadTextureData(this.prefix + bottomUrl);
        cubeImages[4] = await Parser.loadTextureData(this.prefix + frontUrl);
        cubeImages[1] = await Parser.loadTextureData(this.prefix + leftUrl);
        cubeImages[0] = await Parser.loadTextureData(this.prefix + rightUrl);
        cubeImages[2] = await Parser.loadTextureData(this.prefix + topUrl);
      }
    } else
      console.log('Background : Incomplete cubemap');

    let backIrradianceUrl = getNodeAttribute(node, 'backIrradianceUrl');
    let bottomIrradianceUrl = getNodeAttribute(node, 'bottomIrradianceUrl');
    let frontIrradianceUrl = getNodeAttribute(node, 'frontIrradianceUrl');
    let leftIrradianceUrl = getNodeAttribute(node, 'leftIrradianceUrl');
    let rightIrradianceUrl = getNodeAttribute(node, 'rightIrradianceUrl');
    let topIrradianceUrl = getNodeAttribute(node, 'topIrradianceUrl');

    const irradianceCubeURL = [];
    if (typeof backIrradianceUrl !== 'undefined' && typeof bottomIrradianceUrl !== 'undefined' && typeof frontIrradianceUrl !== 'undefined' && typeof leftIrradianceUrl !== 'undefined' && typeof rightIrradianceUrl !== 'undefined' && typeof topIrradianceUrl !== 'undefined') {
      console.log('load irradianceCubeImages');
      backIrradianceUrl = backIrradianceUrl.slice(1, backIrradianceUrl.length - 1);
      bottomIrradianceUrl = bottomIrradianceUrl.slice(1, bottomIrradianceUrl.length - 1);
      frontIrradianceUrl = frontIrradianceUrl.slice(1, frontIrradianceUrl.length - 1);
      leftIrradianceUrl = leftIrradianceUrl.slice(1, leftIrradianceUrl.length - 1);
      rightIrradianceUrl = rightIrradianceUrl.slice(1, rightIrradianceUrl.length - 1);
      topIrradianceUrl = topIrradianceUrl.slice(1, topIrradianceUrl.length - 1);
      if(WbWorld.instance.coordinateSystem === 'ENU'){
        irradianceCubeURL[0] = await Parser.loadTextureData(this.prefix + backIrradianceUrl, true, 90);
        irradianceCubeURL[4] = await Parser.loadTextureData(this.prefix + bottomIrradianceUrl, true, -90);
        irradianceCubeURL[1] = await Parser.loadTextureData(this.prefix + frontIrradianceUrl, true, -90);
        irradianceCubeURL[3] = await Parser.loadTextureData(this.prefix + leftIrradianceUrl, true, 180);
        irradianceCubeURL[2] = await Parser.loadTextureData(this.prefix + rightIrradianceUrl, true);
        irradianceCubeURL[5] = await Parser.loadTextureData(this.prefix + topIrradianceUrl, true, -90);
      } else{
        irradianceCubeURL[2] = await Parser.loadTextureData(this.prefix + topIrradianceUrl, true);
        irradianceCubeURL[5] = await Parser.loadTextureData(this.prefix + backIrradianceUrl, true);
        irradianceCubeURL[3] = await Parser.loadTextureData(this.prefix + bottomIrradianceUrl, true);
        irradianceCubeURL[4] = await Parser.loadTextureData(this.prefix + frontIrradianceUrl, true);
        irradianceCubeURL[1] = await Parser.loadTextureData(this.prefix + leftIrradianceUrl, true);
        irradianceCubeURL[0] = await Parser.loadTextureData(this.prefix + rightIrradianceUrl, true);
      }

    } else
      console.log('Background : Incomplete irradiance cubemap');

    const background = new WbBackground(id, skyColor, luminosity, cubeImages, irradianceCubeURL);
    WbBackground.instance = background;

    WbWorld.instance.nodes.set(background.id, background);

    return background;
  }

  async checkUse(node, currentNode) {
    let use = getNodeAttribute(node, 'USE');
    if (typeof use === 'undefined')
      return;

    const id = getNodeAttribute(node, 'id');
    let result = WbWorld.instance.nodes.get(use);

    if (typeof result === 'undefined') {
      use = 'n' + use;
      result = WbWorld.instance.nodes.get(use);
    }

    if (typeof result === 'undefined')
      return;

    const useNode = await result.clone(id);
    if (typeof currentNode !== 'undefined') {
      useNode.parent = currentNode.id;
      if (result instanceof WbShape || result instanceof WbGroup || result instanceof WbLight)
        currentNode.children.push(useNode);
    }

    WbWorld.instance.nodes.set(id, useNode);
    return useNode;
  }

  async parseTransform(node, currentNode, isBoundingObject) {
    const use = await this.checkUse(node, currentNode);
    if (typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if (typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;
    const isSolid = getNodeAttribute(node, 'solid', 'false').toLowerCase() === 'true';
    const translation = convertStringToVec3(getNodeAttribute(node, 'translation', '0 0 0'));
    const scale = convertStringToVec3(getNodeAttribute(node, 'scale', '1 1 1'));
    const rotation = convertStringToQuaternion(getNodeAttribute(node, 'rotation', '0 1 0 0'));

    const transform = new WbTransform(id, isSolid, translation, scale, rotation);

    WbWorld.instance.nodes.set(transform.id, transform);

    await this.parseChildren(node, transform, isBoundingObject);

    if (typeof currentNode !== 'undefined') {
      transform.parent = currentNode.id;
      currentNode.children.push(transform);
    }

    return transform;
  }

  async parseGroup(node, currentNode, isBoundingObject) {
    const use = await this.checkUse(node, currentNode);
    if (typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if (typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

    const isPropeller = getNodeAttribute(node, 'isPropeller', 'false').toLowerCase() === 'true';

    const group = new WbGroup(id, isPropeller);

    WbWorld.instance.nodes.set(group.id, group);
    await this.parseChildren(node, group, isBoundingObject);

    if (typeof currentNode !== 'undefined') {
      group.parent = currentNode.id;
      currentNode.children.push(group);
    }

    return group;
  }

  async parseShape(node, currentNode, isBoundingObject) {
    const use = await this.checkUse(node, currentNode);
    if (typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if (typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

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
          appearance = await this.parseAppearance(child);
        } else if (child.tagName === 'PBRAppearance')
          appearance = await this.parsePBRAppearance(child);
        if (typeof appearance !== 'undefined')
          continue;
      }

      if (typeof geometry === 'undefined') {
        geometry = await this.parseGeometry(child, id);
        if (typeof geometry !== 'undefined')
          continue;
      }

      console.log('X3dLoader: Unknown node: ' + child.tagName);
    }

    if (isBoundingObject)
      appearance = undefined;

    const shape = new WbShape(id, castShadows, isPickable, geometry, appearance);

    if (typeof currentNode !== 'undefined') {
      currentNode.children.push(shape);
      shape.parent = currentNode.id;
    }

    if (typeof appearance !== 'undefined')
      appearance.parent = shape.id;

    WbWorld.instance.nodes.set(shape.id, shape);

    return shape;
  }

  async parseDirectionalLight(node, currentNode) {
    const use = await this.checkUse(node, currentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');
    const on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const direction = convertStringToVec3(getNodeAttribute(node, 'direction', '0 0 -1'));
    const intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    const ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    const castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    const dirLight = new WbDirectionalLight(id, on, color, direction, intensity, castShadows, ambientIntensity);

    if (typeof currentNode !== 'undefined' && typeof dirLight !== 'undefined') {
      currentNode.children.push(dirLight);
      dirLight.parent = currentNode.id;
    }

    WbWorld.instance.nodes.set(dirLight.id, dirLight);

    return dirLight;
  }

  async parsePointLight(node, currentNode) {
    const use = await this.checkUse(node, currentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');
    const on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    const attenuation = convertStringToVec3(getNodeAttribute(node, 'attenuation', '1 0 0'));
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    const location = convertStringToVec3(getNodeAttribute(node, 'location', '0 0 0'));
    const radius = parseFloat(getNodeAttribute(node, 'radius', '100'));
    const ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    const castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    const pointLight = new WbPointLight(id, on, attenuation, color, intensity, location, radius, ambientIntensity, castShadows, currentNode);

    if (typeof currentNode !== 'undefined' && typeof pointLight !== 'undefined')
      currentNode.children.push(pointLight);

    WbWorld.instance.nodes.set(pointLight.id, pointLight);

    return pointLight;
  }

  async parseSpotLight(node, currentNode) {
    const use = await this.checkUse(node, currentNode);
    if (typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');
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

    const spotLight = new WbSpotLight(id, on, attenuation, beamWidth, color, cutOffAngle, direction, intensity, location, radius, ambientIntensity, castShadows, currentNode);

    if (typeof currentNode !== 'undefined' && typeof spotLight !== 'undefined')
      currentNode.children.push(spotLight);

    WbWorld.instance.nodes.set(spotLight.id, spotLight);

    return spotLight;
  }

  async parseFog(node) {
    const id = getNodeAttribute(node, 'id');
    const color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    const visibilityRange = parseFloat(getNodeAttribute(node, 'visibilityRange', '0'));
    const fogType = getNodeAttribute(node, 'fogType', 'LINEAR');

    const fog = new WbFog(id, color, visibilityRange, fogType);

    WbWorld.instance.nodes.set(fog.id, fog);

    if (typeof fog !== 'undefined')
      WbWorld.instance.hasFog = true;

    return fog;
  }

  async parseGeometry(node, parentId) {
    const use = await this.checkUse(node);
    if (typeof use !== 'undefined') {
      use.parent = parentId;
      return use;
    }

    let id = getNodeAttribute(node, 'id');
    if (typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

    let geometry;
    if (node.tagName === 'Box')
      geometry = this.parseBox(node, id);
    else if (node.tagName === 'Sphere')
      geometry = this.parseSphere(node, id);
    else if (node.tagName === 'Cone')
      geometry = this.parseCone(node, id);
    else if (node.tagName === 'Plane')
      geometry = this.parsePlane(node, id);
    else if (node.tagName === 'Cylinder')
      geometry = this.parseCylinder(node, id);
    else if (node.tagName === 'Capsule')
      geometry = this.parseCapsule(node, id);
    else if (node.tagName === 'IndexedFaceSet')
      geometry = this.parseIndexedFaceSet(node, id);
    else if (node.tagName === 'IndexedLineSet')
      geometry = this.parseIndexedLineSet(node, id);
    else if (node.tagName === 'ElevationGrid')
      geometry = this.parseElevationGrid(node, id);
    else if (node.tagName === 'PointSet')
      geometry = this.parsePointSet(node, id);
    else {
      console.log('Not a recognized geometry : ' + node.tagName);
      geometry = undefined;
    }

    if (typeof parentId !== 'undefined' && typeof geometry !== 'undefined')
      geometry.parent = parentId;

    return geometry;
  }

  parseBox(node, id) {
    const size = convertStringToVec3(getNodeAttribute(node, 'size', '2 2 2'));

    const box = new WbBox(id, size);
    WbWorld.instance.nodes.set(box.id, box);
    return box;
  }

  parseSphere(node, id) {
    const radius = parseFloat(getNodeAttribute(node, 'radius', '1'));
    const ico = getNodeAttribute(node, 'ico', 'false').toLowerCase() === 'true';
    const subdivision = parseInt(getNodeAttribute(node, 'subdivision', '1,1'));

    const sphere = new WbSphere(id, radius, ico, subdivision);

    WbWorld.instance.nodes.set(sphere.id, sphere);

    return sphere;
  }

  parseCone(node, id) {
    const bottomRadius = getNodeAttribute(node, 'bottomRadius', '1');
    const height = getNodeAttribute(node, 'height', '2');
    const subdivision = getNodeAttribute(node, 'subdivision', '32');
    const side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    const bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';

    const cone = new WbCone(id, bottomRadius, height, subdivision, side, bottom);

    WbWorld.instance.nodes.set(cone.id, cone);

    return cone;
  }

  parseCylinder(node, id) {
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

  parsePlane(node, id) {
    const size = convertStringToVec2(getNodeAttribute(node, 'size', '1,1'));

    const plane = new WbPlane(id, size);

    WbWorld.instance.nodes.set(plane.id, plane);

    return plane;
  }

  parseCapsule(node, id) {
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

  parseIndexedFaceSet(node, id) {
    const coordIndexStr = getNodeAttribute(node, 'coordIndex', '').trim().split(/\s/); ;
    const coordIndex = coordIndexStr.map(Number).filter(el => { return el !== -1; });

    const normalIndexStr = getNodeAttribute(node, 'normalIndex', '').trim().split(/\s/);
    const normalIndex = normalIndexStr.map(Number).filter(el => { return el !== -1; });

    const texCoordIndexStr = getNodeAttribute(node, 'texCoordIndex', '').trim().split(/\s/); ;
    const texCoordIndex = texCoordIndexStr.map(Number).filter(el => { return el !== -1; });
    // console.log(texCoordIndexStr.map(Number).filter(el => { return el !== -1; }));

    const coordArray = [];
    const coordinate = node.getElementsByTagName('Coordinate')[0];
    if (typeof coordinate !== 'undefined') {
      const coordStr = getNodeAttribute(coordinate, 'point', '').split(/\s/);
      const coord = coordStr.map(el => parseFloat(el));
      for (let i = 0; i < coord.length; i = i + 3)
        coordArray.push(new WbVector3(coord[i], coord[i + 1], coord[i + 2]));
    }

    const texCoordArray = [];
    const textureCoordinate = node.getElementsByTagName('TextureCoordinate')[0];
    if (typeof textureCoordinate !== 'undefined') {
      const texcoordsStr = getNodeAttribute(textureCoordinate, 'point', '').split(/\s/);
      const texCoord = texcoordsStr.map(el => parseFloat(el));
      for (let i = 0; i < texCoord.length; i = i + 2)
        texCoordArray.push(new WbVector2(texCoord[i], texCoord[i + 1]));
    }

    const normalArray = [];
    const normalNode = node.getElementsByTagName('Normal')[0];
    if (typeof normalNode !== 'undefined') {
      const normalStr = getNodeAttribute(normalNode, 'vector', '').split(/[\s,]+/);
      const normal = normalStr.map(el => parseFloat(el));
      for (let i = 0; i < normal.length; i = i + 3)
        normalArray.push(new WbVector3(normal[i], normal[i + 1], normal[i + 2]));
    }

    const creaseAngle = parseFloat(getNodeAttribute(node, 'creaseAngle', '0'));
    const ccw = parseFloat(getNodeAttribute(node, 'ccw', '1'));
    const normalPerVertex = parseFloat(getNodeAttribute(node, 'normalPerVertex', '1'));
    const ifs = new WbIndexedFaceSet(id, coordIndex, normalIndex, texCoordIndex, coordArray, texCoordArray, normalArray, creaseAngle, ccw, normalPerVertex);
    WbWorld.instance.nodes.set(ifs.id, ifs);

    return ifs;
  }

  parseIndexedLineSet(node, id) {
    const coordinate = node.getElementsByTagName('Coordinate')[0];

    if (typeof coordinate === 'undefined')
      return undefined;

    const indicesStr = getNodeAttribute(node, 'coordIndex', '').trim().split(/\s/);

    const verticesStr = getNodeAttribute(coordinate, 'point', '').trim().split(/\s/);

    const coord = [];
    for (let i = 0; i < verticesStr.length; i += 3)
      coord.push(new WbVector3(parseFloat(verticesStr[i]), parseFloat(verticesStr[i + 1]), parseFloat(verticesStr[i + 2])));

    const coordIndex = indicesStr.map(Number);

    const ils = new WbIndexedLineSet(id, coord, coordIndex);
    WbWorld.instance.nodes.set(ils.id, ils);

    return ils;
  }

  parseElevationGrid(node, id) {
    const heightStr = getNodeAttribute(node, 'height', undefined);
    if (typeof heightStr === 'undefined')
      return;

    const xDimension = parseInt(getNodeAttribute(node, 'xDimension', '0'));
    const xSpacing = parseFloat(getNodeAttribute(node, 'xSpacing', '1'));
    const zDimension = parseInt(getNodeAttribute(node, 'zDimension', '0'));
    const zSpacing = parseFloat(getNodeAttribute(node, 'zSpacing', '1'));
    const thickness = parseFloat(getNodeAttribute(node, 'thickness', '1'));

    const height = heightStr.split(' ').map(Number);

    const eg = new WbElevationGrid(id, height, xDimension, xSpacing, zDimension, zSpacing, thickness);
    WbWorld.instance.nodes.set(eg.id, eg);

    return eg;
  }

  parsePointSet(node, id) {
    const coordinate = node.getElementsByTagName('Coordinate')[0];

    if (typeof coordinate === 'undefined')
      return;

    const coordStrArray = getNodeAttribute(coordinate, 'point', '').trim().split(/\s/);

    if (typeof coordStrArray === 'undefined')
      return;

    const coordArray = coordStrArray.map(Number);
    const coord = [];
    for (let i = 0; i < coordArray.length; i += 3)
      coord.push(new WbVector3(coordArray[i], coordArray[i + 1], coordArray[i + 2]));

    const colorNode = node.getElementsByTagName('Color')[0];
    let color;
    if (typeof colorNode !== 'undefined') {
      const colorStrArray = getNodeAttribute(colorNode, 'color', '').trim().split(/\s/);
      if (typeof colorStrArray !== 'undefined') {
        const colorArray = colorStrArray.map(Number);
        color = [];
        for (let i = 0; i < colorArray.length; i += 3)
          color.push(new WbVector3(colorArray[i], colorArray[i + 1], colorArray[i + 2]));
      }
    }

    const ps = new WbPointSet(id, coord, color);
    WbWorld.instance.nodes.set(ps.id, ps);

    return ps;
  }

  async parseSwitch(node, parent) {
    if (typeof parent === 'undefined')
      return;

    const child = node.childNodes[0];

    let boundingObject;
    if (child.tagName === 'Shape')
      boundingObject = await this.parseShape(child, undefined, true);
    else if (child.tagName === 'Transform')
      boundingObject = await this.parseTransform(child, undefined, true);
    else if (child.tagName === 'Group')
      boundingObject = await this.parseGroup(child, undefined, true);
    else
      console.error('Unknown boundingObject: ' + child.tagName);

    if (typeof boundingObject === 'undefined')
      return;

    boundingObject.parent = parent.id;
    parent.boundingObject = boundingObject;

    return boundingObject;
  }

  async parseAppearance(node, parentId) {
    const use = await this.checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if (typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

    // Get the Material tag.
    const materialNode = node.getElementsByTagName('Material')[0];
    let material;
    if (typeof materialNode !== 'undefined')
      material = await this.parseMaterial(materialNode);

    // Check to see if there is a texture.
    const imageTexture = node.getElementsByTagName('ImageTexture')[0];
    let texture;
    if (typeof imageTexture !== 'undefined')
      texture = await this.parseImageTexture(imageTexture);

    // Check to see if there is a textureTransform.
    const textureTransform = node.getElementsByTagName('TextureTransform')[0];
    let transform;
    if (typeof textureTransform !== 'undefined')
      transform = await this.parseTextureTransform(textureTransform);

    const appearance = new WbAppearance(id, material, texture, transform);
    if (typeof appearance !== 'undefined') {
      if (typeof material !== 'undefined')
        material.parent = appearance.id;
    }

    if (typeof appearance !== 'undefined') {
      if (typeof texture !== 'undefined')
        texture.parent = appearance.id;
    }

    if (typeof appearance !== 'undefined') {
      if (typeof transform !== 'undefined')
        transform.parent = appearance.id;
    }

    if (typeof parentId !== 'undefined')
      appearance.parent = parentId;

    WbWorld.instance.nodes.set(appearance.id, appearance);

    return appearance;
  }

  async parseMaterial(node, parentId) {
    const use = await this.checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if (typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

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

  async parseImageTexture(node, parentId) {
    const use = await this.checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');
    let url = getNodeAttribute(node, 'url', '');
    url = url.slice(1, url.length - 1);
    const isTransparent = getNodeAttribute(node, 'isTransparent', 'false').toLowerCase() === 'true';
    const s = getNodeAttribute(node, 'repeatS', 'true').toLowerCase() === 'true';
    const t = getNodeAttribute(node, 'repeatT', 'true').toLowerCase() === 'true';
    const filtering = parseFloat(getNodeAttribute(node, 'filtering', '4'));

    let imageTexture;
    if (typeof url !== 'undefined' && url !== '') {
      url = this.prefix + url;
      imageTexture = new WbImageTexture(id, url, isTransparent, s, t, filtering);
      await imageTexture.updateUrl();
    }

    if (typeof imageTexture !== 'undefined') {
      if (typeof parentId !== 'undefined')
        imageTexture.parent = parentId;

      WbWorld.instance.nodes.set(imageTexture.id, imageTexture);
    }

    return imageTexture;
  }

  async parsePBRAppearance(node, parentId) {
    const use = await this.checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');

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
      transform = await this.parseTextureTransform(textureTransform);

    const imageTextures = node.getElementsByTagName('ImageTexture');
    let baseColorMap, roughnessMap, metalnessMap, normalMap, occlusionMap, emissiveColorMap;
    for (let i = 0; i < imageTextures.length; i++) {
      const imageTexture = imageTextures[i];
      const type = getNodeAttribute(imageTexture, 'type', undefined);
      if (type === 'baseColor') {
        baseColorMap = await this.parseImageTexture(imageTexture);
        if (typeof baseColorMap !== 'undefined')
          baseColorMap.type = 'baseColorMap';
      } else if (type === 'roughness') {
        roughnessMap = await this.parseImageTexture(imageTexture);
        if (typeof roughnessMap !== 'undefined')
          roughnessMap.type = 'roughnessMap';
      } else if (type === 'metalness') {
        metalnessMap = await this.parseImageTexture(imageTexture);
        if (typeof metalnessMap !== 'undefined')
          metalnessMap.type = 'metalnessMap';
      } else if (type === 'normal') {
        normalMap = await this.parseImageTexture(imageTexture);
        if (typeof normalMap !== 'undefined')
          normalMap.type = 'normalMap';
      } else if (type === 'occlusion') {
        occlusionMap = await this.parseImageTexture(imageTexture);
        if (typeof occlusionMap !== 'undefined')
          occlusionMap.type = 'occlusionMap';
      } else if (type === 'emissiveColor') {
        emissiveColorMap = await this.parseImageTexture(imageTexture);
        if (typeof emissiveColorMap !== 'undefined')
          emissiveColorMap.type = 'emissiveColorMap';
      }
    }

    const pbrAppearance = new WbPBRAppearance(id, baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness, metalnessMap, IBLStrength,
      normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor, emissiveColorMap, emissiveIntensity, transform);

    if (typeof pbrAppearance !== 'undefined') {
      if (typeof transform !== 'undefined')
        transform.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
      if (typeof baseColorMap !== 'undefined')
        baseColorMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
      if (typeof roughnessMap !== 'undefined')
        roughnessMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
      if (typeof metalnessMap !== 'undefined')
        metalnessMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
      if (typeof normalMap !== 'undefined')
        normalMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
      if (typeof occlusionMap !== 'undefined')
        occlusionMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
      if (typeof emissiveColorMap !== 'undefined')
        emissiveColorMap.parent = pbrAppearance.id;
    }

    if (typeof parentId !== 'undefined')
      pbrAppearance.parent = parentId;

    WbWorld.instance.nodes.set(pbrAppearance.id, pbrAppearance);

    return pbrAppearance;
  }

  async parseTextureTransform(node, parentId) {
    const use = await this.checkUse(node);
    if (typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');
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

  static async loadTextureData(url, isHdr, rotation) {
    const canvas2 = document.createElement('canvas');
    const context = canvas2.getContext('2d');

    const image = new WbImage();

    if (isHdr) {
      const img = await Parser.loadHDRImage(url);
      image.bits = img.data;
      image.width = img.width;
      image.height = img.height;
      image.url = url;
      if(typeof rotation !== 'undefined') {
        image.bits = rotateHDR(image, rotation);
      }
    } else {
      const img = await Parser.loadImage(url);
      canvas2.width = img.width;
      canvas2.height = img.height;
      if(typeof rotation !== 'undefined') {
        context.save();
        context.translate(canvas2.width/2, canvas2.height/2);
        context.rotate(rotation * Math.PI / 180);
        context.drawImage(img, -canvas2.width/2, -canvas2.height/2);
        context.restore();
      } else {
        context.drawImage(img, 0, 0);
      }

      const dataBGRA = context.getImageData(0, 0, img.width, img.height).data;
      let data = new Uint8ClampedArray(dataBGRA.length);
      data = dataBGRA;

      image.bits = data;
      image.width = img.width;
      image.height = img.height;
      image.url = url;
    }
    return image;
  }

  static loadImage(src) {
    return new Promise((resolve, reject) => {
      const img = new Image();
      img.onload = () => {
        resolve(img);
      };
      img.onerror = () => console.log('Error in loading : ' + src);
      img.setAttribute('crossOrigin', '');
      img.src = src;
    });
  }

  static loadHDRImage(src) {
    return new Promise((resolve, reject) => {
      const loader = new RGBELoader();
      loader.load(src, function(img) { resolve(img); });
    });
  }
}

function getNodeAttribute(node, attributeName, defaultValue) {
  console.assert(node && node.attributes);
  if (attributeName in node.attributes)
    return node.attributes.getNamedItem(attributeName).value;
  return defaultValue;
}

function convertStringToVec2(s) {
  s = s.split(/\s/);
  return new WbVector2(parseFloat(s[0]), parseFloat(s[1]));
}

function convertStringToVec3(s) {
  s = s.split(/\s/);
  return new WbVector3(parseFloat(s[0]), parseFloat(s[1]), parseFloat(s[2]));
}

function convertStringToQuaternion(s) {
  const pos = s.split(/\s/);
  return new WbVector4(parseFloat(pos[0]), parseFloat(pos[1]), parseFloat(pos[2]), parseFloat(pos[3]));
}

function rotateHDR(image, rotate) {
  let rotatedbits = [];
  if (rotate == 90) {
    for (let x = 0; x < image.width; x++) {
      for (let y = 0; y < image.height; y++) {
        const u = y * image.width * 3 + x * 3;
        const v = (image.width - 1 - x) * image.height * 3 + y * 3;
        for (let c = 0; c < 3; c++){
          rotatedbits[u + c] = image.bits[v + c];
        }
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

Parser.undefinedID = 90000;
export {Parser, convertStringToVec3, convertStringToQuaternion};
