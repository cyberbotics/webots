
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

import {World} from "./webotsjs/World.js"
import {WbScene} from "./webotsjs/WbScene.js";
import {WbViewpoint} from "./webotsjs/WbViewpoint.js";
import {WbBackground} from "./webotsjs/WbBackground.js";

import {WbGroup} from "./webotsjs/WbGroup.js";
import {WbTransform} from "./webotsjs/WbTransform.js";
import {WbShape} from "./webotsjs/WbShape.js";

import {WbBox} from "./webotsjs/WbBox.js";
import {WbCylinder} from "./webotsjs/WbCylinder.js";
import {WbPlane} from "./webotsjs/WbPlane.js";
import {WbSphere} from "./webotsjs/WbSphere.js";
import {WbCone} from "./webotsjs/WbCone.js";
import {WbIndexedFaceSet} from "./webotsjs/WbIndexedFaceSet.js";
import {WbIndexedLineSet} from "./webotsjs/WbIndexedLineSet.js";
import {WbElevationGrid} from "./webotsjs/WbElevationGrid.js";
import {WbPointSet} from "./webotsjs/WbPointSet.js";

import {WbMaterial} from "./webotsjs/WbMaterial.js";
import {WbTextureTransform} from "./webotsjs/WbTextureTransform.js";
import {WbAppearance} from "./webotsjs/WbAppearance.js";
import {WbPBRAppearance} from "./webotsjs/WbPBRAppearance.js";
import {WbImageTexture} from "./webotsjs/WbImageTexture.js";
import {WbImage} from "./webotsjs/WbImage.js";

import {WbDirectionalLight} from "./webotsjs/WbDirectionalLight.js";
import {WbPointLight} from "./webotsjs/WbPointLight.js";
import {WbSpotLight} from "./webotsjs/WbSpotLight.js";
import {WbLight} from "./webotsjs/WbLight.js";
import {WbFog} from "./webotsjs/WbFog.js"

import {Use} from "./webotsjs/Use.js";
import {WbVector2} from "./webotsjs/utils/WbVector2.js";
import {WbVector3} from "./webotsjs/utils/WbVector3.js";
import {WbVector4} from "./webotsjs/utils/WbVector4.js";
import {RGBELoader} from "./hdrLoader.js"

import {DefaultUrl} from "./default_url.js"

class Parser {
  constructor(prefix = '') {
      this.prefix = prefix;
      let world = new World();
  }

  async parse(text, renderer, parent){
    console.log('X3D: Parsing');
    let xml = null;
    if (window.DOMParser) {
      let parser = new DOMParser();
      xml = parser.parseFromString(text, 'text/xml');
    } else { // Internet Explorer
      xml = new ActiveXObject('Microsoft.XMLDOM');
      xml.async = false;
      xml.loadXML(text);
    }
    if (typeof xml === 'undefined') {
      console.error("File to parse not found");
    } else {
      let scene = xml.getElementsByTagName('Scene')[0];
      if (typeof scene === 'undefined') {
        let node = xml.getElementsByTagName('nodes')[0];
        console.log(node);
        if (typeof node === 'undefined')
          console.error("Unknown content, nor Scene, nor Node");
        else
          await this.parseChildren(node, parent);
      } else{
        console.log(scene);
        await this.parseNode(scene);
      }
    }

    console.log(World.instance);
    if(typeof World.instance.viewpoint == 'undefined')
      return;
    World.instance.viewpoint.finalize();
    console.timeEnd('startID')
    console.time('startID')
    World.instance.sceneTree.forEach(node => {
      node.finalize();});
    renderer.render();
    console.timeEnd('startID')
  }

  async parseFile(file) {
    let scene = file.getElementsByTagName('Scene')[0];
    await this.parseNode(scene);
    console.log(World.instance);
    World.instance.viewpoint.finalize();
    World.instance.sceneTree.forEach(node => {
      node.finalize();});
  }

  async parseNode(node, currentNode, isBoundingObject) {
    if(typeof World.instance === 'undefined') {
      new World();
    }

    let result;
    if(node.tagName === 'Scene') {
      World.instance.scene = await this.parseScene(node);
      await this.parseChildren(node, currentNode)
    } else if (node.tagName === 'WorldInfo')
      this.parseWorldInfo(node);
    else if (node.tagName === 'Viewpoint')
      World.instance.viewpoint = this.parseViewpoint(node);
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
    else if (node.tagName === 'Fog')
      if(!World.instance.hasFog)
        result = await this.parseFog(node);
      else{
        console.error("This world already has a fog.");
      }
    else {
      //Either it is a node added after the whole scene, or it is an unknown node
      let id;
      if (typeof currentNode !== 'undefined')
        id = currentNode.id
      result = await this.parseGeometry(node, id);
      if (typeof result !== 'undefined') {
        if (typeof currentNode !== 'undefined' && currentNode instanceof WbShape){
          if(typeof currentNode.geometry !== 'undefined')
            currentNode.geometry.delete();
          currentNode.geometry = result;
        }
      } else if (node.tagName === 'PBRAppearance') {
        if (typeof currentNode !== 'undefined' && currentNode instanceof WbShape){
          if(typeof currentNode.appearance !== 'undefined')
            currentNode.appearance.delete();
          result = await this.parsePBRAppearance(node, id);
          currentNode.appearance = result;
        }
      } else if (node.tagName === 'Appearance') {
        if (typeof currentNode !== 'undefined' && currentNode instanceof WbShape){
          if(typeof currentNode.appearance !== 'undefined')
            currentNode.appearance.delete();

          result = await this.parseAppearance(node, id);
          currentNode.appearance = result;
        }
      } else if (node.tagName === 'Material') {
        result = await this.parseMaterial(node, id);
        if (typeof result !== 'undefined') {
          if (typeof currentNode !== 'undefined' && currentNode instanceof WbAppearance){
            if(typeof currentNode.material !== 'undefined')
              currentNode.material.delete();
            currentNode.material = result;
          }
        }
      } else if (node.tagName === 'ImageTexture') {
        result = await this.parseImageTexture(node, id);
        if (typeof result !== 'undefined') {
          if (typeof currentNode !== 'undefined' && currentNode instanceof WbAppearance){
            if(typeof currentNode.material !== 'undefined')
              currentNode.texture.delete();
            currentNode.texture = result;
          }
        }
      } else if (node.tagName === 'TextureTransform') {
        result = await this.parseTextureTransform(node, id);
        if (typeof result !== 'undefined') {
          if (typeof currentNode !== 'undefined' && currentNode instanceof WbAbstractAppearance){
            if(typeof currentNode.textureTransform !== 'undefined')
              currentNode.textureTransform.delete();
            currentNode.textureTransform = result;
          }
        }
      } else {
        console.log(node.tagName);
        console.error("The parser doesn't support this type of node");
      }
    }

    //check if top-level nodes
    if(typeof result !== 'undefined' && typeof currentNode === 'undefined')
        World.instance.sceneTree.push(result);

    return result;
  }

  async parseChildren(node, currentNode, isBoundingObject) {
    for (let i = 0; i < node.childNodes.length; i++) {
      let child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined')
          await this.parseNode(child, currentNode, isBoundingObject);
    }
    return 1;
  }

  async parseScene(node) {
    let prefix = DefaultUrl.wrenImagesUrl();
    let id = getNodeAttribute(node, 'id');
    let lensFlareLenTexture = await Parser.loadTextureData(prefix + "lens_flare.png");
    lensFlareLenTexture.isTranslucent = true;
    let smaaAreaTexture = await Parser.loadTextureData(prefix + "smaa_area_texture.png");
    smaaAreaTexture.isTranslucent = false;
    let smaaSearchTexture = await Parser.loadTextureData(prefix + "smaa_search_texture.png");
    smaaSearchTexture.isTranslucent = false;
    let gtaoNoiseTexture = await Parser.loadTextureData(prefix + "gtao_noise_texture.png");
    gtaoNoiseTexture.isTranslucent = true;
    return new WbScene(id, lensFlareLenTexture, smaaAreaTexture, smaaSearchTexture, gtaoNoiseTexture);
  }

  parseWorldInfo(node){
    let basicTimeStep = parseInt(getNodeAttribute(node, 'basicTimeStep', '32'));
    World.instance.basicTimeStep = basicTimeStep;
  }

  parseViewpoint(node){
    let id = getNodeAttribute(node, 'id');
    let orientation = convertStringToQuaternion(getNodeAttribute(node, 'orientation', '0 1 0 0'));
    let position = convertStringToVec3(getNodeAttribute(node, 'position', '0 0 10'));
    let exposure = parseFloat(getNodeAttribute(node, 'exposure', '1.0'));
    let bloomThreshold = parseFloat(getNodeAttribute(node, 'bloomThreshold'));
    let far = parseFloat(getNodeAttribute(node, 'far', '2000'));
    let zNear = parseFloat(getNodeAttribute(node, 'zNear', '0.1'));
    let followSmoothness = parseFloat(getNodeAttribute(node, 'followSmoothness'));
    let followedId = getNodeAttribute(node, 'followedId');
    let ambientOcclusionRadius = parseFloat(getNodeAttribute(node, 'ambientOcclusionRadius', 2));

    let viewpoint = new WbViewpoint(id, orientation, position, exposure, bloomThreshold, zNear, far, followSmoothness, followedId, ambientOcclusionRadius);

    return viewpoint
  }

  async parseBackground(node) {
    let id = getNodeAttribute(node, 'id');
    let skyColor = convertStringToVec3(getNodeAttribute(node, 'skyColor', '0 0 0'));
    let luminosity = parseFloat(getNodeAttribute(node, 'luminosity', '1'));

    let backUrl = getNodeAttribute(node, 'backUrl');
    let bottomUrl = getNodeAttribute(node, 'bottomUrl');
    let frontUrl = getNodeAttribute(node, 'frontUrl');
    let leftUrl = getNodeAttribute(node, 'leftUrl');
    let rightUrl = getNodeAttribute(node, 'rightUrl');
    let topUrl = getNodeAttribute(node, 'topUrl');

    let cubeImages = [];
    if(typeof backUrl !== 'undefined' && typeof bottomUrl !== 'undefined' && typeof frontUrl !== 'undefined' && typeof leftUrl !== 'undefined' && typeof rightUrl !== 'undefined' && typeof topUrl !== 'undefined'){
      //TODO add test to see if all images have the same size and alpha and are squared
      console.log("load cubeimages");
      backUrl = backUrl.slice(1, backUrl.length-1);
      bottomUrl = bottomUrl.slice(1, bottomUrl.length-1);
      frontUrl = frontUrl.slice(1, frontUrl.length-1);
      leftUrl = leftUrl.slice(1, leftUrl.length-1);
      rightUrl = rightUrl.slice(1, rightUrl.length-1);
      topUrl = topUrl.slice(1, topUrl.length-1);

      cubeImages[5] = await Parser.loadTextureData(this.prefix + backUrl);
      cubeImages[3] = await Parser.loadTextureData(this.prefix + bottomUrl);
      cubeImages[4] = await Parser.loadTextureData(this.prefix + frontUrl);
      cubeImages[1] = await Parser.loadTextureData(this.prefix + leftUrl);
      cubeImages[0] = await Parser.loadTextureData(this.prefix + rightUrl);
      cubeImages[2] = await Parser.loadTextureData(this.prefix + topUrl);
    } else {
      console.log("Background : Incomplete cubemap");
    }

    let backIrradianceUrl = getNodeAttribute(node, 'backIrradianceUrl');
    let bottomIrradianceUrl = getNodeAttribute(node, 'bottomIrradianceUrl');
    let frontIrradianceUrl = getNodeAttribute(node, 'frontIrradianceUrl');
    let leftIrradianceUrl = getNodeAttribute(node, 'leftIrradianceUrl');
    let rightIrradianceUrl = getNodeAttribute(node, 'rightIrradianceUrl');
    let topIrradianceUrl = getNodeAttribute(node, 'topIrradianceUrl');

    let irradianceCubeURL = [];
    if(typeof backIrradianceUrl !== 'undefined' && typeof bottomIrradianceUrl !== 'undefined' && typeof frontIrradianceUrl !== 'undefined' && typeof leftIrradianceUrl !== 'undefined' && typeof rightIrradianceUrl !== 'undefined' && typeof topIrradianceUrl !== 'undefined'){
      console.log("load irradianceCubeImages");
      backIrradianceUrl = backIrradianceUrl.slice(1, backIrradianceUrl.length-1);
      bottomIrradianceUrl = bottomIrradianceUrl.slice(1, bottomIrradianceUrl.length-1);
      frontIrradianceUrl = frontIrradianceUrl.slice(1, frontIrradianceUrl.length-1);
      leftIrradianceUrl = leftIrradianceUrl.slice(1, leftIrradianceUrl.length-1);
      rightIrradianceUrl = rightIrradianceUrl.slice(1, rightIrradianceUrl.length-1);
      topIrradianceUrl = topIrradianceUrl.slice(1, topIrradianceUrl.length-1);

      irradianceCubeURL[2] = await Parser.loadTextureData(this.prefix + topIrradianceUrl, true);
      irradianceCubeURL[5] = await Parser.loadTextureData(this.prefix + backIrradianceUrl, true);
      irradianceCubeURL[3] = await Parser.loadTextureData(this.prefix + bottomIrradianceUrl, true);
      irradianceCubeURL[4] = await Parser.loadTextureData(this.prefix + frontIrradianceUrl, true);
      irradianceCubeURL[1] = await Parser.loadTextureData(this.prefix + leftIrradianceUrl, true);
      irradianceCubeURL[0] = await Parser.loadTextureData(this.prefix + rightIrradianceUrl, true);
    } else {
      console.log("Background : Incomplete irradiance cubemap");
    }

    let background = new WbBackground(id, skyColor, luminosity, cubeImages, irradianceCubeURL);
    WbBackground.instance = background;

    World.instance.nodes.set(background.id, background);

    return background;
  }

  async checkUse(node, currentNode) {
    let use = getNodeAttribute(node, 'USE');
    if(typeof use === 'undefined')
      return;

    let id = getNodeAttribute(node, 'id');
    let result = World.instance.nodes.get(use);

    if(typeof result === 'undefined'){
      use = 'n' + use
      result = World.instance.nodes.get(use);
    }

    if(typeof result === 'undefined')
      return;

    let useNode = await result.clone(id);
    if (typeof currentNode !== 'undefined'){
      useNode.parent = currentNode.id;
      if (result instanceof WbShape || result instanceof WbGroup || result instanceof WbLight)
        currentNode.children.push(useNode);
    }

    World.instance.nodes.set(id, useNode);
    return useNode;
  }


  async parseTransform(node, currentNode, isBoundingObject){
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if(typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;
    let isSolid = getNodeAttribute(node, 'solid', 'false').toLowerCase() === 'true';
    let translation = convertStringToVec3(getNodeAttribute(node, 'translation', '0 0 0'));
    let scale = convertStringToVec3(getNodeAttribute(node, 'scale', '1 1 1'));
    let rotation = convertStringToQuaternion(getNodeAttribute(node, 'rotation', '0 1 0 0'));

    let transform = new WbTransform(id, isSolid, translation, scale, rotation);

    World.instance.nodes.set(transform.id, transform);

    await this.parseChildren(node, transform, isBoundingObject);

    if(typeof currentNode !== 'undefined'){
      transform.parent = currentNode.id;
      currentNode.children.push(transform);
    }

    return transform;
  }

  async parseGroup(node, currentNode, isBoundingObject) {
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if(typeof id === 'undefined')
      id = "n" + Parser.undefinedID++;

    let isPropeller = getNodeAttribute(node, 'isPropeller', 'false').toLowerCase() === 'true';

    let group = new WbGroup(id, isPropeller);

    World.instance.nodes.set(group.id, group);
    await this.parseChildren(node, group, isBoundingObject);

    if(typeof currentNode !== 'undefined'){
      group.parent = currentNode.id;
      currentNode.children.push(group);
    }

    return group;
  }

  async parseShape(node, currentNode, isBoundingObject) {
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if(typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

    let castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';
    let isPickable = getNodeAttribute(node, 'isPickable', 'true').toLowerCase() === 'true';
    let geometry;
    let appearance;

    for (let i = 0; i < node.childNodes.length; i++) {
      let child = node.childNodes[i];
      if (typeof child.tagName === 'undefined')
        continue;

      if (typeof appearance === 'undefined') {
        if (child.tagName === 'Appearance') {
          if(isBoundingObject)
            continue;
          // If a sibling PBRAppearance is detected, prefer it.
          let pbrAppearanceChild = false;
          for (let j = 0; j < node.childNodes.length; j++) {
            let child0 = node.childNodes[j];
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

    if(isBoundingObject)
      appearance = undefined;

    let shape = new WbShape(id, castShadows, isPickable, geometry, appearance);

    if(typeof currentNode !== 'undefined') {
      //console.log(currentNode);
      currentNode.children.push(shape);
      shape.parent = currentNode.id;
    }

    if(typeof appearance !== 'undefined') {
      appearance.parent = shape.id;
    }

    World.instance.nodes.set(shape.id, shape);

    return shape;
  }

  async parseDirectionalLight(node, currentNode) {
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    let on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    let color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    let direction = convertStringToVec3(getNodeAttribute(node, 'direction', '0 0 -1'));
    let intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    let ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    let castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    let dirLight = new WbDirectionalLight(id, on, color, direction, intensity, castShadows, ambientIntensity);

    if(typeof currentNode !== 'undefined' && typeof dirLight !== 'undefined' ) {
      currentNode.children.push(dirLight);
      dirLight.parent = currentNode.id;
    }

    World.instance.nodes.set(dirLight.id, dirLight);

    return dirLight;
  }

  async parsePointLight(node, currentNode) {
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    let on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    let attenuation = convertStringToVec3(getNodeAttribute(node, 'attenuation', '1 0 0'));
    let color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    let intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    let location = convertStringToVec3(getNodeAttribute(node, 'location', '0 0 0'));
    let radius = parseFloat(getNodeAttribute(node, 'radius', '100'));
    let ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    let castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    let pointLight = new WbPointLight(id, on, attenuation, color, intensity, location, radius, ambientIntensity, castShadows, currentNode);

    if(typeof currentNode !== 'undefined' && typeof pointLight !== 'undefined') {
      currentNode.children.push(pointLight);
    }

    World.instance.nodes.set(pointLight.id, pointLight);

    return pointLight;

  }

  async parseSpotLight(node, currentNode) {
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    let on = getNodeAttribute(node, 'on', 'true').toLowerCase() === 'true';
    let attenuation = convertStringToVec3(getNodeAttribute(node, 'attenuation', '1 0 0'));
    let beamWidth = parseFloat(getNodeAttribute(node, 'beamWidth', '0.785'));
    let color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    let cutOffAngle = parseFloat(getNodeAttribute(node, 'cutOffAngle', '0.785'));
    let direction = convertStringToVec3(getNodeAttribute(node, 'direction', '0 0 -1'));
    let intensity = parseFloat(getNodeAttribute(node, 'intensity', '1'));
    let location = convertStringToVec3(getNodeAttribute(node, 'location', '0 0 0'));
    let radius = parseFloat(getNodeAttribute(node, 'radius', '100'));
    let ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0'));
    let castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    let spotLight = new WbSpotLight(id, on, attenuation, beamWidth, color, cutOffAngle, direction, intensity, location, radius, ambientIntensity, castShadows, currentNode);

    if(typeof currentNode !== 'undefined' && typeof spotLight !== 'undefined') {
      currentNode.children.push(spotLight);
    }

    World.instance.nodes.set(spotLight.id, spotLight);

    return spotLight;
  }

  async parseFog(node) {
    let id = getNodeAttribute(node, 'id');
    let color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    let visibilityRange = parseFloat(getNodeAttribute(node, 'visibilityRange', '0'));
    let fogType = getNodeAttribute(node, 'fogType', 'LINEAR');

    let fog = new WbFog(id, color, visibilityRange, fogType);

    World.instance.nodes.set(fog.id, fog);

    if(typeof fog !== 'undefined')
      World.instance.hasFog = true;

    return fog;
  }

  async parseGeometry(node, parentId) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined') {
      use.parent = parentId;
      return use;
    }

    let id = getNodeAttribute(node, 'id');
    if(typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

    let geometry;
    if(node.tagName === 'Box')
      geometry = this.parseBox(node, id);
    else if (node.tagName === 'Sphere')
      geometry = this.parseSphere(node, id);
    else if (node.tagName === 'Cone')
      geometry = this.parseCone(node, id);
    else if (node.tagName === 'Plane')
      geometry = this.parsePlane(node, id);
    else if (node.tagName === 'Cylinder')
      geometry = this.parseCylinder(node, id);
    else if (node.tagName === 'IndexedFaceSet')
      geometry = this.parseIndexedFaceSet(node, id);
    else if (node.tagName === 'IndexedLineSet')
      geometry = this.parseIndexedLineSet(node, id);
    else if (node.tagName === 'ElevationGrid')
      geometry = this.parseElevationGrid(node, id);
    else if (node.tagName === 'PointSet')
      geometry = this.parsePointSet(node, id);
    else {
      console.log("Not a recognized geometry : " + node.tagName);
      geometry = undefined
    }

    if(typeof parentId !== 'undefined' && typeof geometry !== 'undefined') {
      geometry.parent = parentId;
    }

    return geometry;
  }

  parseBox(node, id) {
    let size = convertStringToVec3(getNodeAttribute(node, 'size', '2 2 2'));

    let box = new WbBox(id, size);
    World.instance.nodes.set(box.id, box);
    return box;
  }

  parseSphere(node, id) {
    let radius = parseFloat(getNodeAttribute(node, 'radius', '1'));
    let ico = getNodeAttribute(node, 'ico', 'false').toLowerCase() === 'true'
    let subdivision = parseInt(getNodeAttribute(node, 'subdivision', '1,1'));

    let sphere = new WbSphere(id, radius, ico, subdivision);

    World.instance.nodes.set(sphere.id, sphere);

    return sphere;
  }

  parseCone(node, id) {
    let bottomRadius = getNodeAttribute(node, 'bottomRadius', '1');
    let height = getNodeAttribute(node, 'height', '2');
    let subdivision = getNodeAttribute(node, 'subdivision', '32');
    let side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    let bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';

    let cone = new WbCone(id, bottomRadius, height, subdivision, side, bottom);

    World.instance.nodes.set(cone.id, cone);

    return cone;
  }

  parseCylinder(node, id) {
    let radius = getNodeAttribute(node, 'radius', '1');
    let height = getNodeAttribute(node, 'height', '2');
    let subdivision = getNodeAttribute(node, 'subdivision', '32');
    let bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';
    let side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    let top = getNodeAttribute(node, 'top', 'true').toLowerCase() === 'true';

    let cylinder = new WbCylinder(id, radius, height, subdivision, bottom, side, top);

    World.instance.nodes.set(cylinder.id, cylinder);

    return cylinder;
  }

  parsePlane(node, id) {
    let size = convertStringToVec2(getNodeAttribute(node, 'size', '1,1'));

    let plane = new WbPlane(id, size);

    World.instance.nodes.set(plane.id, plane);

    return plane;
  }

  parseIndexedFaceSet(node, id) {
    let isDefaultMapping = getNodeAttribute(node, 'defaultMapping', 'false').toLowerCase() === 'true';

    let coordIndexStr = getNodeAttribute(node, 'coordIndex', '').trim().split(/\s/);;
    let coordIndex = coordIndexStr.map(Number);

    let normalIndexStr = getNodeAttribute(node, 'normalIndex', '').trim().split(/\s/);
    let normalIndex = normalIndexStr.map(Number);

    let texCoordIndexStr = getNodeAttribute(node, 'texCoordIndex', '').trim().split(/\s/);;
    let texCoordIndex = texCoordIndexStr.map(Number);

    let coordArray = [];
    let coordinate = node.getElementsByTagName('Coordinate')[0];
    if (typeof coordinate !== 'undefined') {
      let coordStr = getNodeAttribute(coordinate, 'point', '').split(/\s/);
      let coord = coordStr.map(el => parseFloat(el));
      for(let i = 0; i < coord.length; i = i + 3) {
        coordArray.push(new WbVector3(coord[i], coord[i + 1], coord[i + 2]));
      }
    }

    let texCoordArray = [];
    let textureCoordinate = node.getElementsByTagName('TextureCoordinate')[0];
    if (typeof textureCoordinate !== 'undefined') {
      let texcoordsStr = getNodeAttribute(textureCoordinate, 'point', '').split(/\s/);
      let texCoord  = texcoordsStr.map(el => parseFloat(el));
      for(let i = 0; i < texCoord.length; i = i + 2) {
        texCoordArray.push(new WbVector2(texCoord[i], texCoord[i + 1]));
      }
    }

    let normalArray = [];
    let normalNode = node.getElementsByTagName('Normal')[0];
    if (typeof normalNode !== 'undefined') {
      let normalStr = getNodeAttribute(normalNode, 'vector', '').split(/[\s,]+/);
      let normal = normalStr.map(el => parseFloat(el));
      for(let i = 0; i < normal.length; i = i + 3) {
        normalArray.push(new WbVector3(normal[i], normal[i + 1], normal[i + 2]));
      }
    }

    let creaseAngle = parseFloat(getNodeAttribute(node, 'creaseAngle', '0'));
    let ccw = parseFloat(getNodeAttribute(node, 'ccw', '1'));
    let normalPerVertex = parseFloat(getNodeAttribute(node, 'normalPerVertex', '1'));

    let ifs = new WbIndexedFaceSet(id, isDefaultMapping, coordIndex, normalIndex, texCoordIndex, coordArray, texCoordArray, normalArray, creaseAngle, ccw, normalPerVertex);
    World.instance.nodes.set(ifs.id, ifs);

    return ifs;
  }

  parseIndexedLineSet(node, id) {
    let coordinate = node.getElementsByTagName('Coordinate')[0];

    if (typeof coordinate === 'undefined')
      return undefined;

    let indicesStr = getNodeAttribute(node, 'coordIndex', '').trim().split(/\s/);

    let verticesStr = getNodeAttribute(coordinate, 'point', '').trim().split(/\s/);

    let coord = [];
    for (let i = 0; i < verticesStr.length; i += 3) {
      coord.push(new WbVector3(parseFloat(verticesStr[i]), parseFloat(verticesStr[i + 1]), parseFloat(verticesStr[i + 2])));
    }

    let coordIndex = indicesStr.map(Number);

    let ils = new WbIndexedLineSet(id, coord, coordIndex);
    World.instance.nodes.set(ils.id, ils);

    return ils;
  }

  parseElevationGrid(node, id) {
    let heightStr = getNodeAttribute(node, 'height', undefined);
    if(typeof heightStr === 'undefined')
      return;

    let xDimension = parseInt(getNodeAttribute(node, 'xDimension', '0'));
    let xSpacing = parseFloat(getNodeAttribute(node, 'xSpacing', '1'));
    let zDimension = parseInt(getNodeAttribute(node, 'zDimension', '0'));
    let zSpacing = parseFloat(getNodeAttribute(node, 'zSpacing', '1'));
    let thickness = parseFloat(getNodeAttribute(node, 'thickness', '1'));

    let height = heightStr.split(' ').map(Number);


    let eg = new WbElevationGrid(id, height, xDimension, xSpacing, zDimension, zSpacing, thickness);
    World.instance.nodes.set(eg.id, eg);

    return eg;
  }

  parsePointSet(node, id) {
    let coordinate = node.getElementsByTagName('Coordinate')[0];

    if(typeof coordinate === 'undefined')
      return;

    let coordStrArray = getNodeAttribute(coordinate, 'point', '').trim().split(/\s/);

    if(typeof coordStrArray === 'undefined')
      return;

    let coordArray = coordStrArray.map(Number);
    let coord = [];
    for(let i = 0; i < coordArray.length; i+=3 ){
      coord.push(new WbVector3(coordArray[i], coordArray[i + 1], coordArray[i + 2]))
    }

    let colorNode = node.getElementsByTagName('Color')[0];
    let color = undefined;
    if (typeof colorNode !== 'undefined') {
      let colorStrArray = getNodeAttribute(colorNode, 'color', '').trim().split(/\s/);
        if (typeof colorStrArray !== 'undefined'){
          let colorArray = colorStrArray.map(Number);
          color = [];
          for(let i = 0; i < colorArray.length; i+=3 ){
            color.push(new WbVector3(colorArray[i], colorArray[i + 1], colorArray[i + 2]))
          }
        }
    }


    let ps = new WbPointSet(id, coord, color);
    World.instance.nodes.set(ps.id, ps);

    return ps;
  }

  async parseSwitch(node, parent) {
    if(typeof parent === 'undefined')
      return;

    let child = node.childNodes[0];

    let boundingObject = undefined
    if (child.tagName === 'Shape')
      boundingObject = await this.parseShape(child, undefined, true);
    else if (child.tagName === 'Transform')
      boundingObject = await this.parseTransform(child, undefined, true);
    else if (child.tagName === 'Group')
      boundingObject = await this.parseGroup(child, undefined, true)
    else {
      console.error("Unknown boundingObject: " + child.tagName);
    }

    if (typeof boundingObject === 'undefined')
      return

    boundingObject.parent = parent.id;
    parent.boundingObject = boundingObject;

    return boundingObject;
  }

  async parseAppearance(node, parentId) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if(typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

    // Get the Material tag.
    let materialNode = node.getElementsByTagName('Material')[0];
    let material;
    if (typeof materialNode !== 'undefined')
      material = await this.parseMaterial(materialNode)

    // Check to see if there is a texture.
    let imageTexture = node.getElementsByTagName('ImageTexture')[0];
    let texture;
    if (typeof imageTexture !== 'undefined'){
        texture = await this.parseImageTexture(imageTexture);
    }

    // Check to see if there is a textureTransform.
    let textureTransform = node.getElementsByTagName('TextureTransform')[0];
    let transform;
    if (typeof textureTransform !== 'undefined'){
        transform = await this.parseTextureTransform(textureTransform);
    }

    let appearance = new WbAppearance(id, material, texture, transform);
    if (typeof appearance !== 'undefined') {
        if(typeof material !== 'undefined')
          material.parent = appearance.id;
    }

    if (typeof appearance !== 'undefined') {
        if(typeof texture !== 'undefined')
          texture.parent = appearance.id;
    }

    if (typeof appearance !== 'undefined') {
        if(typeof transform !== 'undefined')
          transform.parent = appearance.id;
    }

    if(typeof parentId !== 'undefined')
      appearance.parent = parentId;

    World.instance.nodes.set(appearance.id, appearance);

    return appearance;
  }

  async parseMaterial(node, parentId) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    if(typeof id === 'undefined')
      id = 'n' + Parser.undefinedID++;

    let ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0.2')),
    diffuseColor = convertStringToVec3(getNodeAttribute(node, 'diffuseColor', '0.8 0.8 0.8')),
    specularColor = convertStringToVec3(getNodeAttribute(node, 'specularColor', '0 0 0')),
    emissiveColor = convertStringToVec3(getNodeAttribute(node, 'emissiveColor', '0 0 0')),
    shininess = parseFloat(getNodeAttribute(node, 'shininess', '0.2')),
    transparency = parseFloat(getNodeAttribute(node, 'transparency', '0'));

    let material = new WbMaterial(id, ambientIntensity, diffuseColor, specularColor, emissiveColor, shininess, transparency);

    if(typeof parentId !== 'undefined')
      material.parent = parentId;

    World.instance.nodes.set(material.id, material);

    return material;
  }

  async parseImageTexture(node, parentId) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');
    let url = getNodeAttribute(node, 'url', '');
    url = url.slice(1, url.length-1);
    const isTransparent = getNodeAttribute(node, 'isTransparent', 'false').toLowerCase() === 'true';
    const s = getNodeAttribute(node, 'repeatS', 'true').toLowerCase() === 'true';
    const t = getNodeAttribute(node, 'repeatT', 'true').toLowerCase() === 'true';
    const filtering = parseFloat(getNodeAttribute(node, 'filtering', '4'));

    const textureProperties = node.getElementsByTagName('TextureProperties')[0];

    let imageTexture = undefined;

    if(typeof url !== 'undefined' && url !== '') {
      url = this.prefix + url;
      imageTexture = new WbImageTexture(id, url, isTransparent, s, t, filtering);
      await imageTexture.updateUrl();
    }


    if(typeof imageTexture !== 'undefined'){
      if(typeof parentId !== 'undefined')
        imageTexture.parent = parentId;

      World.instance.nodes.set(imageTexture.id, imageTexture);
    }

    return imageTexture;
  }

  async parsePBRAppearance(node, parentId) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');

    let baseColor = convertStringToVec3(getNodeAttribute(node, 'baseColor', '1 1 1'));
    let transparency = parseFloat(getNodeAttribute(node, 'transparency', '0'));
    let roughness = parseFloat(getNodeAttribute(node, 'roughness', '0'))
    let metalness = parseFloat(getNodeAttribute(node, 'metalness', '1'));
    let IBLStrength = parseFloat(getNodeAttribute(node, 'IBLStrength', '1'));
    let normalMapFactor = parseFloat(getNodeAttribute(node, 'normalMapFactor', '1'));
    let occlusionMapStrength = parseFloat(getNodeAttribute(node, 'occlusionMapStrength', '1'));
    let emissiveColor = convertStringToVec3(getNodeAttribute(node, 'emissiveColor', '0 0 0'));
    let emissiveIntensity = parseFloat(getNodeAttribute(node, 'emissiveIntensity', '1'));

    // Check to see if there is a textureTransform.
    let textureTransform = node.getElementsByTagName('TextureTransform')[0];
    let transform;
    if (typeof textureTransform !== 'undefined'){
        transform = await this.parseTextureTransform(textureTransform);
    }

    let imageTextures = node.getElementsByTagName('ImageTexture');
    let baseColorMap = undefined;
    let roughnessMap = undefined;
    let metalnessMap = undefined;
    let normalMap = undefined;
    let occlusionMap = undefined;
    let emissiveColorMap = undefined;
    for (let i = 0; i < imageTextures.length; i++) {
      let imageTexture = imageTextures[i];
      let type = getNodeAttribute(imageTexture, 'type', undefined);
      if (type === 'baseColor'){
        baseColorMap = await this.parseImageTexture(imageTexture);
        if (typeof baseColorMap !== 'undefined')
          baseColorMap.type = "baseColorMap";
      } else if (type === 'roughness'){
        roughnessMap = await this.parseImageTexture(imageTexture);
        if (typeof roughnessMap !== 'undefined')
          roughnessMap.type = "roughnessMap";
      } else if (type === 'metalness'){
        metalnessMap = await this.parseImageTexture(imageTexture);
        if (typeof metalnessMap !== 'undefined')
          metalnessMap.type = "metalnessMap";
      } else if (type === 'normal'){
        normalMap = await this.parseImageTexture(imageTexture);
        if (typeof normalMap !== 'undefined')
          normalMap.type = "normalMap";
      } else if (type === 'occlusion') {
        occlusionMap = await this.parseImageTexture(imageTexture);
        if (typeof occlusionMap !== 'undefined')
          occlusionMap.type = "occlusionMap";
      } else if (type === 'emissiveColor'){
        emissiveColorMap = await this.parseImageTexture(imageTexture);
        if (typeof emissiveColorMap !== 'undefined')
          emissiveColorMap.type = "emissiveColorMap";
      }
    }

    let pbrAppearance = new WbPBRAppearance(id, baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness, metalnessMap, IBLStrength,
       normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor, emissiveColorMap, emissiveIntensity, transform);

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof transform !== 'undefined')
          transform.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof baseColorMap !== 'undefined')
          baseColorMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof roughnessMap !== 'undefined')
          roughnessMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof metalnessMap !== 'undefined')
          metalnessMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof normalMap !== 'undefined')
          normalMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof occlusionMap !== 'undefined')
          occlusionMap.parent = pbrAppearance.id;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof emissiveColorMap !== 'undefined')
          emissiveColorMap.parent = pbrAppearance.id;
    }

    if(typeof parentId !== 'undefined')
      pbrAppearance.parent = parentId;

    World.instance.nodes.set(pbrAppearance.id, pbrAppearance);

    return pbrAppearance;
  }

  async parseTextureTransform(node, parentId) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');
    let center = convertStringToVec2(getNodeAttribute(node, 'center', '0 0')),
    rotation = parseFloat(getNodeAttribute(node, 'rotation', '0')),
    scale = convertStringToVec2(getNodeAttribute(node, 'scale', '1 1')),
    translation = convertStringToVec2(getNodeAttribute(node, 'translation', '0 0'))

    let textureTransform = new WbTextureTransform(id, center, rotation, scale, translation);

    if(typeof parentId !== 'undefined')
      textureTransform.parent = parentId;

    World.instance.nodes.set(textureTransform.id, textureTransform);

    return textureTransform;
  }

  static async loadTextureData(url, isHdr) {
    let canvas2 = document.createElement('canvas')
    let context =  canvas2.getContext('2d');

    let image = new WbImage();

    if(isHdr){
      let img = await Parser.loadHDRImage(url);
      image.bits = img.data;
      image.width = img.width;
      image.height = img.height;
      image.url = url;
    }
    else {
      let img = await Parser.loadImage(url);
      canvas2.width = img.width;
      canvas2.height = img.height;
      context.drawImage(img, 0, 0);
      let dataBGRA = context.getImageData(0, 0, img.width, img.height).data;
      let data = new Uint8ClampedArray(dataBGRA.length);
      data = dataBGRA;

      image.bits = data
      image.width = img.width;
      image.height = img.height;
      image.url = url;
    }
    return image;
 }

 static loadImage(src){
   return new Promise((resolve, reject) => {
     let img = new Image();
     img.onload = () => {
       resolve(img);
     }
     img.onerror = () => console.log("Error in loading : " + src);
     img.setAttribute('crossOrigin', '');
     img.src = src;
   })
 }

 static loadHDRImage(src){
   return new Promise((resolve, reject) => {
     let loader = new RGBELoader;
     loader.load(src, function(img){resolve(img)});
   })
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
  let v = new WbVector2(parseFloat(s[0]), parseFloat(s[1]));
  return v;
}

function convertStringToVec3(s) {
  s = s.split(/\s/);
  let v = new WbVector3(parseFloat(s[0]), parseFloat(s[1]), parseFloat(s[2]));
  return v;
}

function convertStringToQuaternion(s) {
  let pos = s.split(/\s/);
  let q = new WbVector4(parseFloat(pos[0]), parseFloat(pos[1]), parseFloat(pos[2]), parseFloat(pos[3]));
  return q;
}

Parser.undefinedID = 90000;
export {Parser, convertStringToVec3, convertStringToQuaternion}
