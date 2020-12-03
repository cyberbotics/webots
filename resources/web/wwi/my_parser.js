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
import {WbIndexedFaceSet} from "./webotsjs/WbIndexedFaceSet.js"

import {WbMaterial} from "./webotsjs/WbMaterial.js";
import {WbTextureTransform} from "./webotsjs/WbTextureTransform.js";
import {WbAppearance} from "./webotsjs/WbAppearance.js";
import {WbPBRAppearance} from "./webotsjs/WbPBRAppearance.js";
import {WbImageTexture} from "./webotsjs/WbImageTexture.js";
import {WbImage} from "./webotsjs/WbImage.js";

import {WbDirectionalLight} from "./webotsjs/WbDirectionalLight.js";
import {WbPointLight} from "./webotsjs/WbPointLight.js";
import {WbSpotLight} from "./webotsjs/WbSpotLight.js";
import {WbFog} from "./webotsjs/WbFog.js"

import {Use} from "./webotsjs/Use.js";


class MyParser {
  constructor() {
      this.prefix = "http://localhost:1234/";
      this.irradiancePrefix = "/projects/default/worlds/"
      let world = new World();
      this.fog = false;
  }

  parse(text){
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
          this.parseChildren(node);
      } else {
        console.log(scene);
        this.parseNode(scene);
      }
    }
  }

  async parseNode(node, currentNode) {
    let result;
    if(node.tagName === 'Scene') {
      await this.parseScene(node);
      await this.parseChildren(node, currentNode)
      console.log(World.instance);
      World.instance.viewpoint.finalize();
      World.instance.sceneTree.forEach(node => {
        node.finalize();});
    } else if (node.tagName === 'WorldInfo')
      this.parseWorldInfo(node);
    else if (node.tagName === 'Viewpoint')
      World.instance.viewpoint = this.parseViewpoint(node);
    else if (node.tagName === 'Background')
      result = await this.parseBackground(node);
    else if (node.tagName === 'Transform')
      result = await this.parseTransform(node, currentNode);
    else if (node.tagName === 'Group')
      result = await this.parseGroup(node, currentNode);
    else if (node.tagName === 'Shape')
      result = await this.parseShape(node, currentNode);
    else if (node.tagName === 'Switch')
      result = undefined;
    else if (node.tagName === 'DirectionalLight')
      result = await this.parseDirectionalLight(node, currentNode);
    else if (node.tagName === 'PointLight')
      result = await this.parsePointLight(node, currentNode);
    else if (node.tagName === 'SpotLight')
      result = await this.parseSpotLight(node, currentNode);
    else if (node.tagName === 'Fog' && !this.fog)
      result = await this.parseFog(node);
    else {
      console.log(node.tagName);
      console.error("The parser doesn't support this type of node");
    }

    //check if top-level nodes
    if(typeof result !== 'undefined' && typeof currentNode === 'undefined')
        World.instance.sceneTree.push(result);

    return result;
  }

  async parseChildren(node, currentNode) {
    for (let i = 0; i < node.childNodes.length; i++) {
      let child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined'){
        await this.parseNode(child, currentNode);
      }
    }
    return 1;
  }

  async parseScene(node) {
    let id = getNodeAttribute(node, 'id');
    let lensFlareLenTexture = await this.loadTextureData("/resources/wren/textures/lens_flare.png", true);
    lensFlareLenTexture.isTranslucent = true;
    let smaaAreaTexture = await this.loadTextureData("/resources/wren/textures/smaa_area_texture.png", true);
    smaaAreaTexture.isTranslucent = false;
    let smaaSearchTexture = await this.loadTextureData("/resources/wren/textures/smaa_search_texture.png", true);
    smaaSearchTexture.isTranslucent = false;
    let gtaoNoiseTexture = await this.loadTextureData("/resources/wren/textures/gtao_noise_texture.png", true);
    gtaoNoiseTexture.isTranslucent = true;
    return new WbScene(id, lensFlareLenTexture, smaaAreaTexture, smaaSearchTexture, gtaoNoiseTexture);
  }

  parseWorldInfo(node){
    console.log("Un WorldInfo");
  }

  parseViewpoint(node){
    let id = getNodeAttribute(node, 'id');
    let orientation = convertStringToQuaternion(getNodeAttribute(node, 'orientation', '0 1 0 0'));
    let position = convertStringToVec3(getNodeAttribute(node, 'position', '0 0 10'));
    let exposure = parseFloat(getNodeAttribute(node, 'exposure', '1.0'));
    let bloomThreshold = parseFloat(getNodeAttribute(node, 'bloomThreshold'));
    let far = parseFloat(getNodeAttribute(node, 'zFar', '2000'));
    let zNear = parseFloat(getNodeAttribute(node, 'zNear', '0.1'));
    let followsmoothness = parseFloat(getNodeAttribute(node, 'followsmoothness'));

    let viewpoint = new WbViewpoint(id, orientation, position, exposure, bloomThreshold, zNear, far, followsmoothness);

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


      cubeImages[5] = await this.loadTextureData(this.prefix + backUrl);
      cubeImages[3] = await this.loadTextureData(this.prefix + bottomUrl);
      cubeImages[4] = await this.loadTextureData(this.prefix + frontUrl);
      cubeImages[1] = await this.loadTextureData(this.prefix + leftUrl);
      cubeImages[0] = await this.loadTextureData(this.prefix + rightUrl);
      cubeImages[2] = await this.loadTextureData(this.prefix + topUrl);
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

      irradianceCubeURL[5] = this.irradiancePrefix + backIrradianceUrl;
      irradianceCubeURL[3] = this.irradiancePrefix + bottomIrradianceUrl;
      irradianceCubeURL[4] = this.irradiancePrefix + frontIrradianceUrl;
      irradianceCubeURL[1] = this.irradiancePrefix + leftIrradianceUrl;
      irradianceCubeURL[0] = this.irradiancePrefix + rightIrradianceUrl;
      irradianceCubeURL[2] = this.irradiancePrefix + topIrradianceUrl;
    } else {
      console.log("Background : Incomplete irradiance cubemap");
    }

    let background = new WbBackground(id, skyColor, luminosity, cubeImages, irradianceCubeURL);
    WbBackground.instance = background;

    World.instance.nodes[background.id] = background;

    return background;
  }

  async checkUse(node, currentNode) {;
    let use = getNodeAttribute(node, 'USE');
    if(typeof use === 'undefined')
      return;

    let id = getNodeAttribute(node, 'id');
    let result = World.instance.nodes[use];
    if(typeof result === 'undefined'){
      use = 'n' + use
      result = World.instance.nodes[use];
    }

    if(typeof result === 'undefined')
      return;

    let useNode = new Use(id, result);

    if (typeof currentNode !== 'undefined'){
      useNode.parent = currentNode.id;
      if (useNode.def instanceof WbShape || useNode.def instanceof WbTransform)
        currentNode.children.push(useNode);
    }

    if(typeof World.instance.defUse[use] === 'undefined')
      World.instance.defUse[use] = new Array();

    World.instance.nodes[id] = useNode;
    World.instance.defUse[use].push(id);

    return useNode;
  }


  async parseTransform(node, currentNode){
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    let isSolid = getNodeAttribute(node, 'solid', 'false').toLowerCase() === 'true';
    let translation = convertStringToVec3(getNodeAttribute(node, 'translation', '0 0 0'));
    let scale = convertStringToVec3(getNodeAttribute(node, 'scale', '1 1 1'));
    let rotation = convertStringToQuaternion(getNodeAttribute(node, 'rotation', '0 1 0 0'));

    let transform = new WbTransform(id, isSolid, translation, scale, rotation);

    World.instance.nodes[transform.id] = transform;

    await this.parseChildren(node, transform);

    if(typeof currentNode !== 'undefined'){
      transform.parent = currentNode.id;
      currentNode.children.push(transform);
    }

    return transform;
  }

  async parseGroup(node, currentNode) {
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    let group = new WbGroup(id);

    World.instance.nodes[group.id] = group;
    await this.parseChildren(node, group);

    if(typeof currentNode !== 'undefined'){
      group.parent = currentNode.id;
      currentNode.children.push(group);
    }

    return group;
  }

  async parseShape(node, currentNode) {
    let use = await this.checkUse(node, currentNode);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    let castShadows = getNodeAttribute(node, 'castShadows', 'false').toLowerCase() === 'true';

    let geometry;
    let appearance;

    for (let i = 0; i < node.childNodes.length; i++) {
      let child = node.childNodes[i];
      if (typeof child.tagName === 'undefined')
        continue;

      if (typeof appearance === 'undefined') {
        if (child.tagName === 'Appearance') {
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

    let shape = new WbShape(id, castShadows, geometry, appearance);

    if(typeof currentNode !== 'undefined') {
      currentNode.children.push(shape);
      shape.parent = currentNode.id;
    }

    if(typeof appearance !== 'undefined') {
      appearance.parent = shape.id;
    }

    World.instance.nodes[shape.id] = shape;

    return shape;
  }

  async parseDirectionalLight(node, currentNode) {
    //TODO USE
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

    World.instance.nodes[dirLight.id] = dirLight;

    return dirLight;
  }

  async parsePointLight(node, currentNode) {
    //TODO USE
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

    World.instance.nodes[pointLight.id] = pointLight;

    return pointLight;

  }

  async parseSpotLight(node, currentNode) {
    //TODO USE
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

    World.instance.nodes[spotLight.id] = spotLight;

    return spotLight;
  }

  async parseFog(node) {
    let id = getNodeAttribute(node, 'id');
    let color = convertStringToVec3(getNodeAttribute(node, 'color', '1 1 1'));
    let visibilityRange = parseFloat(getNodeAttribute(node, 'visibilityRange', '0'));
    let fogType = getNodeAttribute(node, 'fogType', 'LINEAR');

    let fog = new WbFog(id, color, visibilityRange, fogType);

    World.instance.nodes[fog.id] = fog;

    if(typeof fog !== 'undefined')
      this.fog = true;

    return fog;
  }

  async parseGeometry(node, parentId) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined') {
      use.parent = parentId;
      return use;
    }

    let geometry;
    if(node.tagName === 'Box')
      geometry = this.parseBox(node);
    else if (node.tagName === 'Sphere')
      geometry = this.parseSphere(node);
    else if (node.tagName === 'Cone')
      geometry = this.parseCone(node);
    else if (node.tagName === 'Plane')
      geometry = this.parsePlane(node);
    else if (node.tagName === 'Cylinder')
      geometry = this.parseCylinder(node);
    else if (node.tagName === 'IndexedFaceSet')
      geometry = this.parseIndexedFaceSet(node);
    else {
      console.log("Not a recognized geometry : " + node.tagName);
      geometry = undefined
    }

    if(typeof parentId !== 'undefined' && typeof geometry !== 'undefined') {
      geometry.parent = parentId;
    }

    return geometry;
  }

  parseBox(node) {
    let id = getNodeAttribute(node, 'id');
    let size = convertStringToVec3(getNodeAttribute(node, 'size', '2 2 2'));

    let box = new WbBox(id, size);

    World.instance.nodes[box.id] = box;
    return box;
  }

  parseSphere(node) {
    let id = getNodeAttribute(node, 'id');
    let radius = parseFloat(getNodeAttribute(node, 'radius', '1'));
    let ico = getNodeAttribute(node, 'ico', 'false').toLowerCase() === 'true'
    let subdivision = parseInt(getNodeAttribute(node, 'subdivision', '1,1'));

    let sphere = new WbSphere(id, radius, ico, subdivision);

    World.instance.nodes[sphere.id] = sphere;

    return sphere;
  }

  parseCone(node) {
    let id = getNodeAttribute(node, 'id');
    let bottomRadius = getNodeAttribute(node, 'bottomRadius', '1');
    let height = getNodeAttribute(node, 'height', '2');
    let subdivision = getNodeAttribute(node, 'subdivision', '32');
    let side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    let bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';

    let cone = new WbCone(id, bottomRadius, height, subdivision, side, bottom);

    World.instance.nodes[cone.id] = cone;

    return cone;
  }

  parseCylinder(node) {
    let id = getNodeAttribute(node, 'id');
    let radius = getNodeAttribute(node, 'radius', '1');
    let height = getNodeAttribute(node, 'height', '2');
    let subdivision = getNodeAttribute(node, 'subdivision', '32');
    let bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';
    let side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    let top = getNodeAttribute(node, 'top', 'true').toLowerCase() === 'true';

    let cylinder = new WbCylinder(id, radius, height, subdivision, bottom, side, top);

    World.instance.nodes[cylinder.id] = cylinder;

    return cylinder;
  }

  parsePlane(node) {
    let id = getNodeAttribute(node, 'id');
    let size = convertStringToVec2(getNodeAttribute(node, 'size', '1,1'));

    let plane = new WbPlane(id, size);

    World.instance.nodes[plane.id] = plane;

    return plane;
  }

  parseIndexedFaceSet(node) {
    let id = getNodeAttribute(node, 'id');
    let isDefaultMapping = getNodeAttribute(node, 'defaultMapping', 'false').toLowerCase() === 'true';

    let coordIndexStr = getNodeAttribute(node, 'coordIndex', '').split(/\s/);
    let coordIndex = coordIndexStr.map(Number);

    let normalIndexStr = getNodeAttribute(node, 'normalIndex', '').split(/\s/);
    let normalIndex = normalIndexStr.map(Number);

    let texCoordIndexStr = getNodeAttribute(node, 'texCoordIndex', '').split(/\s/);
    let texCoordIndex = texCoordIndexStr.map(Number);

    let coordinate = node.getElementsByTagName('Coordinate')[0];
    let coordStr = getNodeAttribute(coordinate, 'point', '').split(/\s/);
    let coord = coordStr.map(el => parseFloat(el));

    let textureCoordinate = node.getElementsByTagName('TextureCoordinate')[0];
    let texcoordsStr = getNodeAttribute(textureCoordinate, 'point', '').split(/\s/);
    let texCoord  = texcoordsStr.map(el => parseFloat(el));

    let normalNode = node.getElementsByTagName('Normal')[0];
    let normalStr = getNodeAttribute(normalNode, 'vector', '').split(/[\s,]+/);
    let normal = normalStr.map(el => parseFloat(el));

    let ifs = new IndexedFaceSet(id, isDefaultMapping, coordIndex, normalIndex, texCoordIndex, coord, texCoord, normal);
    World.instance.nodes[ifs.id] = ifs;

    return ifs;
  }

  async parseAppearance(node) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');

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

    World.instance.nodes[appearance.id] = appearance;

    return appearance;
  }

  async parseMaterial(node) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined')
      return use;

    let id = getNodeAttribute(node, 'id');
    let ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0.2')),
    diffuseColor = convertStringToVec3(getNodeAttribute(node, 'diffuseColor', '0.8 0.8 0.8')),
    specularColor = convertStringToVec3(getNodeAttribute(node, 'specularColor', '0 0 0')),
    emissiveColor = convertStringToVec3(getNodeAttribute(node, 'emissiveColor', '0 0 0')),
    shininess = parseFloat(getNodeAttribute(node, 'shininess', '0.2')),
    transparency = parseFloat(getNodeAttribute(node, 'transparency', '0'));

    let material = new WbMaterial(id, ambientIntensity, diffuseColor, specularColor, emissiveColor, shininess, transparency);

    World.instance.nodes[material.id] = material;

    return material;
  }

  async parseImageTexture(node, hasPrefix) {
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
    let anisotropy = 8;
    if (typeof textureProperties !== 'undefined'){
      anisotropy = parseFloat(getNodeAttribute(textureProperties, 'anisotropicDegree', '8'));
    }
    let imageTexture = undefined;

    if(typeof url !== 'undefined' && url !== '') {
      url = this.prefix + url
      let image = await this.loadTextureData(url);
      imageTexture = new WbImageTexture(id, url, isTransparent, s, t, filtering, anisotropy, image);
    }

    if(typeof imageTexture !== 'undefined'){
      World.instance.nodes[imageTexture.id] = imageTexture;
    }

    return imageTexture;
  }

  async parsePBRAppearance(node) {
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
      if (type === 'baseColor')
        baseColorMap = await this.parseImageTexture(imageTexture, true);
      else if (type === 'roughness')
        roughnessMap = await this.parseImageTexture(imageTexture, true);
      else if (type === 'metalness')
        metalnessMap = await this.parseImageTexture(imageTexture, true);
      else if (type === 'normal')
        normalMap = await this.parseImageTexture(imageTexture, true);
      else if (type === 'occlusion')
        occlusionMap = await this.parseImageTexture(imageTexture, true);
      else if (type === 'emissiveColor')
        emissiveColorMap = await this.parseImageTexture(imageTexture, true);
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

    World.instance.nodes[pbrAppearance.id] = pbrAppearance;

    return pbrAppearance;
  }

  async parseTextureTransform(node) {
    let use = await this.checkUse(node);
    if(typeof use !== 'undefined')
      return use;

    const id = getNodeAttribute(node, 'id');
    let center = convertStringToVec2(getNodeAttribute(node, 'center', '0 0')),
    rotation = parseFloat(getNodeAttribute(node, 'rotation', '0')),
    scale = convertStringToVec2(getNodeAttribute(node, 'scale', '1 1')),
    translation = convertStringToVec2(getNodeAttribute(node, 'translation', '0 0'))

    let textureTransform = new WbTextureTransform(id, center, rotation, scale, translation);

    World.instance.nodes[textureTransform.id] = textureTransform;

    return textureTransform;
  }

  async loadTextureData(url, bgra) {
   let context = document.getElementById('canvas2').getContext('2d');
   let img = await this.loadImage(url);
   canvas2.width = img.width;
   canvas2.height = img.height;
   context.drawImage(img, 0, 0);
   let dataBGRA = context.getImageData(0, 0, img.width, img.height).data;
   let data = new Uint8ClampedArray(dataBGRA.length);
   if(bgra){
     for(let x = 0; x < dataBGRA.length; x = x+4){
       data[0+x] = dataBGRA[2+x];
       data[1+x] = dataBGRA[1+x];
       data[2+x] = dataBGRA[0+x];
       data[3+x] = dataBGRA[3+x];
    }
  }else
    data = dataBGRA;


   let image = new WbImage();

   image.bits = data;
   image.width = img.width;
   image.height = img.height;
   image.url = url;
   return image;
 }

 loadImage(src){
   return new Promise((resolve, reject) => {
     let img = new Image();
     img.onload = () => {
       resolve(img);
     }
     img.onerror = () => console.log("Error in loading : " + src);
     img.setAttribute('crossOrigin', ''); //TODO Check if we want to let it
     img.src = src;
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
  let v = new glm.vec2(parseFloat(s[0]), parseFloat(s[1]));
  return v;
}

function convertStringToVec3(s) {
  s = s.split(/\s/);
  let v = new glm.vec3(parseFloat(s[0]), parseFloat(s[1]), parseFloat(s[2]));
  return v;
}

function convertStringToQuaternion(s) {
  let pos = s.split(/\s/);
  let q = new glm.vec4(parseFloat(pos[0]), parseFloat(pos[1]), parseFloat(pos[2]), parseFloat(pos[3]));
  return q;
}

export {MyParser, convertStringToVec3, convertStringToQuaternion}
