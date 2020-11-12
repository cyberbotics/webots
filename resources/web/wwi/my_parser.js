class MyParser {
  constructor() {
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

    let scene = xml.getElementsByTagName('Scene')[0];
    console.log(scene);
    if (typeof scene === 'undefined') {
      console.error("Scene not found");
    } else {
      this.parseNode(scene).then(() => {
        _wr_scene_render(_wr_scene_get_instance(), null, true);
        console.log("File Parsed => render");
      });
    }
  }

  async parseNode(node, currentNode) {
    let result;
    if(node.tagName === 'Scene') {
      let id = getNodeAttribute(node, 'id');
      result = new WbScene(id);
      await this.parseChildren(node, currentNode);
    } else if (node.tagName === 'WorldInfo')
      this.parseWorldInfo(node);
    else if (node.tagName === 'Viewpoint')
      this.parseViewpoint(node);
    else if (node.tagName === 'Background')
      await this.parseBackground(node);
    else if (node.tagName === 'Transform')
      this.parseTransform(node, currentNode);
    else if (node.tagName === 'Shape')
      await this.parseShape(node, currentNode);

    else {
      console.log(node.tagName);
      console.error("The parser doesn't support this type of node");
    }
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
    viewpoint.createWrenObjects();

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

    let cubeTexture = undefined;
    if(typeof backUrl !== 'undefined' && typeof bottomUrl !== 'undefined' && typeof frontUrl !== 'undefined' && typeof leftUrl !== 'undefined' && typeof rightUrl !== 'undefined' && typeof topUrl !== 'undefined'){
      console.log("TODO load");
    } else {
      console.log("Background : Incomplete cubemap");
    }

    let backIrradianceUrl = getNodeAttribute(node, 'backIrradianceUrl');
    let bottomIrradianceUrl = getNodeAttribute(node, 'bottomIrradianceUrl');
    let frontIrradianceUrl = getNodeAttribute(node, 'frontIrradianceUrl');
    let leftIrradianceUrl = getNodeAttribute(node, 'leftIrradianceUrl');
    let rightIrradianceUrl = getNodeAttribute(node, 'rightIrradianceUrl');
    let topIrradianceUrl = getNodeAttribute(node, 'topIrradianceUrl');

    let irradianceCubeTexture = undefined;
    if(typeof backIrradianceUrl !== 'undefined' && typeof bottomIrradianceUrl !== 'undefined' && typeof frontIrradianceUrl !== 'undefined' && typeof leftIrradianceUrl !== 'undefined' && typeof rightIrradianceUrl !== 'undefined' && typeof topIrradianceUrl !== 'undefined'){
      console.log("TODO load irradiance");
    } else {
      console.log("Background : Incomplete irradiance cubemap");
    }

    let background = new WbBackground(id, skyColor, luminosity, cubeTexture, irradianceCubeTexture);
    background.createWrenObjects();
    return background;
  }


  async parseTransform(node, currentNode){
    let id = getNodeAttribute(node, 'id');
    let isSolid = getNodeAttribute(node, 'solid', 'false').toLowerCase() === 'true';
    let translation = convertStringToVec3(getNodeAttribute(node, 'translation', '0 0 0'));
    let scale = convertStringToVec3(getNodeAttribute(node, 'scale', '1 1 1'));
    let rotation = convertStringToQuaternion(getNodeAttribute(node, 'rotation', '0 1 0 0'));

    let transform = new WbTransform(id, isSolid, translation, scale, rotation);

    await this.parseChildren(node, transform);

    transform.createWrenObjects();
  }

  async parseShape(node, currentNode){
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
        geometry = this.parseGeometry(child, currentNode);
        if (typeof geometry !== 'undefined')
          continue;
      }

      console.log('X3dLoader: Unknown node: ' + child.tagName);
    }
    let shape = new WbShape(id, castShadows, geometry, appearance);

    if(typeof currentNode !== 'undefined') {
      currentNode.children.push(shape);
      shape.parent = currentNode;
    } else {
      shape.createWrenObjects();
    }
    return 0;
  }

  parseGeometry(node, currentNode) {
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
    else {
      console.log("Not a recognized geometry : " + node.tagName);
      geometry = undefined
    }

    if(typeof currentNode !== 'undefined' && typeof geometry !== 'undefined') {
      geometry.parent = currentNode;
    }
    return geometry;
  }

  parseBox(node) {
    let id = getNodeAttribute(node, 'id');
    let size = convertStringToVec3(getNodeAttribute(node, 'size', '2 2 2'));

    return new WbBox(id, size);
  }

  parseSphere(node) {
    let id = getNodeAttribute(node, 'id');
    let radius = parseFloat(getNodeAttribute(node, 'radius', '1'));
    let ico = getNodeAttribute(node, 'ico', 'false').toLowerCase() === 'true'
    let subdivision = parseInt(getNodeAttribute(node, 'subdivision', '1,1'));

    return new WbSphere(id, radius, ico, subdivision);
  }

  parseCone(node) {
    let id = getNodeAttribute(node, 'id');
    let bottomRadius = getNodeAttribute(node, 'bottomRadius', '1');
    let height = getNodeAttribute(node, 'height', '2');
    let subdivision = getNodeAttribute(node, 'subdivision', '32');
    let side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    let bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';

    return new WbCone(id, bottomRadius, height, subdivision, side, bottom);
  }

  parseCylinder(node) {
    let id = getNodeAttribute(node, 'id');
    let radius = getNodeAttribute(node, 'radius', '1');
    let height = getNodeAttribute(node, 'height', '2');
    let subdivision = getNodeAttribute(node, 'subdivision', '32');
    let bottom = getNodeAttribute(node, 'bottom', 'true').toLowerCase() === 'true';
    let side = getNodeAttribute(node, 'side', 'true').toLowerCase() === 'true';
    let top = getNodeAttribute(node, 'top', 'true').toLowerCase() === 'true';

    return new WbCylinder(id, radius, height, subdivision, bottom, side, top);
  }

  parsePlane(node) {
    let id = getNodeAttribute(node, 'id');
    let size = convertStringToVec2(getNodeAttribute(node, 'size', '1,1'));

    return new WbPlane(id, size);
  }

  async parseAppearance(node) {
    let id = getNodeAttribute(node, 'id');

    // Get the Material tag.
    let materialNode = node.getElementsByTagName('Material')[0];
    let material;
    if (typeof materialNode !== 'undefined')
      material = this.parseMaterial(materialNode)

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
        transform = this.parseTextureTransform(textureTransform);
    }

    let appearance = new WbAppearance(id, material, texture, transform);
    if (typeof appearance !== 'undefined') {
        if(typeof material !== 'undefined')
          material.parent = appearance;
    }

    if (typeof appearance !== 'undefined') {
        if(typeof texture !== 'undefined')
          texture.parent = appearance;
    }

    if (typeof appearance !== 'undefined') {
        if(typeof transform !== 'undefined')
          transform.parent = appearance;
    }

    return appearance;
  }

  parseMaterial(node) {
    let id = getNodeAttribute(node, 'id');
    let ambientIntensity = parseFloat(getNodeAttribute(node, 'ambientIntensity', '0.2')),
    diffuseColor = convertStringToVec3(getNodeAttribute(node, 'diffuseColor', '0.8 0.8 0.8')),
    specularColor = convertStringToVec3(getNodeAttribute(node, 'specularColor', '0 0 0')),
    emissiveColor = convertStringToVec3(getNodeAttribute(node, 'emissiveColor', '0 0 0')),
    shininess = parseFloat(getNodeAttribute(node, 'shininess', '0.2')),
    transparency = parseFloat(getNodeAttribute(node, 'transparency', '0'));

    return new WbMaterial(id, ambientIntensity, diffuseColor, specularColor, emissiveColor, shininess, transparency);
  }

  async parseImageTexture(node) {
    const id = getNodeAttribute(node, 'id');
    let url = getNodeAttribute(node, 'url', '');
    url = url.slice(1, url.length-1);
    const isTransparent = getNodeAttribute(node, 'isTransparent', 'false').toLowerCase() === 'true';
    const s = getNodeAttribute(node, 'repeatS', 'true').toLowerCase() === 'true';
    const t = getNodeAttribute(node, 'repeatT', 'true').toLowerCase() === 'true';

    const textureProperties = node.getElementsByTagName('TextureProperties')[0];
    let anisotropy = 8;
    if (typeof textureProperties !== 'undefined'){
      anisotropy = parseFloat(getNodeAttribute(textureProperties, 'anisotropicDegree', '8'));
    }

    const prefix = "/projects/default/worlds/";
    let image = await this.loadTextureData(prefix+url);
    return new WbImageTexture(id, prefix + url, isTransparent, s, t, anisotropy, image);
  }

  async parsePBRAppearance(node) {
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
        transform = this.parseTextureTransform(textureTransform);
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
        baseColorMap = await this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'roughness')
        roughnessMap = await this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'metalness')
        metalnessMap = await this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'normal')
        normalMap = await this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'occlusion')
        occlusionMap = await this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'emissiveColor')
        emissiveColorMap = await this.parseImageTexture(imageTexture, textureTransform);
    }

    let pbrAppearance = new WbPBRAppearance(id, baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness, metalnessMap, IBLStrength,
       normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor, emissiveColorMap, emissiveIntensity, transform);

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof transform !== 'undefined')
          transform.parent = pbrAppearance;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof baseColorMap !== 'undefined')
          baseColorMap.parent = pbrAppearance;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof roughnessMap !== 'undefined')
          roughnessMap.parent = pbrAppearance;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof metalnessMap !== 'undefined')
          metalnessMap.parent = pbrAppearance;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof normalMap !== 'undefined')
          normalMap.parent = pbrAppearance;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof occlusionMap !== 'undefined')
          occlusionMap.parent = pbrAppearance;
    }

    if (typeof pbrAppearance !== 'undefined') {
        if(typeof emissiveColorMap !== 'undefined')
          emissiveColorMap.parent = pbrAppearance;
    }

    return pbrAppearance;
  }

  parseTextureTransform(node) {
      const id = getNodeAttribute(node, 'id');
      let center = convertStringToVec2(getNodeAttribute(node, 'center', '0 0')),
      rotation = parseFloat(getNodeAttribute(node, 'rotation', '0')),
      scale = convertStringToVec2(getNodeAttribute(node, 'scale', '1 1')),
      translation = convertStringToVec2(getNodeAttribute(node, 'translation', '0 0'))

    return new WbTextureTransform(id, center, rotation, scale, translation);
  }

  async loadTextureData(url) {
   let context = document.getElementById('canvas2').getContext('2d');
   let img = await this.loadImage(url);

   canvas2.width = img.width;
   canvas2.height = img.height;
   context.drawImage(img, 0, 0);
   let dataBGRA = context.getImageData(0, 0, img.width, img.height).data;
   let data = new Uint8ClampedArray(dataBGRA.length);
   for(let x = 0; x < dataBGRA.length; x = x+4){
     data[2+x] = dataBGRA[2+x];
     data[1+x] = dataBGRA[1+x];
     data[0+x] = dataBGRA[0+x];
     data[3+x] = dataBGRA[3+x];
   }
   let image = new WbImage();

   image.bits = data;
   image.width = img.width;
   image.height = img.height;
   return image;
 }

 loadImage(src){
   return new Promise((resolve, reject) => {
     let img = new Image();
     img.onload = () => {
       resolve(img);
     }
     img.onerror = () => console.log("Error in loading");
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
