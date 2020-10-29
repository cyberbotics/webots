class MyParser {
  constructor() {
    this.Nodes = [];
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
    if (scene === undefined) {
      console.error("Scene not found");
    } else {
      console.log("Une scene");
      this.parseNode(scene);
    }
    _wr_scene_render(_wr_scene_get_instance(), null, true);

    console.log("File Parsed");
  }

  parseNode(node) {
    if(node.tagName === 'Scene') {
      let id = getNodeAttribute(node, 'id');
      new WbScene(id);
      this.parseChildren(node);
    } else if (node.tagName === 'WorldInfo') {
      this.parseWorldInfo(node);
    } else if (node.tagName === 'Viewpoint') {
      this.parseViewpoint(node);
    } else if (node.tagName === 'Shape') {
      this.parseShape(node);
    } else {
      console.log(node.tagName);
      console.error("The parser doesn't support this type of node");
    }
  }

  parseChildren(node, currentObject) {
    for (let i = 0; i < node.childNodes.length; i++) {
      let child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined'){
        this.parseNode(child);
      }

    }
  }

  parseWorldInfo(node){
    console.log("Un WorldInfo");
  }

  parseViewpoint(node){
    let id = getNodeAttribute(node, 'id');
    let orientation = convertStringToQuaternion(getNodeAttribute(node, 'orientation'));
    let position = convertStringToVec3(getNodeAttribute(node, 'position'));
    let exposure = parseFloat(getNodeAttribute(node, 'exposure'));
    let bloomThreshold = parseFloat(getNodeAttribute(node, 'bloomThreshold'));
    let far = parseFloat(getNodeAttribute(node, 'far'));
    let zNear = parseFloat(getNodeAttribute(node, 'zNear'));
    let followsmoothness = parseFloat(getNodeAttribute(node, 'followsmoothness'));

    let viewpoint = new WbViewpoint(id, orientation, position, exposure, bloomThreshold, zNear, far, followsmoothness);
  }

  parseShape(node){
    console.log("Une Shape");
    let id = getNodeAttribute(node, 'id');
    let castShadows = getNodeAttribute(node, 'castShadows').toLowerCase() === 'true';

    let shape = new WbShape(id, castShadows);

    let geometry = this.parseGeometry(node.childNodes[0]);
    shape.geometry = geometry;
    shape.createWrenObjects();
  }

  parseGeometry(node){
    let geometry;
    if(node.tagName === 'Box') {
      geometry = this.parseBox(node);
    } else {
      geometry = undefined
    }
    return geometry;
  }

  parseBox(node) {
    console.log("Une box");
    let id = getNodeAttribute(node, 'id');
    let size = convertStringToVec3(getNodeAttribute(node, 'size'));
    if(size === undefined)
      size = glm.vec3(0.1,0.1,0.1);

    return new WbBox(id, size);
  }
}

function getNodeAttribute(node, attributeName) {
  console.assert(node && node.attributes);
  if (attributeName in node.attributes)
    return node.attributes.getNamedItem(attributeName).value;
  return undefined;
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
