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
    if (scene === undefined) {
      console.error("Scene not found");
    } else {
      console.log("Une scene");
      this.parseNode(scene);
    }
    console.log(scene);
  }

  parseNode(node) {
    if(node.tagName === 'Scene') {
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
    console.log("Un Viewpoint");
  }

  parseShape(node){
    console.log("Une Shape");
    this.parseGeometry(node.childNodes[0]);
  }

  parseGeometry(node){
    let isGeometry = true;
    if(node.tagName === 'Box') {
      console.log('Une Boxe');
    } else {
      isGeometry = false
    }
    return isGeometry;
  }
}
