import FloatingWindow from './FloatingWindow.js';
import { VRML } from './protoVisualizer/vrml_type.js';

class ProtoInfo {
  #url;
  #baseType;
  #license;
  #licenseUrl;
  #description;
  #slotType;
  #tags;
  #needsRobotAncestor;
  constructor(url, baseType, license, licenseUrl, description, slotType, tags, needsRobotAncestor) {
    this.#url = url;
    this.#baseType = baseType;
    this.#license = license;
    this.#licenseUrl = licenseUrl;
    this.#description = description;
    this.#slotType = slotType;
    this.#tags = tags;
    this.#needsRobotAncestor = needsRobotAncestor;
  }

  get url() {
    return this.#url;
  }

  get baseType() {
    return this.#baseType;
  }

  get license() {
    return this.#license;
  }

  get licenseUrl() {
    return this.#licenseUrl;
  }

  get description() {
    return this.#description;
  }

  get slotType() {
    return this.#slotType;
  }
  get tags() {
    return this.#tags;
  }
  get needsRobotAncestor() {
    return this.#needsRobotAncestor;
  }
}

export default class FloatingNodeSelectorWindow extends FloatingWindow {
  #protoManager;
  #view;
  constructor(parentNode, protoManager, view, proto) {
    super(parentNode, 'node-selector');
    this.floatingWindow.style.zIndex = '3';
    this.headerText.innerHTML = 'Proto window';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);

    this.#protoManager = protoManager;
    this.proto = proto;
    this.#view = view;

    const div = document.createElement('div');
    div.className = 'node-library';
    div.id = 'node-library';
    parentNode.appendChild(div);

    return new Promise((resolve, reject) => {
      const xmlhttp = new XMLHttpRequest();
      xmlhttp.open('GET', '../../proto-list.xml', true);
      xmlhttp.onreadystatechange = async() => {
        if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
          resolve(xmlhttp.responseText);
      };
      xmlhttp.send();
    }).then(text => {
      this.protoList = new Map();

      const parser = new DOMParser();
      const xml = parser.parseFromString(text, 'text/xml').firstChild;
      for (const proto of xml.getElementsByTagName('proto')) {
        const url = proto.getElementsByTagName('url')[0]?.innerHTML;
        const baseType = proto.getElementsByTagName('base-type')[0]?.innerHTML;
        const license = proto.getElementsByTagName('license')[0]?.innerHTML;
        const licenseUrl = proto.getElementsByTagName('license-url')[0]?.innerHTML;
        const description = proto.getElementsByTagName('description')[0]?.innerHTML;
        const slotType = proto.getElementsByTagName('slot-type')[0]?.innerHTML;
        const items = proto.getElementsByTagName('tags')[0]?.innerHTML.split(',');
        const tags = typeof items !== 'undefined' ? items : [];
        const needsRobotAncestor = proto.getElementsByTagName('needs-robot-ancestor')[0]?.innerHTML === 'true';
        const protoInfo = new ProtoInfo(url, baseType, license, licenseUrl, description, slotType, tags, needsRobotAncestor);

        const protoName = url.split('/').pop().replace('.proto', '');
        this.protoList.set(protoName, protoInfo);
      }
    });
  }

  populateWindow() {
  }

  filterNodeLibrary(element) {
    console.log(element.target.value);
  }

  isAllowedToInsert(parameter, baseType, slotType) {
    const baseNode = parameter.node.getBaseNode();

    if (baseNode.name === 'Slot' && typeof slotType !== 'undefined') {
      const otherSlotType = baseNode.getParameterByName('type').value.value.replaceAll('"', '');
      return this.isSlotTypeMatch(otherSlotType, slotType);
    }

    for (const link of parameter.parameterLinks) {
      // console.log('found ' + link.node.name, link.name);
      const fieldName = link.name; // TODO: does it work for derived proto? or need to get the basenode equivalent first?

      if (fieldName === 'appearance') {
        if (baseType === 'Appearance')
          return true;
        else if (baseType === 'PBRAppearance')
          return true;
      }

      if (fieldName === 'geometry')
        return this.isGeometryTypeMatch(baseType);
    }

    return false;
  }

  isSlotTypeMatch(firstType, secondType) {
    // console.log('compare slot type: ', firstType, ' with ', secondType);
    if (typeof firstType === 'undefined' || typeof secondType === 'undefined')
      throw new Error('Cannot determine slot match because inputs are undefined.');

    if (firstType.length === 0 || secondType.length === 0)
      return true; // empty type matches any type
    else if (firstType.endsWith('+') || firstType.endsWith('-')) {
      // gendered slot types
      if (firstType.slice(0, -1) === secondType.slice(0, -1)) {
        if (firstType === secondType)
          return false; // same gender
        else
          return true; // different gender
      }
    } else if (firstType === secondType)
      return true;

    return false;
  }

  isGeometryTypeMatch(type) {
    return ['Box', 'Capsule', 'Cylinder', 'Cone', 'Plane', 'Sphere', 'Mesh', 'ElevationGrid',
      'IndexedFaceSet', 'IndexedLineSet'].includes(type);
  }
}
