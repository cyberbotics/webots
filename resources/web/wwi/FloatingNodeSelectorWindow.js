import FloatingWindow from './FloatingWindow.js';
import Parameter from './protoVisualizer/Parameter.js';

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

    this.#setupWindow(parentNode);
  }

  async initialize() {
    return new Promise((resolve, reject) => {
      const xmlhttp = new XMLHttpRequest();
      xmlhttp.open('GET', './protoVisualizer/proto-list.xml', true);
      xmlhttp.onreadystatechange = async() => {
        if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
          resolve(xmlhttp.responseText);
      };
      xmlhttp.send();
    }).then(text => {
      this.nodes = new Map();

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
        this.nodes.set(protoName, protoInfo);
      }
    });
  }

  #setupWindow(parentNode) {
    const panel = document.createElement('div'); // TODO: add elements to floatingWindow, not this div
    panel.className = 'node-library';
    panel.id = 'node-library';
    parentNode.appendChild(panel);

    // setup search input
    const nodeFilter = document.createElement('div');
    nodeFilter.id = 'node-filter';
    nodeFilter.className = 'node-filter';
    const p = document.createElement('p');
    p.innerHTML = 'Find: ';

    const input = document.createElement('input');
    input.id = 'filter';
    input.type = 'text';
    input.style.marginRight = '10px';
    input.oninput = () => this.populateWindow();
    p.appendChild(input);
    nodeFilter.appendChild(p);

    // setup container of compatible PROTO
    const nodeList = document.createElement('div');
    nodeList.id = 'node-list';
    nodeList.className = 'node-list';

    // setup node info container
    const nodeInfo = document.createElement('div');
    nodeInfo.id = 'node-info';
    nodeInfo.className = 'node-info';

    const img = document.createElement('img');
    img.id = 'node-image';
    img.setAttribute('draggable', false);
    img.setAttribute('src', '../../images/missing_proto_icon.png');
    img.style.maxWidth = '100%';
    img.style.maxHeight = '100%';
    nodeInfo.appendChild(img);

    const buttonContainer = document.createElement('div');
    buttonContainer.className = 'node-buttons';

    const acceptButton = document.createElement('button');
    acceptButton.innerHTML = 'Accept';
    acceptButton.onclick = () => {
      // TODO: show selection in gui
      if (typeof this.selectedNode === 'undefined')
        throw new Error('No node selected.');

      this.insertNode(this.selectedNode);
    };

    const cancelButton = document.createElement('button');
    cancelButton.innerHTML = 'Cancel';
    cancelButton.onclick = () => this.hide();

    buttonContainer.appendChild(cancelButton);
    buttonContainer.appendChild(acceptButton);

    panel.appendChild(nodeFilter);
    panel.appendChild(nodeList);
    panel.appendChild(nodeInfo);
    panel.appendChild(buttonContainer);
  }

  populateWindow() {
    const filterInput = document.getElementById('filter');
    console.log('populate with filter : ' + filterInput.value + ' for nodes compatible with: ' + this.parameter.name);

    // populate node list
    const nodeList = document.getElementById('node-list');
    nodeList.innerHTML = '';

    const ol = document.createElement('ol');

    // the NULL element is always available
    ol.appendChild(this.#createNodeButton('NULL', 'null'));

    // add compatible nodes
    for (const [name, info] of this.nodes) {
      // filter incompatible nodes
      if (typeof info.tags !== 'undefined' && (info.tags.includes('hidden') || info.tags.includes('deprecated')))
        continue;

      // don't display PROTO nodes which have been filtered-out by the user's "filter" widget.
      if (!info.url.toLowerCase().includes(filterInput.value) && !info.baseType.toLowerCase().includes(filterInput.value))
        continue;

      // don't display non-Robot PROTO nodes containing devices (e.g. Kinect) about to be inserted outside a robot.
      const isRobotDescendant = false; // TODO
      if (typeof info.baseType === 'undefined')
        throw new Error('base-type property is undefined, is the xml complete?');

      if (!isRobotDescendant && !(info.baseType === 'Robot') && info.needsRobotAncestor)
        continue;

      if (!this.isAllowedToInsert(info.baseType, info.slotType))
        continue;

      const item = this.#createNodeButton(name, info.url);
      ol.appendChild(item);
    }

    nodeList.appendChild(ol);

    // remove selection
    this.selectedNode = undefined;

    // populate node info
    this.populateNodeInfo('null');
  }

  #createNodeButton(name, url) {
    const item = document.createElement('li');
    const button = document.createElement('button');
    button.innerText = name;
    button.value = url;
    item.appendChild(button);

    button.onclick = (item) => {
      this.selectedNode = item.target.value;
      this.populateNodeInfo(this.selectedNode);
    };

    button.ondblclick = async(item) => await this.insertNode(item.target.value);
    return item;
  }

  populateNodeInfo(url) {
    const nodeImage = document.getElementById('node-image');

    if (url === 'null')
      nodeImage.src = '../../images/missing_proto_icon.png';
    else {
      const protoName = url.split('/').pop().replace('.proto', '');
      nodeImage.src = url.slice(0, url.lastIndexOf('/') + 1) + 'icons/' + protoName + '.png';
    }
  }

  async insertNode(url) {
    console.log('inserting:', url);

    if (url === 'null')
      this.parameter.setValueFromJavaScript(this.#view, null);
    else {
      console.log('generating node');
      const node = await this.#protoManager.generateNodeFromUrl(url);

      this.parameter.setValueFromJavaScript(this.#view, node);
    }

    // close library panel
    this.hide();

    // update button name
    // TODO:can we use the onchange of protomanager?
    /*
    if (parameter.value.value === null) {
      nodeButton.innerHTML = 'NULL';
      configureButton.style.display = 'none';
    } else {
      nodeButton.innerHTML = parameter.value.value.name;
      configureButton.style.display = 'block';
    }
    */
  }

  isAllowedToInsert(baseType, slotType) {
    if (typeof this.parameter === 'undefined')
      throw new Error('The parameter is expected to be defined prior to checking node compatibility.')

    const baseNode = this.parameter.node.getBaseNode();

    if (baseNode.name === 'Slot' && typeof slotType !== 'undefined') {
      const otherSlotType = baseNode.getParameterByName('type').value.value.replaceAll('"', '');
      return this.isSlotTypeMatch(otherSlotType, slotType);
    }

    for (const link of this.parameter.parameterLinks) {
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

  show(parameter) {
    // cleanup input field
    const filterInput = document.getElementById('filter');
    filterInput.value = '';

    if (!(parameter instanceof Parameter))
      throw new Error('Cannot display node selector unless a parameter is provided.')

    this.parameter = parameter;
    this.populateWindow();
    const panel = document.getElementById('node-library');
    panel.style.display = 'block';
  }

  hide() {
    this.parameter = undefined;
    const panel = document.getElementById('node-library');
    panel.style.display = 'none';
  }
}
