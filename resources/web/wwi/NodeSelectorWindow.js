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

export default class NodeSelectorWindow {
  #callback;
  #rootProto;
  constructor(parentNode, callback, rootProto) {
    this.#callback = callback;
    this.#rootProto = rootProto;

    this.#setupWindow(parentNode);
  }

  async initialize() {
    return new Promise((resolve, reject) => {
      const xmlhttp = new XMLHttpRequest();
      //xmlhttp.open('GET', './protoVisualizer/temporary-proto-list.xml', true);
      xmlhttp.open('GET', 'https://raw.githubusercontent.com/cyberbotics/webots/enhancement-insert-sfnode/resources/web/wwi/protoVisualizer/temporary-proto-list.xml', true);
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
        let description = proto.getElementsByTagName('description')[0]?.innerHTML;
        if (typeof description !== 'undefined')
          description = description.replace('\\n', '<br>');
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
    this.nodeSelector = document.createElement('div');
    this.nodeSelector.className = 'node-library';
    this.nodeSelector.id = 'node-library';
    parentNode.appendChild(this.nodeSelector);

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
    img.className = 'node-image'
    img.draggable = false;
    img.src = '../../images/missing_proto_icon.png';
    nodeInfo.appendChild(img);

    const line = document.createElement('hr');
    line.id = 'line';
    line.style.width = '75%'
    nodeInfo.appendChild(line);

    const warning = document.createElement('span');
    warning.id = 'node-warning';
    warning.innerHTML = 'No nodes match the criteria.';
    warning.style.height = '100%';
    warning.style.verticalAlign = 'middle';
    warning.style.alignItems = 'center';
    warning.style.color = 'rgba(240, 240, 240, 1)';
    warning.style.display = 'none';
    nodeInfo.appendChild(warning);

    const description = document.createElement('span');
    description.id = 'node-description';
    description.className = 'node-description';
    description.innerHTML = 'No description available.';
    nodeInfo.appendChild(description);

    const license = document.createElement('span');
    license.id = 'node-license';
    license.className = 'node-license';
    license.innerHTML = 'License:&nbsp;<i>not specified.</i>';
    nodeInfo.appendChild(license);

    const buttonContainer = document.createElement('div');
    buttonContainer.className = 'node-buttons';

    const acceptButton = document.createElement('button');
    acceptButton.innerHTML = 'Accept';
    acceptButton.onclick = () => {
      if (typeof this.selection === 'undefined')
        return;

      this.insertNode(this.selection.innerText);
    };

    const cancelButton = document.createElement('button');
    cancelButton.innerHTML = 'Cancel';
    cancelButton.onclick = () => this.hide();

    buttonContainer.appendChild(cancelButton);
    buttonContainer.appendChild(acceptButton);

    this.nodeSelector.appendChild(nodeFilter);
    this.nodeSelector.appendChild(nodeList);
    this.nodeSelector.appendChild(nodeInfo);
    this.nodeSelector.appendChild(buttonContainer);
  }

  populateWindow() {
    const filterInput = document.getElementById('filter');

    // populate node list
    const nodeList = document.getElementById('node-list');
    nodeList.innerHTML = '';

    const compatibleNodes = [];
    for (const [name, info] of this.nodes) {
      // filter incompatible nodes
      if (typeof info.tags !== 'undefined' && (info.tags.includes('hidden') || info.tags.includes('deprecated')))
        continue;

      // don't display PROTO nodes which have been filtered-out by the user's "filter" widget
      if (!info.url.toLowerCase().includes(filterInput.value) && !info.baseType.toLowerCase().includes(filterInput.value))
        continue;

      // don't display non-Robot PROTO nodes containing devices (e.g. Kinect) about to be inserted outside a robot
      if (typeof info.baseType === 'undefined')
        throw new Error('"base-type" property is undefined, is the proto-list.xml complete?');

      if (!this.isRobotDescendant() && !(info.baseType === 'Robot') && info.needsRobotAncestor)
        continue;

      if (!this.isAllowedToInsert(info.baseType, info.slotType))
        continue;

      compatibleNodes.push(name);
    }

    compatibleNodes.sort();

    const ol = document.createElement('ol');
    for (const name of compatibleNodes) {
      const item = this.#createNodeButton(name);
      ol.appendChild(item);
    }

    nodeList.appendChild(ol);

    // select first item by default, if any
    if (ol.children.length === 0) {
      if (typeof this.selection !== 'undefined')
        this.selection.style.backgroundColor = '';
      this.selection = undefined;
    } else {
      if (this.parameter.value.value === null || !compatibleNodes.includes(this.parameter.value.value.name))
        this.selection = ol.children[0]; // select the first item if the parameter doesn't currently contain anything
      else
        this.selection = ol.children[compatibleNodes.indexOf(this.parameter.value.value.name)]

      this.selection.style.backgroundColor = '#007acc';

    }

    // populate node info
    this.populateNodeInfo(this.selection?.innerText);
  }

  #createNodeButton(name) {
    const item = document.createElement('li');
    const button = document.createElement('button');
    button.style.width = '100%';
    button.style.textAlign = 'left';
    button.innerText = name;
    item.appendChild(button);

    button.onclick = (item) => {
      if (typeof this.selection !== 'undefined')
        this.selection.style.backgroundColor = '';

      this.selection = item.target;
      this.selection.style.backgroundColor = '#007acc';

      this.populateNodeInfo(this.selection.innerText);
    };

    button.ondblclick = async(item) => this.insertNode(item.target.innerText);

    return item;
  }

  populateNodeInfo(protoName) {
    const nodeImage = document.getElementById('node-image');
    const line = document.getElementById('line');
    const description = document.getElementById('node-description');
    const license = document.getElementById('node-license');
    const warning = document.getElementById('node-warning');

    if (typeof protoName === 'undefined') {
      nodeImage.style.display = 'none';
      line.style.display = 'none';
      description.style.display = 'none';
      license.style.display = 'none';
      warning.style.display = 'flex';
    } else {
      nodeImage.style.display = 'block';
      line.style.display = 'block';
      description.style.display = 'block';
      license.style.display = 'block';
      warning.style.display = 'none';

      const info = this.nodes.get(protoName);
      const url = info.url;
      nodeImage.src = url.slice(0, url.lastIndexOf('/') + 1) + 'icons/' + protoName + '.png';
      description.innerHTML = info.description;
      license.innerHTML = 'License:&nbsp;<i>' + info.license + '</i>'
    }
  }

  insertNode(protoName) {
    if (typeof protoName === 'undefined')
      throw new Error('It should not be possible to insert an undefined node.');

    const info = this.nodes.get(protoName);
    this.#callback(this.parameter, info.url);
    this.hide();
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
      const fieldName = link.name;

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
    this.nodeSelector.style.display = 'block';
  }

  isRobotDescendant() {
    return this.#rootProto.getBaseNode().name === 'Robot';
  }

  hide() {
    this.parameter = undefined;
    this.nodeSelector.style.display = 'none';
  }
}
