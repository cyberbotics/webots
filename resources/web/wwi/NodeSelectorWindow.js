import Parameter from './protoVisualizer/Parameter.js';
import { VRML } from './protoVisualizer/vrml_type.js';

class ProtoInfo {
  #url;
  #baseType;
  #license;
  #description;
  constructor(url, baseType, license, description) {
    this.#url = url;
    this.#baseType = baseType;
    this.#license = license;
    this.#description = description;
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

  get description() {
    return this.#description;
  }
}

export default class NodeSelectorWindow {
  #rootProto;
  constructor(parentNode, rootProto) {
    this.#rootProto = rootProto;

    this.#setupWindow(parentNode);
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
    p.style.margin = 'auto 10px auto 0px';
    p.innerHTML = 'Find: ';

    const input = document.createElement('input');
    input.id = 'filter';
    input.type = 'text';
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
    img.className = 'node-image';
    img.draggable = false;
    img.src = 'https://raw.githubusercontent.com/cyberbotics/webots/R2023a/resources/images/missing_proto_icon.png';
    nodeInfo.appendChild(img);

    const line = document.createElement('hr');
    line.id = 'line';
    line.style.width = '75%';
    line.style.marginLeft = 'auto';
    line.style.marginRight = 'auto';
    nodeInfo.appendChild(line);

    const warning = document.createElement('span');
    warning.id = 'node-warning';
    warning.innerHTML = 'No nodes match the criteria.';
    warning.style.height = '100%';
    warning.style.verticalAlign = 'middle';
    warning.style.alignItems = 'center';
    warning.style.marginLeft = '15px';
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

  async fetchCompatibleNodes() {
    // The provider of the list of protos is webots.cloud but it should be possible to generalize it.
    let url = window.location.href;
    if (!url.includes('webots.cloud'))
      url = 'https://proto.webots.cloud/';
    url = new URL(url).hostname;

    const content = {};
    content.base_types = this.getAllowedBaseType();

    if (content.base_types.length === 1 && content.base_types[0] === 'Slot')
      content.slot_type = this.getSlotType();

    if (!this.isRobotDescendant())
      content.skip_no_robot_ancestor = true;

    return fetch('https://' + url + '/ajax/proto/insertable.php', {method: 'post', body: JSON.stringify(content)})
      .then(result => result.json())
      .then(json => {
        this.nodes = new Map();
        for (const proto of json) {
          let url = proto.url;
          url = url.replace('github.com', 'raw.githubusercontent.com').replace('/blob/', '/');
          const baseType = proto.base_type;
          const license = proto.license;
          let description = proto.description;
          if (typeof description !== 'undefined')
            description = description.replaceAll('\\n', '<br>');
          const protoInfo = new ProtoInfo(url, baseType, license, description);
          const protoName = url.split('/').pop().replace('.proto', '');
          this.nodes.set(protoName, protoInfo);
        }
      });
  }

  populateWindow() {
    const filterInput = document.getElementById('filter');

    // populate node list
    const nodeList = document.getElementById('node-list');
    nodeList.innerHTML = '';

    const compatibleNodes = [];
    for (const [name, info] of this.nodes) {
      // filter nodes based on restrictions
      if (!this.doFieldRestrictionsAllowNode(name))
        continue;

      // don't display PROTO nodes which have been filtered-out by the user's "filter" widget
      if (!info.url.toLowerCase().includes(filterInput.value) && !info.baseType.toLowerCase().includes(filterInput.value))
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
        this.selection = ol.children[compatibleNodes.indexOf(this.parameter.value.value.name)];

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
      license.innerHTML =
        'License:&nbsp;<i>' + (typeof info.license !== 'undefined' ? info.license : 'not specified.') + '</i>';
    }
  }

  insertNode(protoName) {
    if (typeof protoName === 'undefined')
      throw new Error('It should not be possible to insert an undefined node.');

    const info = this.nodes.get(protoName);
    if (this.parameter.type === VRML.SFNode)
      this.callback(this.parameter, info.url);
    else
      this.callback(this.parameter, info.url, this.element, this.parent, this.mfId, this.resetButton);
    this.hide();
  }

  doFieldRestrictionsAllowNode(nodeName) {
    if (this.parameter.restrictions.length === 0)
      return true;

    for (const restriction of this.parameter.restrictions) {
      if (nodeName === restriction.value.name)
        return true;
    }

    return false;
  }

  getAllowedBaseType() {
    if (typeof this.parameter === 'undefined')
      throw new Error('The parameter is expected to be defined prior to checking node compatibility.');

    if (this.parameter.parameterLinks.length <= 0)
      throw new Error('The parameter has no IS.');

    let baseType = [];
    const fieldName = this.parameter.parameterLinks[0].name;
    const parentNode = this.parameter.parameterLinks[0].node;
    if (fieldName === 'appearance')
      baseType = ['Appearance', 'PBRAppearance'];
    else if (fieldName === 'geometry')
      baseType = ['Box', 'Capsule', 'Cylinder', 'Cone', 'Plane', 'Sphere', 'Mesh', 'ElevationGrid', 'IndexedFaceSet',
        'IndexedLineSet'];
    else if (fieldName === 'endPoint' && parentNode.name === 'Slot')
      baseType = ['Slot'];
    else if (fieldName === 'endPoint' || fieldName === 'children')
      baseType = ['Group', 'Transform', 'Shape', 'CadShape', 'Solid', 'Robot', 'PointLight', 'SpotLight', 'Propeller',
        'Charger'];

    return baseType;
  }

  getSlotType() {
    const parentNode = this.parameter.parameterLinks[0].node;
    let slotType = parentNode.getParameterByName('type').value.value.replaceAll('"', '');

    if (slotType.endsWith('+'))
      slotType = slotType.substring(0, slotType.length - 1) + '-';
    else if (slotType.endsWith('-'))
      slotType = slotType.substring(0, slotType.length - 1) + '+';

    return slotType;
  }

  show(parameter, element, callback, parent, mfId, resetButton) {
    // cleanup input field
    const filterInput = document.getElementById('filter');
    filterInput.value = '';

    if (!(parameter instanceof Parameter))
      throw new Error('Cannot display node selector unless a parameter is provided.');

    this.parameter = parameter;
    this.element = element;
    this.callback = callback;
    this.parent = parent;
    this.mfId = mfId;
    this.resetButton = resetButton;
    this.fetchCompatibleNodes()
      .then(() => this.populateWindow());
    this.nodeSelector.style.display = 'block';
  }

  isRobotDescendant() {
    return this.#rootProto.getBaseNode().name === 'Robot';
  }

  hide() {
    this.parameter = undefined;
    this.element = undefined;
    this.callback = undefined;
    this.parent = undefined;
    this.mfId = undefined;
    this.resetButton = undefined;
    this.nodeSelector.style.display = 'none';
  }
}
