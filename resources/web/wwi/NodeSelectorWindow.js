import {FieldModel} from './protoVisualizer/FieldModel.js';
import Parameter from './protoVisualizer/Parameter.js';
import { VRML } from './protoVisualizer/vrml_type.js';

class NodeInfo {
  #url;
  #baseType;
  #license;
  #description;
  #icon;
  constructor(url, baseType, license, description, icon) {
    this.#url = url;
    this.#baseType = baseType;
    this.#license = license;
    this.#description = description;
    this.#license = license;
    this.#icon = icon;
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

  get icon() {
    return this.#icon;
  }
}

export default class NodeSelectorWindow {
  #rootProto;
  #devices = ['Accelerometer', 'Altimeter', 'Camera', 'Compass', 'Connector', 'Display',
    'DistanceSensor', 'Emitter', 'GPS', 'Gyro', 'InertialUnit', 'LED', 'Lidar', 'LightSensor', 'Pen', 'Radar',
    'RangeFinder', 'Receiver', 'Speaker', 'TouchSensor', 'VacuumGripper'];

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
    img.src = 'https://raw.githubusercontent.com/cyberbotics/webots/R2023b/resources/images/missing_proto_icon.png';
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

      this.insertNode(this.selection.innerText, this.selection.baseNode ? this.baseNodes : this.protoNodes);
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
      url = 'https://webots.cloud/';
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
        this.protoNodes = new Map();
        for (const proto of json) {
          let url = proto.url;
          url = url.replace('github.com', 'raw.githubusercontent.com').replace('/blob/', '/');
          const baseType = proto.base_type;
          const license = proto.license;
          let description = proto.description;
          if (typeof description !== 'undefined')
            description = description.replaceAll('\\n', '<br>');
          const protoName = url.split('/').pop().replace('.proto', '');
          const icon = url.slice(0, url.lastIndexOf('/') + 1) + 'icons/' + protoName + '.png';
          const nodeInfo = new NodeInfo(url, baseType, license, description, icon);
          this.protoNodes.set(protoName, nodeInfo);
        }

        // add base nodes
        this.baseNodes = new Map();
        for (const baseType of content.base_types) {
          if (Object.keys(FieldModel).includes(baseType)) {
            const description = FieldModel[baseType]['description'].replaceAll('\\n', '<br>');
            const icon = FieldModel[baseType]['icon'];
            const license = 'Apache License 2.0';
            this.baseNodes.set(baseType, new NodeInfo(baseType, baseType, license, description, icon));
          }
        }
      });
  }

  populateWindow() {
    const filterInput = document.getElementById('filter');

    // populate node list
    const nodeList = document.getElementById('node-list');
    nodeList.innerHTML = '';

    const compatibleBaseNodes = [];
    for (const [name, info] of this.baseNodes) {
      // filter nodes based on restrictions
      if (!this.doFieldRestrictionsAllowNode(name))
        continue;

      // don't display base nodes nodes which have been filtered-out by the user's "filter" widget
      if (!info.url.toLowerCase().includes(filterInput.value) && !info.baseType.toLowerCase().includes(filterInput.value))
        continue;

      compatibleBaseNodes.push(name);
    }

    compatibleBaseNodes.sort();

    const compatibleProtoNodes = [];
    for (const [name, info] of this.protoNodes) {
      // filter nodes based on restrictions
      if (!this.doFieldRestrictionsAllowNode(name))
        continue;

      // don't display PROTO nodes which have been filtered-out by the user's "filter" widget
      if (!info.url.toLowerCase().includes(filterInput.value) && !info.baseType.toLowerCase().includes(filterInput.value))
        continue;

      compatibleProtoNodes.push(name);
    }

    compatibleProtoNodes.sort();

    const baseNodeItems = document.createElement('ol');
    for (const name of compatibleBaseNodes) {
      const item = this.#createNodeButton(name, true);
      baseNodeItems.appendChild(item);
    }

    nodeList.appendChild(baseNodeItems);

    if (compatibleBaseNodes.length > 0 && compatibleProtoNodes.length > 0) {
      const hr = document.createElement('hr');
      hr.style.width = '100%';
      nodeList.appendChild(hr);
    }

    const protoItems = document.createElement('ol');
    for (const name of compatibleProtoNodes) {
      const item = this.#createNodeButton(name);
      protoItems.appendChild(item);
    }

    nodeList.appendChild(protoItems);

    // select first item by default, if any
    if (baseNodeItems.children.length === 0 && protoItems.children.length === 0) {
      if (typeof this.selection !== 'undefined')
        this.selection.style.backgroundColor = '';
      this.selection = undefined;
    } else {
      // select the node corresponding to the current value if one is present
      if (this.parameter.type === VRML.SFNode) {
        if (this.parameter.value.value === null) {
          if (baseNodeItems.children.length > 0)
            this.selection = baseNodeItems.children[0];
          else
            this.selection = protoItems.children[0];
        } else {
          let index = compatibleBaseNodes.indexOf(this.parameter.value.value.name);
          if (index === -1) {
            index = compatibleProtoNodes.indexOf(this.parameter.value.value.name);
            if (index !== -1)
              this.selection = protoItems.children[index];
          } else
            this.selection = baseNodeItems.children[index];
        }
      } else if (this.parameter.type === VRML.MFNode) {
        if (this.parameter.value.value.length === 0) {
          if (baseNodeItems.children.length > 0)
            this.selection = baseNodeItems.children[0];
          else
            this.selection = protoItems.children[0];
        } else {
          const name = this.parameter.value.value[0].value.name;
          let index = compatibleBaseNodes.indexOf(name);
          if (index === -1) {
            index = compatibleProtoNodes.indexOf(name);
            if (index !== -1)
              this.selection = protoItems.children[index];
          } else
            this.selection = baseNodeItems.children[index];
        }
      }

      if (typeof this.selection !== 'undefined')
        this.selection.style.backgroundColor = '#007acc';
    }

    // populate node info
    this.populateNodeInfo(this.selection?.innerText);
  }

  #createNodeButton(name, baseNode = false) {
    const item = document.createElement('li');
    const button = document.createElement('button');
    button.style.width = '100%';
    button.style.textAlign = 'left';
    button.innerHTML = baseNode ? name.italics() : name;
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

  populateNodeInfo(nodeName) {
    const nodeImage = document.getElementById('node-image');
    const line = document.getElementById('line');
    const description = document.getElementById('node-description');
    const license = document.getElementById('node-license');
    const warning = document.getElementById('node-warning');

    if (typeof nodeName === 'undefined') {
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

      const info = this.baseNodes.has(nodeName) ? this.baseNodes.get(nodeName) : this.protoNodes.get(nodeName);
      nodeImage.src = info.icon;
      nodeImage.onerror = () => {
        nodeImage.onerror = undefined;
        nodeImage.src = 'https://raw.githubusercontent.com/cyberbotics/webots/R2023b/resources/images/missing_proto_icon.png';
      };
      description.innerHTML = info.description;
      license.innerHTML =
        'License:&nbsp;<i>' + (typeof info.license !== 'undefined' ? info.license : 'not specified.') + '</i>';
    }
  }

  insertNode(nodeName) {
    if (typeof nodeName === 'undefined')
      throw new Error('It should not be possible to insert an undefined node.');

    const info = this.baseNodes.has(nodeName) ? this.baseNodes.get(nodeName) : this.protoNodes.get(nodeName);
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
      if (this.parameter.type === VRML.MFNode) {
        for (const child of restriction.value) {
          if (nodeName === child.value.name)
            return true;
        }
      } else if (nodeName === restriction.value.name)
        return true;
    }

    return false;
  }

  getAllowedBaseType() {
    if (typeof this.parameter === 'undefined')
      throw new Error('The parameter is expected to be defined prior to checking node compatibility.');

    let baseType = [];
    // get the field name linked to the parameter (full-depth)
    let p = this.parameter;
    while (p instanceof Parameter && p.aliasLinks.length > 0)
      p = p.aliasLinks[0];
    const fieldName = p.name;
    const parentNode = p.node;

    const isInBoundingObject = this.isInBoundingObject(fieldName, parentNode);

    if (fieldName === 'appearance')
      baseType = ['Appearance', 'PBRAppearance'];
    else if (fieldName === 'geometry') {
      baseType = ['Box', 'Capsule', 'Cylinder', 'Cone', 'Plane', 'Sphere', 'Mesh', 'ElevationGrid', 'IndexedFaceSet',
        'IndexedLineSet'];
    } else if (fieldName === 'endPoint' && parentNode.name === 'Slot')
      baseType = ['Slot'];
    else if (fieldName === 'endPoint' || fieldName === 'children') {
      if (fieldName === 'children' && isInBoundingObject) {
        baseType = ['Box', 'Capsule', 'Cylinder', 'ElevationGrid', 'IndexedFaceSet', 'Mesh', 'Plane', 'Shape',
          'Sphere', 'Pose'];
      } else {
        if (this.isTransformOrTransformDescendant(p))
          baseType = ['Shape', 'CadShape', 'Group', 'Pose', 'Transform'];
        else {
          baseType = ['Group', 'Transform', 'Shape', 'CadShape', 'Solid', 'Robot', 'PointLight', 'Pose', 'SpotLight',
            'Propeller', 'Charger'];
          if (this.isRobotDescendant())
            baseType = baseType.concat(this.#devices);
        }
      }
    } else if (['baseColorMap', 'roughnessMap', 'metalnessMap', 'normalMap', 'occlusionMap',
      'emissiveColorMap', 'texture'].includes(fieldName))
      baseType = ['ImageTexture'];
    else if (fieldName === 'textureTransform')
      baseType = ['TextureTransform'];
    else if (fieldName === 'boundingObject') {
      baseType = ['Box', 'Capsule', 'Cylinder', 'ElevationGrid', 'Group', 'IndexedFaceSet', 'Mesh', 'Plane', 'Shape',
        'Sphere', 'Pose'];
    } else if (fieldName === 'physics')
      baseType = ['Physics'];
    else if (fieldName === 'immersionProperties')
      baseType = ['ImmersionProperties'];
    else if (fieldName === 'damping')
      baseType = ['Damping'];
    else if (fieldName === 'lens')
      baseType = ['Lens'];
    else if (fieldName === 'focus')
      baseType = ['Focus'];
    else if (fieldName === 'zoom')
      baseType = ['Zoom'];
    else if (fieldName === 'recognition')
      baseType = ['Recognition'];
    else if (fieldName === 'lensFlare')
      baseType = ['LensFlare'];
    else if (fieldName === 'material')
      baseType = ['Material'];
    else if (fieldName === 'rotatingHead')
      baseType = ['Solid', 'Robot', 'Charger', 'Track'].concat(this.#devices);

    return baseType;
  }

  isTransformOrTransformDescendant(field) {
    let node = field?.node;

    if (typeof node === 'undefined')
      return false;

    if (node.isProto)
      node = node.baseType;

    if (node.name === 'Transform')
      return true;

    return this.isTransformOrTransformDescendant(node.parentField);
  }

  getSlotType() {
    let parentNode;
    if (this.parameter instanceof Parameter) {
      parentNode = this.parameter.aliasLinks[0].node;
      let name = this.parameter.aliasLinks[0].name;
      while (parentNode.isProto) {
        const parameter = parentNode.parameters.get(name);
        if (typeof parameter !== 'undefined') {
          parentNode = parameter.aliasLinks[0].node;
          name = parameter.aliasLinks[0].name;
        } else {
          console.error('Not able to find slot type.');
          break;
        }
      }
    } else
      parentNode = this.parameter.node;

    let slotType = parentNode.fields.get('type').value.value.replaceAll('"', '');

    if (slotType.endsWith('+'))
      slotType = slotType.substring(0, slotType.length - 1) + '-';
    else if (slotType.endsWith('-'))
      slotType = slotType.substring(0, slotType.length - 1) + '+';

    return slotType;
  }

  show(parameter, element, callback, parent, mfId, resetButton) {
    // global variable set by webots.cloud
    if (window.webotsJSPreventAddNode) {
      alert('New nodes can only be inserted in protos that are registered on webots.cloud.');
      return;
    }

    // cleanup input field
    const filterInput = document.getElementById('filter');
    filterInput.value = '';

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

  isInBoundingObject(fieldName, parentNode) {
    if (fieldName === 'boundingObject')
      return true;

    let node = parentNode;
    let parentField = node?.parentField;

    while (typeof node !== 'undefined' && typeof parentField !== 'undefined') {
      while (parentField instanceof Parameter && parentField.aliasLinks.length > 0)
        parentField = parentField.aliasLinks[0];

      if (parentField.name === 'boundingObject')
        return true;

      node = node.parentField.node;
      parentField = node?.parentField;
    }

    return false;
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
