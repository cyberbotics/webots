import FloatingWindow from './FloatingWindow.js';
import {VRML} from './protoVisualizer/vrml_type.js';
import WbHingeJoint from './nodes/WbHingeJoint.js';
import WbWorld from './nodes/WbWorld.js';

export default class FloatingProtoParameterWindow extends FloatingWindow {
  #protoManager;
  #view;
  constructor(parentNode, protoManager, view, proto) {
    super(parentNode, 'proto-parameter');
    this.floatingWindow.style.zIndex = '2';
    this.headerText.innerHTML = 'Proto window';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);

    this.joints = document.createElement('div');
    this.joints.id = 'joints-tab';
    this.joints.style.display = 'none';
    this.floatingWindowContent.appendChild(this.joints);

    this.devices = document.createElement('div');
    this.devices.id = 'devices-tab';
    this.devices.style.display = 'none';
    this.floatingWindowContent.appendChild(this.devices);

    this.#protoManager = protoManager;
    this.proto = proto;
    this.headerText.innerHTML = this.proto.name;
    this.#view = view;

    this.setupNodeSelector(parentNode);

    // create tabs
    const infoTabsBar = document.createElement('div');
    infoTabsBar.className = 'proto-tabs-bar';
    this.floatingWindowContent.prepend(infoTabsBar);

    this.tab0 = document.createElement('button');
    this.tab0.className = 'proto-tab tab0';
    this.tab0.innerHTML = 'Parameters';
    this.tab0.onclick = () => this.switchTab(0);
    infoTabsBar.appendChild(this.tab0);

    this.tab1 = document.createElement('button');
    this.tab1.className = 'proto-tab tab1';
    this.tab1.innerHTML = 'Joints';
    this.tab1.onclick = () => this.switchTab(1);
    infoTabsBar.appendChild(this.tab1);

    this.tab2 = document.createElement('button');
    this.tab2.className = 'proto-tab tab2';
    this.tab2.innerHTML = 'Devices';
    this.tab2.onclick = () => this.switchTab(2);
    infoTabsBar.appendChild(this.tab2);
  }

  switchTab(number) {
    if (number === 0) {
      this.tab0.style.backgroundColor = '#222';
      this.tab1.style.backgroundColor = '#333';
      this.tab2.style.backgroundColor = '#333';
      this.frame.style.display = 'grid';
      this.joints.style.display = 'none';
      this.devices.style.display = 'none';
    } else if (number === 1) {
      this.tab0.style.backgroundColor = '#333';
      this.tab1.style.backgroundColor = '#222';
      this.tab2.style.backgroundColor = '#333';
      this.frame.style.display = 'none';
      this.joints.style.display = 'block';
      this.devices.style.display = 'none';
    } else if (number === 2) {
      this.tab0.style.backgroundColor = '#333';
      this.tab1.style.backgroundColor = '#333';
      this.tab2.style.backgroundColor = '#222';
      this.frame.style.display = 'none';
      this.joints.style.display = 'none';
      this.devices.style.display = 'grid';
    }
  }

  populateProtoParameterWindow() {
    const contentDiv = document.getElementById('proto-parameter-content');
    if (contentDiv) {
      this.headerText.innerHTML = this.proto.name;

      contentDiv.innerHTML = '';
      let row = 1;
      if (!this.proto.isRoot)
        this.#createBackButton(contentDiv, row++);

      const keys = this.proto.parameters.keys();
      for (let key of keys) {
        const parameter = this.proto.parameters.get(key);
        if (parameter.type === VRML.SFVec3f)
          this.#createSFVec3Field(key, contentDiv, row++);
        else if (parameter.type === VRML.SFRotation)
          this.#createSFRotation(key, contentDiv, row++);
        else if (parameter.type === VRML.SFString)
          this.#createSFStringField(key, contentDiv, row++);
        else if (parameter.type === VRML.SFFloat)
          this.#createSFFloatField(key, contentDiv, row++);
        else if (parameter.type === (VRML.SFInt32))
          this.#createSFInt32Field(key, contentDiv, row++);
        else if (parameter.type === VRML.SFBool)
          this.#createSFBoolField(key, contentDiv, row++);
        else if (parameter.type === VRML.SFNode)
          this.#createSFNodeField(key, contentDiv, row++);
      }

      if (this.proto.isRoot)
        this.#createDownloadButton(contentDiv, row);
    }
  }

  setupNodeSelector(parentNode) {
    let div = document.createElement('div');
    div.className = 'node-library';
    div.setAttribute('id', 'node-library');
    parentNode.appendChild(div);
  }

  #createBackButton(parent, row) {
    const buttonContainer = document.createElement('span');
    buttonContainer.style.gridRow = '' + row + ' / ' + row;
    buttonContainer.style.gridColumn = '1 / 1';

    const backButton = document.createElement('button');
    backButton.innerHTML = 'Back';
    backButton.title = 'Return to the previous PROTO';
    backButton.onclick = () => {
      console.log('back.');
      this.proto = this.#protoManager.proto; // TODO: go one layer up, not back to root
      this.populateProtoParameterWindow();
    };
    buttonContainer.appendChild(backButton);

    parent.appendChild(buttonContainer);
  }

  #createDownloadButton(parent, row) {
    const buttonContainer = document.createElement('span');
    buttonContainer.className = 'value-parameter';
    buttonContainer.style.gridRow = '' + row + ' / ' + row;
    buttonContainer.style.gridColumn = '3 / 3';

    const input = document.createElement('input');
    input.type = 'text';
    input.title = 'New proto name';
    input.value = 'My' + this.proto.name;
    buttonContainer.appendChild(input);

    const downloadButton = document.createElement('button');
    downloadButton.innerHTML = 'Download';
    downloadButton.title = 'Download the new proto';
    downloadButton.onclick = () => {
      const fields = document.getElementsByClassName('key-parameter');
      let fieldsToExport = new Set();
      if (fields) {
        for (let i = 0; i < fields.length; i++) {
          if (fields[i].checkbox.checked)
            fieldsToExport.add(fields[i].key);
        }
      }

      const data = this.#protoManager.exportProto(input.value, fieldsToExport);
      const downloadLink = document.createElement('a');
      downloadLink.download = input.value + '.proto';
      const file = new Blob([data], {
        type: 'text/plain'
      });
      downloadLink.href = window.URL.createObjectURL(file);
      downloadLink.click();
    };
    buttonContainer.appendChild(downloadButton);
    parent.appendChild(buttonContainer);
  }

  #createSFVec3Field(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.key = key;
    p.inputs = [];
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const values = document.createElement('p');
    values.style.gridRow = '' + row + ' / ' + row;
    values.style.gridColumn = '3 / 3';
    values.className = 'value-parameter';
    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => this.#vector3OnChange(p)));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, values, () => this.#vector3OnChange(p)));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, values, () => this.#vector3OnChange(p)));

    const resetButton = this.#createResetButton(values);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      p.inputs[2].value = parameter.defaultValue.value.z;
      this.#vector3OnChange(p);
    };

    parent.appendChild(p);
    parent.appendChild(values);
  }

  #createSFRotation(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.inputs = [];
    p.key = key;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const values = document.createElement('p');
    values.style.gridRow = '' + row + ' / ' + row;
    values.style.gridColumn = '3 / 3';
    values.className = 'value-parameter';
    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' a', parameter.value.value.a, values, () => this.#rotationOnChange(p)));
    const resetButton = this.#createResetButton(values);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      p.inputs[2].value = parameter.defaultValue.value.z;
      p.inputs[3].value = parameter.defaultValue.value.a;
      this.#rotationOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(values);
  }

  #createVectorInput(name, initialValue, parent, callback) {
    const span = document.createElement('span');
    span.innerHTML = name + ': ';
    const input = document.createElement('input');
    input.type = 'number';
    input.value = initialValue;
    input.step = 0.1;
    input.style.width = '50px';
    input.onchange = () => callback(parent);

    span.appendChild(input);
    parent.appendChild(span);

    return input;
  }

  #rotationOnChange(node) {
    const object = {'x': node.inputs[0].value, 'y': node.inputs[1].value, 'z': node.inputs[2].value, 'a': node.inputs[3].value};
    node.parameter.setValueFromJavaScript(this.#view, object);
  }

  #vector3OnChange(node) {
    const object = {'x': node.inputs[0].value, 'y': node.inputs[1].value, 'z': node.inputs[2].value};
    node.parameter.setValueFromJavaScript(this.#view, object);
  }

  #createSFStringField(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.parameter = parameter;
    p.key = key;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.className = 'key-parameter';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const value = document.createElement('p');
    value.style.gridRow = '' + row + ' / ' + row;
    value.style.gridColumn = '3 / 3';
    value.className = 'value-parameter';

    const input = document.createElement('input');
    input.type = 'text';

    const string = parameter.value.value;

    input.value = this.#stringRemoveQuote(string);
    input.style.height = '20px';

    input.onchange = () => this.#stringOnChange(p);
    p.input = input;
    value.appendChild(input);

    const resetButton = this.#createResetButton(value);
    resetButton.onclick = () => {
      input.value = this.#stringRemoveQuote(parameter.defaultValue.value);
      this.#stringOnChange(p);
    };

    parent.appendChild(p);
    parent.appendChild(value);
  }

  #stringOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.value);
  }

  #stringRemoveQuote(string) {
    if (typeof string !== 'undefined' && string.length > 0) {
      string = string?.trim();
      if (string[0] === '"')
        string = string.substring(1);
      if (string[string.length - 1] === '"')
        string = string.slice(0, string.length - 1);
    }
    return string;
  }

  #createSFFloatField(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.parameter = parameter;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + row + ' / ' + row;
    value.style.gridColumn = '3 / 3';

    const input = document.createElement('input');
    input.type = 'number';
    input.step = 0.1;
    input.value = parameter.value.value;
    input.style.width = '50px';

    input.onchange = () => this.#floatOnChange(p);
    p.input = input;
    value.appendChild(input);

    const resetButton = this.#createResetButton(value);
    resetButton.onclick = () => {
      input.value = parameter.defaultValue.value;
      this.#floatOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  #createSFNodeField(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.inputs = [];
    p.parameter = parameter;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + row + ' / ' + row;
    value.style.gridColumn = '3 / 3';

    const buttonContainer = document.createElement('div');
    buttonContainer.style.display = 'flex';

    const configureButton = document.createElement('button');
    configureButton.innerHTML = 'configure';
    configureButton.onclick = async() => {
      console.log('configure.');
      this.proto = parameter.value.value;
      this.populateProtoParameterWindow();
    };

    const nodeButton = document.createElement('button');
    nodeButton.title = 'Select a node to insert';
    nodeButton.onclick = async() => {
      console.log('clicked.');
      return new Promise((resolve, reject) => {
        const xmlhttp = new XMLHttpRequest();
        xmlhttp.open('GET', '../../proto-list.xml', true);
        xmlhttp.onreadystatechange = async() => {
          if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
            resolve(xmlhttp.responseText);
        };
        xmlhttp.send();
      }).then(text => {
        this.#populateNodeLibrary(text, parameter, nodeButton, configureButton);
      });
    };

    if (parameter.value.value === null) {
      configureButton.style.display = 'none';
      nodeButton.innerHTML = 'NULL';
    } else {
      configureButton.style.display = 'block';
      configureButton.title = 'Configure ' + parameter.value.value.name + ' node';
      nodeButton.innerHTML = parameter.value.value.name;
    }

    buttonContainer.appendChild(nodeButton);
    buttonContainer.appendChild(configureButton);
    value.append(buttonContainer);

    const resetButton = this.#createResetButton(buttonContainer);
    resetButton.onclick = () => {
      throw new Error('TODO: implement reset.');
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  async #populateNodeLibrary(protoList, parameter, nodeButton, configureButton) {
    let panel = document.getElementById('node-library');
    panel.innerHTML = '';
    panel.style.display = 'block';

    // TODO: filter clicks inside the panel
    // window.addEventListener('click', _ => {
    //   console.log('close')
    //   panel.style.display = 'none';
    // });

    let protoNodes = [{name: 'NULL', url: null}];

    const parser = new DOMParser();
    const xml = parser.parseFromString(protoList, 'text/xml').firstChild;
    for (const proto of xml.getElementsByTagName('proto')) {
      // don't display PROTOs which contain a "hidden" or a "deprecated" tag
      const tags = proto.getElementsByTagName('tags')[0]?.innerHTML.split(',');
      if (typeof tags !== 'undefined' && (tags.includes('hidden') || tags.includes('deprecated')))
        continue;

      // don't display PROTO nodes which have been filtered-out by the user's "filter" widget.
      // TODO

      // don't display non-Robot PROTO nodes containing devices (e.g. Kinect) about to be inserted outside a robot.
      const needsRobotAncestor = proto.getElementsByTagName('needs-robot-ancestor')[0]?.innerHTML;
      const baseType = proto.getElementsByTagName('base-type')[0]?.innerHTML;
      const isRobotDescendant = false; // TODO
      if (typeof baseType === 'undefined')
        throw new Error('base-type property is undefined, is the xml complete?');

      if (!isRobotDescendant && !(baseType === 'Robot') && needsRobotAncestor)
        continue;

      const slotType = proto.getElementsByTagName('slot-type')[0]?.innerHTML;
      if (!this.isAllowedToInsert(parameter, baseType, slotType))
        continue;

    /*

    QString errorMessage;
    const QString nodeName = it.key();
    if (!WbNodeUtilities::isAllowedToInsert(mField, baseType, mCurrentNode, errorMessage, nodeUse, info->slotType(),
                                            QStringList() << baseType << nodeName))
      continue;

    */


      const info = {};
      info['name'] = proto.getElementsByTagName('name')[0].innerHTML;
      info['url'] = proto.getElementsByTagName('url')[0].innerHTML;
      protoNodes.push(info);
    }

    const nodeList = document.createElement('div');
    nodeList.id = 'node-list';
    nodeList.className = 'node-list';
    let ol = document.createElement('ol');
    for (const node of protoNodes) {
      const item = document.createElement('li');
      const button = document.createElement('button');
      button.innerText = node['name'];
      button.value = node['url'];
      item.appendChild(button);

      button.onclick = (element) => {
        const url = element.target.value;

        const protoName = url.split('/').pop().replace('.proto', '');
        const iconUrl = url.slice(0, url.lastIndexOf('/') + 1) + 'icons/' + protoName + '.png';
        const nodeImage = document.getElementById('node-image');
        nodeImage.setAttribute('src', iconUrl);
      };

      button.ondblclick = async(element) => {
        const url = element.target.value;
        console.log('inserting:', url);

        // const protoManager = new ProtoManager(this.#view);
        // await protoManager.loadProto(url);
        // const x3d = new XMLSerializer().serializeToString(protoManager.proto.toX3d());
        // console.log(x3d);

        if (url === 'null')
          parameter.setValueFromJavaScript(this.#view, null);
        else {
          console.log('generating node');
          const node = await this.#protoManager.generateNodeFromUrl(url);
          // const x3d = new XMLSerializer().serializeToString(node.toX3d());
          // console.log('Grafted x3d:', x3d);
          // console.log(parameter);

          // const sfnode = new SFNode();
          // sfnode.setValue(node);

          parameter.setValueFromJavaScript(this.#view, node);
        }

        // close library panel
        panel.style.display = 'none';

        // update button name
        if (parameter.value.value === null) {
          nodeButton.innerHTML = 'NULL';
          configureButton.style.display = 'none';
        } else {
          nodeButton.innerHTML = parameter.value.value.name;
          configureButton.style.display = 'block';
        }
      };

      ol.appendChild(item);
    }
    nodeList.appendChild(ol);

    const nodeInfo = document.createElement('div');
    nodeInfo.id = 'node-info';
    nodeInfo.className = 'node-info';

    // const p = document.createElement('p');
    // p.innerText = 'Select a node';
    // nodeInfo.append(p);

    const img = document.createElement('img');
    img.id = 'node-image';
    img.setAttribute('draggable', false);
    img.setAttribute('src', './protoVisualizer/red_texture.jpg');
    img.style.maxWidth = '100%';
    img.style.maxHeight = '100%';

    nodeInfo.appendChild(img);

    const container = document.createElement('div');
    container.appendChild(nodeList);
    container.appendChild(nodeInfo);
    panel.appendChild(container);
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

  #floatOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.value);
  }

  #createSFInt32Field(key, parent, row) {
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.parameter = parameter;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';

    const exportCheckbox = this.#createCheckbox(parent, row);

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + row + ' / ' + row;
    value.style.gridColumn = '3 / 3';

    const input = document.createElement('input');
    input.type = 'number';
    input.step = 0.1;
    input.value = parameter.value.value;
    input.style.width = '50px';

    input.oninput = () => this.#intOnChange(p);
    input.onchange = () => this.#floatOnChange(p);
    p.input = input;
    p.checkbox = exportCheckbox;
    value.appendChild(input);

    const resetButton = this.#createResetButton(value);
    resetButton.onclick = () => {
      input.value = parameter.defaultValue.value;
      this.#floatOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  #intOnChange(node) {
    node.input.value = node.input.value.replace(/[^0-9-]/g, '');
    node.input.value = node.input.value.replace(/(\..*)\-/g, '$1');
  }

  #createSFBoolField(key, parent, row) {
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.parameter = parameter;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';

    const exportCheckbox = this.#createCheckbox(parent, row);

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + row + ' / ' + row;
    value.style.gridColumn = '3 / 3';

    const input = document.createElement('input');
    input.type = 'checkbox';
    input.checked = parameter.value.value;
    input.style.width = '50px';

    input.onchange = () => this.#boolOnChange(p);
    p.input = input;
    p.checkbox = exportCheckbox;
    value.appendChild(input);

    const resetButton = this.#createResetButton(value);
    resetButton.onclick = () => {
      input.checked = parameter.defaultValue.value;
      this.#boolOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  #boolOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.checked);
  }

  #createResetButton(parentNode) {
    const resetButton = document.createElement('button');
    resetButton.className = 'reset-field-button';
    resetButton.title = 'Reset to initial value';

    parentNode.appendChild(resetButton);
    return resetButton;
  }

  #createCheckbox(parent, row) {
    const exportCheckbox = document.createElement('input');
    exportCheckbox.type = 'checkbox';
    exportCheckbox.className = 'export-checkbox';
    exportCheckbox.title = 'Field to be exposed';
    exportCheckbox.style.gridRow = '' + row + ' / ' + row;
    exportCheckbox.style.gridColumn = '1 / 1';
    parent.appendChild(exportCheckbox);

    return exportCheckbox;
  }

  populateDeviceTab() {
    this.devices.innerHTML = '';
    const noDevice = document.createElement('h1');
    noDevice.innerHTML = 'No devices';
    this.devices.appendChild(noDevice);
  }

  populateJointTab() {
    this.listJoints();
  }

  listJoints() {
    this.joints.innerHTML = '';
    const nodes = WbWorld.instance.nodes;
    const keys = nodes.keys();
    let numberOfJoint = 0;
    for (const key of keys) {
      const joint = nodes.get(key);
      if (joint instanceof WbHingeJoint) {
        numberOfJoint++;

        let div = document.createElement('div');
        div.className = 'proto-joint';

        const nameDiv = document.createElement('div');
        nameDiv.innerHTML = 'Joint ' + numberOfJoint;
        nameDiv.className = 'proto-joint-name';
        div.appendChild(nameDiv);

        const sliderElement = document.createElement('div');
        sliderElement.className = 'proto-slider-element';

        const slider = document.createElement('input');
        slider.className = 'proto-slider';
        slider.type = 'range';
        slider.step = 'any';

        const minLabel = document.createElement('div');
        minLabel.className = 'proto-joint-value-label';

        const maxLabel = document.createElement('div');
        maxLabel.className = 'proto-joint-value-label';

        const parameters = joint.jointParameters;
        if (typeof parameters !== 'undefined' && parameters.minStop !== parameters.maxStop) {
          minLabel.innerHTML = parameters.minStop;
          slider.min = parameters.minStop;
          maxLabel.innerHTML = parameters.maxStop;
          slider.max = parameters.maxStop;
        } else {
          minLabel.innerHTML = -3.14;
          slider.min = -3.14;
          maxLabel.innerHTML = 3.14;
          slider.max = 3.14;
        }

        if (typeof parameters === 'undefined')
          slider.value = 0;
        else
          slider.value = parameters.position;

        slider.addEventListener('input', _ => {
          if (parameters) {
            parameters.position = _.target.value;
            this.#view.x3dScene.render();
          }
        });

        sliderElement.appendChild(minLabel);
        sliderElement.appendChild(slider);
        sliderElement.appendChild(maxLabel);
        div.appendChild(sliderElement);
        this.joints.appendChild(div);
      }
    }

    if (numberOfJoint === 0) {
      const title = document.createElement('h1');
      title.innerHTML = 'No joints';
      this.joints.appendChild(title);
    }
  }
}
