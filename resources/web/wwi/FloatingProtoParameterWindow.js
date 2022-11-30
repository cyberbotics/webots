import FloatingWindow from './FloatingWindow.js';
import {VRML} from './protoVisualizer/vrml_type.js';
import WbCamera from './nodes/WbCamera.js';
import WbHingeJoint from './nodes/WbHingeJoint.js';
import WbWorld from './nodes/WbWorld.js';

export default class FloatingProtoParameterWindow extends FloatingWindow {
  #protoManager;
  #view;
  constructor(parentNode, protoManager, view) {
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
    this.#view = view;

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
      this.#displayOptionalRendering();
    } else if (number === 1) {
      this.tab0.style.backgroundColor = '#333';
      this.tab1.style.backgroundColor = '#222';
      this.tab2.style.backgroundColor = '#333';
      this.frame.style.display = 'none';
      this.joints.style.display = 'block';
      this.devices.style.display = 'none';
      this.#displayOptionalRendering();
    } else if (number === 2) {
      this.tab0.style.backgroundColor = '#333';
      this.tab1.style.backgroundColor = '#333';
      this.tab2.style.backgroundColor = '#222';
      this.frame.style.display = 'none';
      this.joints.style.display = 'none';
      this.devices.style.display = 'block';
    }
  }

  populateProtoParameterWindow() {
    const contentDiv = document.getElementById('proto-parameter-content');
    if (contentDiv) {
      contentDiv.innerHTML = '';

      // populate the parameters
      const keys = this.#protoManager.exposedParameters.keys();
      let row = 1;
      for (let key of keys) {
        const parameter = this.#protoManager.exposedParameters.get(key);
        if (parameter.type === VRML.SFVec3f)
          this.#createSFVec3Field(key, contentDiv, row);
        else if (parameter.type === VRML.SFRotation)
          this.#createSFRotation(key, contentDiv, row);
        else if (parameter.type === VRML.SFString)
          this.#createSFStringField(key, contentDiv, row);
        else if (parameter.type === VRML.SFFloat)
          this.#createSFFloatField(key, contentDiv, row);
        else if (parameter.type === (VRML.SFInt32))
          this.#createSFInt32Field(key, contentDiv, row);
        else if (parameter.type === VRML.SFBool)
          this.#createSFBoolField(key, contentDiv, row);

        row++;
      }

      this.#createDownloadButton(contentDiv, row);
    }
  }

  #createDownloadButton(parent, row) {
    const downloadlButton = document.createElement('button');
    downloadlButton.innerHTML = 'Download';
    const buttonContainer = document.createElement('span');
    buttonContainer.className = 'value-parameter';
    buttonContainer.style.gridRow = '' + row + ' / ' + row;
    buttonContainer.style.gridColumn = '3 / 3';

    const input = document.createElement('input');
    input.type = 'text';
    input.title = 'New proto name';
    input.value = 'My' + this.#protoManager.proto.name;
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
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.key = key;
    p.inputs = [];
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    const exportCheckbox = this.#createCheckbox(parent, row);
    p.checkbox = exportCheckbox;

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
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.inputs = [];
    p.key = key;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    const exportCheckbox = this.#createCheckbox(parent, row);

    const values = document.createElement('p');
    values.style.gridRow = '' + row + ' / ' + row;
    values.style.gridColumn = '3 / 3';
    values.className = 'value-parameter';
    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' a', parameter.value.value.a, values, () => this.#rotationOnChange(p)));
    p.checkbox = exportCheckbox;
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
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.parameter = parameter;
    p.key = key;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.className = 'key-parameter';

    const exportCheckbox = this.#createCheckbox(parent, row);

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
    p.checkbox = exportCheckbox;
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
    input.step = 1;
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
    const nodes = WbWorld.instance.nodes;
    const keys = nodes.keys();
    let numberOfDevices = 0;

    for (const key of keys) {
      const device = nodes.get(key);
      if (device instanceof WbCamera) {
        numberOfDevices++;

        let div = document.createElement('div');
        div.className = 'proto-device';
        div.addEventListener('mouseover', () => this.#displayOptionalRendering(device.id));

        const nameDiv = document.createElement('div');
        nameDiv.innerHTML = this.#stringRemoveQuote(device.name);
        nameDiv.className = 'proto-device-name';
        div.appendChild(nameDiv);
        this.devices.appendChild(div);
      }
    }

    if (numberOfDevices === 0) {
      const noDevice = document.createElement('h1');
      noDevice.innerHTML = 'No devices';
      this.devices.appendChild(noDevice);
    }
  }

  #displayOptionalRendering(id) {
    if (WbWorld.instance.readyForUpdates) {
      let node = WbWorld.instance.nodes?.get(this.optionalDisplayId);
      if (node)
        node.applyOptionalRendering(false);

      node = WbWorld.instance.nodes?.get(id);
      if (node) {
        this.optionalDisplayId = id;
        node.applyOptionalRendering(true);
      }
      this.#view.x3dScene.render();
    }
  }

  populateJointTab() {
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
