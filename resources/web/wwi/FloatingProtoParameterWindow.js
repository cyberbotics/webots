import FloatingWindow from './FloatingWindow.js';
import {VRML} from './protoVisualizer/vrml_type.js';
import WbAccelerometer from './nodes/WbAccelerometer.js';
import WbAltimeter from './nodes/WbAltimeter.js';
import WbBallJoint from './nodes/WbBallJoint.js';
import WbCamera from './nodes/WbCamera.js';
import WbCharger from './nodes/WbCharger.js';
import WbCompass from './nodes/WbCompass.js';
import WbConnector from './nodes/WbConnector.js';
import WbDevice from './nodes/WbDevice.js';
import WbDisplay from './nodes/WbDisplay.js';
import WbDistanceSensor from './nodes/WbDistanceSensor.js';
import WbEmitter from './nodes/WbEmitter.js';
import WbGps from './nodes/WbGps.js';
import WbGyro from './nodes/WbGyro.js';
import WbHingeJoint from './nodes/WbHingeJoint.js';
import WbHinge2Joint from './nodes/WbHinge2Joint.js';
import WbInertialUnit from './nodes/WbInertialUnit.js';
import WbJoint from './nodes/WbJoint.js';
import WbLed from './nodes/WbLed.js';
import WbLidar from './nodes/WbLidar.js';
import WbLinearMotor from './nodes/WbLinearMotor.js';
import WbLightSensor from './nodes/WbLightSensor.js';
import WbMotor from './nodes/WbMotor.js';
import WbPen from './nodes/WbPen.js';
import WbRadar from './nodes/WbRadar.js';
import WbReceiver from './nodes/WbReceiver.js';
import WbRotationalMotor from './nodes/WbRotationalMotor.js';
import WbSpeaker from './nodes/WbSpeaker.js';
import WbWorld from './nodes/WbWorld.js';
import WbRangeFinder from './nodes/WbRangeFinder.js';
import WbTouchSensor from './nodes/WbTouchSensor.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbBrake from './nodes/WbBrake.js';
import WbPositionSensor from './nodes/WbPositionSensor.js';

export default class FloatingProtoParameterWindow extends FloatingWindow {
  #mfId;
  #protoManager;
  #rowNumber;
  #rowId;
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

    this.#mfId = 0;
    this.#rowId = 0;

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
    this.tab1.innerHTML = 'Joint devices';
    this.tab1.onclick = () => this.switchTab(1);
    infoTabsBar.appendChild(this.tab1);

    this.tab2 = document.createElement('button');
    this.tab2.className = 'proto-tab tab2';
    this.tab2.innerHTML = 'Other devices';
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
      this.#rowNumber = 1;
      for (let key of keys) {
        const parameter = this.#protoManager.exposedParameters.get(key);
        if (parameter.type === VRML.SFVec3f)
          this.#createSFVec3Field(key, contentDiv);
        else if (parameter.type === VRML.SFRotation)
          this.#createSFRotation(key, contentDiv);
        else if (parameter.type === VRML.SFString)
          this.#createSFStringField(key, contentDiv);
        else if (parameter.type === VRML.SFFloat)
          this.#createSFFloatField(key, contentDiv);
        else if (parameter.type === (VRML.SFInt32))
          this.#createSFInt32Field(key, contentDiv);
        else if (parameter.type === VRML.SFBool)
          this.#createSFBoolField(key, contentDiv);
        else if (parameter.type === VRML.MFVec3f)
          this.#createMFVec3fField(key, contentDiv);

        this.#rowNumber++;
      }

      this.#createDownloadButton(contentDiv);
    }
  }

  #disableResetButton(resetButton) {
    resetButton.style.filter = 'brightness(50%)';
    resetButton.style.pointerEvents = 'none';
  }

  #enableResetButton(resetButton) {
    resetButton.style.filter = 'brightness(100%)';
    resetButton.style.pointerEvents = 'all';
  }

  #createDownloadButton(parent) {
    const downloadlButton = document.createElement('button');
    downloadlButton.innerHTML = 'Download';
    const buttonContainer = document.createElement('span');
    buttonContainer.className = 'value-parameter';
    buttonContainer.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    buttonContainer.style.gridColumn = '4 / 4';

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

  #createSFVec3Field(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.key = key;
    p.inputs = [];
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    const exportCheckbox = this.#createCheckbox(parent);
    p.checkbox = exportCheckbox;

    const values = document.createElement('p');
    values.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    values.style.gridColumn = '4 / 4';
    values.className = 'value-parameter';

    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => {
      this.#vector3OnChange(p);
      this.#enableResetButton(resetButton);
    }));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, values, () => {
      this.#vector3OnChange(p);
      this.#enableResetButton(resetButton);
    }));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, values, () => {
      this.#vector3OnChange(p);
      this.#enableResetButton(resetButton);
    }));

    const resetButton = this.#createResetButton(parent, p.style.gridRow);
    this.#disableResetButton(resetButton);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      p.inputs[2].value = parameter.defaultValue.value.z;
      this.#vector3OnChange(p);
      this.#disableResetButton(resetButton);
    };

    parent.appendChild(p);
    parent.appendChild(values);
  }

  #createMFVec3fField(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);
    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.key = key;
    p.inputs = [];
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';
    p.className = 'key-parameter';

    const exportCheckbox = this.#createCheckbox(parent);
    p.checkbox = exportCheckbox;

    const hideShowButton = document.createElement('button');
    hideShowButton.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    hideShowButton.style.gridColumn = '4 / 4';
    hideShowButton.className = 'mf-expand-button';
    hideShowButton.title = 'Show content';
    hideShowButton.isHidden = true;

    const currentMfId = this.#mfId;
    hideShowButton.onclick = () => {
      const nodes = document.getElementsByClassName('mf-id-' + currentMfId);
      for (let i = 0; i < nodes.length; i++) {
        const sfVec3 = nodes[i];
        if (sfVec3) {
          if (sfVec3.style.display === 'block')
            sfVec3.style.display = 'none';
          else
            sfVec3.style.display = 'block';
        }
      }

      if (hideShowButton.isHidden) {
        hideShowButton.style.transform = 'rotate(90deg)';
        hideShowButton.isHidden = false;
        hideShowButton.title = 'Hide content';
      } else {
        hideShowButton.style.transform = '';
        hideShowButton.isHidden = true;
        hideShowButton.title = 'Show content';
      }
    };

    const resetButton = this.#createResetButton(parent, p.style.gridRow);
    this.#disableResetButton(resetButton);
    resetButton.onclick = () => {
      this.#disableResetButton(resetButton);
      const nodesToRemove = document.getElementsByClassName('mf-id-' + currentMfId);
      let maxRowNumber = 0;
      for (let i = nodesToRemove.length - 1; i >= 0; i--) {
        const rowNumber = this.#getRow(nodesToRemove[i]);
        if (rowNumber > maxRowNumber)
          maxRowNumber = rowNumber;
        nodesToRemove[i].parentNode.removeChild(nodesToRemove[i]);
      }

      parameter.value = parameter.defaultValue.clone();
      const resetButtonRow = this.#getRow(resetButton);
      // two times because of the `add` button and plus one for the first `add` button.
      const maxRowNumberNeeded = parameter.value.value.length * 2 + 1 + resetButtonRow;

      // Need to offset the following rows by the difference to keep the coherency.
      if (maxRowNumber > maxRowNumberNeeded)
        this.#offsetNegativelyRows(resetButtonRow, maxRowNumber - maxRowNumberNeeded);
      else if (maxRowNumber < maxRowNumberNeeded)
        this.#offsetPositivelyRows(resetButtonRow + 1, maxRowNumberNeeded - maxRowNumber);

      this.#populateMFVec3f(resetButton, parent, parameter, resetButtonRow, currentMfId, !hideShowButton.isHidden);

      this.#MFVec3fOnChange('value-parameter mf-parameter mf-id-' + currentMfId, parameter);
    };

    this.#rowNumber += this.#populateMFVec3f(resetButton, parent, parameter, this.#rowNumber, currentMfId);

    parent.appendChild(p);
    parent.appendChild(hideShowButton);

    this.#mfId++;
  }

  #populateMFVec3f(resetButton, parent, parameter, firstRow, mfId, isVisible) {
    this.#createAddRowSection(mfId, resetButton, firstRow, parent, parameter, isVisible);
    let numberOfRows = 1;
    for (let i = 0; i < parameter.value.value.length; i++) {
      numberOfRows++;
      this.createVector3Row(parameter.value.value[i].value, firstRow + numberOfRows, parent, mfId, resetButton,
        parameter, isVisible);
      numberOfRows++;
    }

    return numberOfRows;
  }

  createVector3Row(value, row, parent, mfId, resetButton, parameter, isVisible) {
    const p = document.createElement('p');
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '4 / 4';
    p.id = 'row-' + this.#rowId;
    p.className = 'value-parameter mf-parameter mf-id-' + mfId;
    if (isVisible)
      p.style.display = 'block';
    this.#createVectorInput(' x', value.x, p, () => {
      this.#MFVec3fOnChange(p.className, parameter);
      this.#enableResetButton(resetButton);
    });
    this.#createVectorInput(' y', value.y, p, () => {
      this.#MFVec3fOnChange(p.className, parameter);
      this.#enableResetButton(resetButton);
    });
    this.#createVectorInput(' z', value.z, p, () => {
      this.#MFVec3fOnChange(p.className, parameter);
      this.#enableResetButton(resetButton);
    });

    parent.appendChild(p);

    const removeButton = document.createElement('button');
    removeButton.className = 'remove-row-button';
    removeButton.title = 'Delete this row';
    removeButton.onclick = () => {
      const row = this.#getRow(removeButton.parentNode);
      this.#offsetNegativelyRows(row, 2);
      const className = removeButton.parentNode.className;
      removeButton.parentNode.parentNode.removeChild(removeButton.parentNode);

      // remove the 'add new node row'
      const addNew = document.getElementById(p.id);
      addNew.parentNode.removeChild(addNew);
      this.#enableResetButton(resetButton);
      this.#MFVec3fOnChange(className, parameter);
    };
    p.appendChild(removeButton);

    // Add row
    const addRow = this.#createAddRowSection(mfId, resetButton, row, parent, parameter, isVisible);
    return [p, addRow];
  }

  #offsetNegativelyRows(row, offset) {
    const grid = document.getElementById('proto-parameter-content');
    for (let i = 0; i < grid.childNodes.length; i++) {
      let node = grid.childNodes[i];
      const position = this.#getRow(node);
      if (position > row) {
        const newPosition = position - offset;
        node.style.gridRow = '' + newPosition + ' / ' + newPosition;
      }
    }
  }

  #offsetPositivelyRows(row, offset) {
    const grid = document.getElementById('proto-parameter-content');
    for (let i = 0; i < grid.childNodes.length; i++) {
      let node = grid.childNodes[i];
      const position = this.#getRow(node);
      if (position >= row) {
        const newPosition = position + offset;
        node.style.gridRow = '' + newPosition + ' / ' + newPosition;
      }
    }
  }

  #createAddRowSection(mfId, resetButton, row, parent, parameter, isVisible) {
    const addRow = document.createElement('button');
    addRow.onclick = () => {
      const row = this.#getRow(addRow) + 1;
      this.#offsetPositivelyRows(row, 2);

      const newRows = this.createVector3Row(new WbVector3(), row, parent, mfId, resetButton, parameter);
      newRows[0].style.display = 'block';
      newRows[1].style.display = 'block';
      this.#enableResetButton(resetButton);
      this.#MFVec3fOnChange(newRows[0].className, parameter);
    };

    addRow.style.gridColumn = '4 / 4';
    addRow.className = 'add-row mf-parameter mf-id-' + mfId;
    addRow.id = 'row-' + this.#rowId;
    addRow.title = 'Insert a new row here';
    if (isVisible)
      addRow.style.display = 'block';

    const rowNumber = row + 1;
    addRow.style.gridRow = '' + rowNumber + ' / ' + rowNumber;
    parent.appendChild(addRow);

    this.#rowId++;

    return addRow;
  }

  #getRow(node) {
    const nodeRow = node.style.gridRow;
    const slashIndex = nodeRow.indexOf('/');
    return parseInt(nodeRow.substring(0, slashIndex));
  }

  #MFVec3fOnChange(className, parameter) {
    const elements = document.getElementsByClassName(className);
    let lut = new Map();
    for (let i = 0; i < elements.length; i++) {
      let order = this.#getRow(elements[i]);

      // the last one is the delete button
      const value = new WbVector3(elements[i].childNodes[0].childNodes[1].value, elements[i].childNodes[1].childNodes[1].value,
        elements[i].childNodes[2].childNodes[1].value);

      lut.set(order, value);
    }
    lut = new Map([...lut.entries()].sort((a, b) => a[0] - b[0]));

    // Separately printing only keys
    const vectorArray = [];
    let i = 0;
    for (let value of lut.values()) {
      vectorArray[i] = value;
      i++;
    }

    parameter.setValueFromJavaScript(this.#view, vectorArray);
  }

  #createSFRotation(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.inputs = [];
    p.key = key;
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    const exportCheckbox = this.#createCheckbox(parent);

    const values = document.createElement('p');
    values.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    values.style.gridColumn = '4 / 4';
    values.className = 'value-parameter';
    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => {
      this.#rotationOnChange(p);
      this.#enableResetButton(resetButton);
    }));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, values, () => {
      this.#rotationOnChange(p);
      this.#enableResetButton(resetButton);
    }));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, values, () => {
      this.#rotationOnChange(p);
      this.#enableResetButton(resetButton);
    }));
    p.inputs.push(this.#createVectorInput(' a', parameter.value.value.a, values, () => {
      this.#rotationOnChange(p);
      this.#enableResetButton(resetButton);
    }));
    p.checkbox = exportCheckbox;
    const resetButton = this.#createResetButton(parent, p.style.gridRow);
    this.#disableResetButton(resetButton);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      p.inputs[2].value = parameter.defaultValue.value.z;
      p.inputs[3].value = parameter.defaultValue.value.a;
      this.#rotationOnChange(p);
      this.#disableResetButton(resetButton);
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

  #createSFStringField(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.parameter = parameter;
    p.key = key;
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';
    p.className = 'key-parameter';

    const exportCheckbox = this.#createCheckbox(parent);

    const value = document.createElement('p');
    value.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    value.style.gridColumn = '4 / 4';
    value.className = 'value-parameter';

    const input = document.createElement('input');
    input.type = 'text';

    const string = parameter.value.value;

    input.value = this.#stringRemoveQuote(string);
    input.style.height = '20px';
    value.appendChild(input);

    const resetButton = this.#createResetButton(parent, p.style.gridRow);
    this.#disableResetButton(resetButton);
    resetButton.onclick = () => {
      input.value = this.#stringRemoveQuote(parameter.defaultValue.value);
      this.#stringOnChange(p);
      this.#disableResetButton(resetButton);
    };

    input.onchange = () => {
      this.#stringOnChange(p);
      this.#enableResetButton(resetButton);
    };

    p.input = input;
    p.checkbox = exportCheckbox;

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

  #createSFFloatField(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.parameter = parameter;
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';

    const exportCheckbox = this.#createCheckbox(parent);

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    value.style.gridColumn = '4 / 4';

    const input = document.createElement('input');
    input.type = 'number';
    input.step = 0.1;
    input.value = parameter.value.value;
    input.style.width = '50px';

    input.onchange = () => {
      this.#enableResetButton(resetButton);
      this.#floatOnChange(p);
    };
    p.input = input;
    p.checkbox = exportCheckbox;
    value.appendChild(input);

    const resetButton = this.#createResetButton(parent, p.style.gridRow);
    this.#disableResetButton(resetButton);
    resetButton.onclick = () => {
      input.value = parameter.defaultValue.value;
      this.#floatOnChange(p);
      this.#disableResetButton(resetButton);
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  #floatOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.value);
  }

  #createSFInt32Field(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.parameter = parameter;
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';

    const exportCheckbox = this.#createCheckbox(parent, this.#rowNumber);

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    value.style.gridColumn = '4 / 4';

    const input = document.createElement('input');
    input.type = 'number';
    input.step = 1;
    input.value = parameter.value.value;
    input.style.width = '50px';

    input.oninput = () => this.#intOnChange(p);
    input.onchange = () => {
      this.#floatOnChange(p);
      this.#enableResetButton(resetButton);
    };
    p.input = input;
    p.checkbox = exportCheckbox;
    value.appendChild(input);

    const resetButton = this.#createResetButton(parent, p.style.gridRow);
    this.#disableResetButton(resetButton);
    resetButton.onclick = () => {
      input.value = parameter.defaultValue.value;
      this.#floatOnChange(p);
      this.#disableResetButton(resetButton);
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  #intOnChange(node) {
    node.input.value = node.input.value.replace(/[^0-9-]/g, '');
    // eslint-disable-next-line
    node.input.value = node.input.value.replace(/(\..*)\-/g, '$1');
  }

  #createSFBoolField(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.parameter = parameter;
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';

    const exportCheckbox = this.#createCheckbox(parent);

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    value.style.gridColumn = '4 / 4';

    const input = document.createElement('input');
    input.type = 'checkbox';
    input.checked = parameter.value.value;
    input.className = 'bool-field';

    p.input = input;
    p.checkbox = exportCheckbox;
    value.appendChild(input);

    const boolText = document.createElement('span');
    boolText.style.verticalAlign = 'middle';
    this.#changeBoolText(boolText, input);
    value.appendChild(boolText);

    input.onchange = () => {
      this.#boolOnChange(p);
      this.#enableResetButton(resetButton);
      this.#changeBoolText(boolText, input);
    };

    const resetButton = this.#createResetButton(parent, p.style.gridRow);
    this.#disableResetButton(resetButton);
    resetButton.onclick = () => {
      input.checked = parameter.defaultValue.value;
      this.#boolOnChange(p);
      this.#disableResetButton(resetButton);
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  #changeBoolText(boolText, input) {
    if (input.checked)
      boolText.innerHTML = 'TRUE';
    else
      boolText.innerHTML = 'FALSE';
  }

  #boolOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.checked);
  }

  #createResetButton(parentNode, row) {
    const resetButton = document.createElement('button');
    resetButton.className = 'reset-field-button';
    resetButton.title = 'Reset to initial value';
    resetButton.style.gridColumn = '3 / 3';
    resetButton.style.gridRow = row;

    parentNode.appendChild(resetButton);
    return resetButton;
  }

  #createCheckbox(parent) {
    const exportCheckbox = document.createElement('input');
    exportCheckbox.type = 'checkbox';
    exportCheckbox.className = 'export-checkbox';
    exportCheckbox.title = 'Field to be exposed';
    exportCheckbox.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    exportCheckbox.style.gridColumn = '1 / 1';
    parent.appendChild(exportCheckbox);

    return exportCheckbox;
  }

  populateDeviceTab() {
    this.devices.innerHTML = '';
    this.devicesList = this.#initDeviceList();
    const nodes = WbWorld.instance.nodes;
    const keys = nodes.keys();
    let numberOfDevices = 0;

    for (const key of keys) {
      const device = nodes.get(key);
      // TODO once all the optional rendering are implemented, replace this list by device instanceof WbDevice
      if (device instanceof WbDevice) {
        numberOfDevices++;

        let div = document.createElement('div');
        div.className = 'proto-device';
        div.addEventListener('mouseover', () => this.#displayOptionalRendering(device.id));
        div.addEventListener('mouseleave', () => this.#hideOptionalRendering(device.id));
        div.addEventListener('click', _ => this.#changeVisibility(device.id, _));
        const nameDiv = document.createElement('div');
        nameDiv.innerHTML = this.#stringRemoveQuote(device.name);
        nameDiv.className = 'proto-device-name';
        div.appendChild(nameDiv);

        this.#sortDevice(device, div);
      }
    }

    this.#appendDevices();

    if (numberOfDevices === 0) {
      const noDevice = document.createElement('h1');
      noDevice.innerHTML = 'No devices';
      this.devices.appendChild(noDevice);
    }
  }

  #initDeviceList() {
    const map = new Map();
    map.set('Accelerometer', []);
    map.set('Altimeter', []);
    map.set('Camera', []);
    map.set('Charger', []);
    map.set('Compass', []);
    map.set('Connector', []);
    map.set('Display', []);
    map.set('DistanceSensor', []);
    map.set('Emitter', []);
    map.set('Gps', []);
    map.set('Gyro', []);
    map.set('InertialUnit', []);
    map.set('Led', []);
    map.set('Lidar', []);
    map.set('LightSensor', []);
    map.set('Pen', []);
    map.set('Radar', []);
    map.set('Rangefinder', []);
    map.set('Receiver', []);
    map.set('Speaker', []);
    map.set('TouchSensor', []);

    return map;
  }

  #sortDevice(device, div) {
    if (device instanceof WbAccelerometer)
      this.devicesList.get('Accelerometer').push(div);
    else if (device instanceof WbAltimeter)
      this.devicesList.get('Altimeter').push(div);
    else if (device instanceof WbCamera)
      this.devicesList.get('Camera').push(div);
    else if (device instanceof WbCharger)
      this.devicesList.get('Charger').push(div);
    else if (device instanceof WbCompass)
      this.devicesList.get('Compass').push(div);
    else if (device instanceof WbConnector)
      this.devicesList.get('Connector').push(div);
    else if (device instanceof WbDisplay)
      this.devicesList.get('Display').push(div);
    else if (device instanceof WbDistanceSensor)
      this.devicesList.get('DistanceSensor').push(div);
    else if (device instanceof WbEmitter)
      this.devicesList.get('Emitter').push(div);
    else if (device instanceof WbGps)
      this.devicesList.get('Gps').push(div);
    else if (device instanceof WbGyro)
      this.devicesList.get('Gyro').push(div);
    else if (device instanceof WbInertialUnit)
      this.devicesList.get('InertialUnit').push(div);
    else if (device instanceof WbLed)
      this.devicesList.get('Led').push(div);
    else if (device instanceof WbLidar)
      this.devicesList.get('Lidar').push(div);
    else if (device instanceof WbLightSensor)
      this.devicesList.get('LightSensor').push(div);
    else if (device instanceof WbPen)
      this.devicesList.get('Pen').push(div);
    else if (device instanceof WbRadar)
      this.devicesList.get('Radar').push(div);
    else if (device instanceof WbRangeFinder)
      this.devicesList.get('Rangefinder').push(div);
    else if (device instanceof WbReceiver)
      this.devicesList.get('Receiver').push(div);
    else if (device instanceof WbSpeaker)
      this.devicesList.get('Speaker').push(div);
    else if (device instanceof WbTouchSensor)
      this.devicesList.get('TouchSensor').push(div);
  }

  #appendDevices() {
    let first = true;
    for (const key of this.devicesList.keys()) {
      const values = this.devicesList.get(key);
      if (values.length > 0) {
        const titleContainer = document.createElement('div');
        titleContainer.className = 'device-title-container';
        if (first) {
          first = false;
          titleContainer.style.paddingTop = '10px';
        }
        let hr = document.createElement('hr');
        titleContainer.appendChild(hr);
        const title = document.createElement('div');
        title.className = 'device-title';
        title.innerHTML = key;
        titleContainer.appendChild(title);
        hr = document.createElement('hr');
        titleContainer.appendChild(hr);
        this.devices.appendChild(titleContainer);
        for (const value of values)
          this.devices.appendChild(value);
      }
    }
  }

  #displayOptionalRendering(id) {
    if (WbWorld.instance.readyForUpdates) {
      const node = WbWorld.instance.nodes?.get(id);
      if (node)
        node.applyOptionalRendering(true);
      this.#view.x3dScene.render();
    }
  }

  #hideOptionalRendering(id) {
    const node = WbWorld.instance.nodes?.get(id);
    if (node) {
      if (node.optionnalRenderingLocked)
        return;
      node.applyOptionalRendering(false);
    }
    this.#view.x3dScene.render();
  }

  #changeVisibility(id, event) {
    const node = WbWorld.instance.nodes?.get(id);
    if (node) {
      node.optionnalRenderingLocked = !node.optionnalRenderingLocked;
      node.applyOptionalRendering(node.optionnalRenderingLocked);
      if (node.optionnalRenderingLocked)
        event.target.style.backgroundColor = '#007acc';
      else
        event.target.style.backgroundColor = '';
    }
    this.#view.x3dScene.render();
  }

  populateJointTab() {
    this.joints.innerHTML = '';
    const nodes = WbWorld.instance.nodes;
    const keys = nodes.keys();
    let numberOfJoint = 0;
    for (const key of keys) {
      const joint = nodes.get(key);
      if (joint instanceof WbJoint) {
        numberOfJoint++;

        let div = document.createElement('div');
        div.className = 'proto-joint';

        const endPointName = joint.endPoint ? joint.endPoint.name : numberOfJoint;
        let jointType;
        if (joint instanceof WbBallJoint)
          jointType = 'BallJoint 1: ';
        else if (joint instanceof WbHinge2Joint)
          jointType = 'Hinge2joint 1: ';
        else if (joint instanceof WbHingeJoint)
          jointType = 'Hingejoint: ';
        else
          jointType = 'Sliderjoint: ';

        div.appendChild(this.#createJointInfo(jointType, endPointName, joint.device));
        const parameters = joint.jointParameters;
        this.#createSlider(parameters, joint.device, div, _ => {
          if (parameters)
            parameters.position = _.target.value;
          else
            joint.position = _.target.value;
          this.#view.x3dScene.render();
        });

        this.joints.appendChild(div);

        if (joint instanceof WbBallJoint) {
          div = document.createElement('div');
          div.className = 'proto-joint';

          jointType = document.createElement('div');
          jointType.innerHTML = 'BallJoint 2: ' + endPointName;
          jointType.className = 'proto-joint-name';
          div.appendChild(jointType);
          const parameters2 = joint.jointParameters2;
          this.#createSlider(parameters2, joint.device2, div, _ => {
            if (parameters2)
              parameters2.position = _.target.value;
            else
              joint.position2 = _.target.value;
            this.#view.x3dScene.render();
          });
          this.joints.appendChild(div);

          jointType = document.createElement('div');
          jointType.innerHTML = 'BallJoint 3: ' + endPointName;
          jointType.className = 'proto-joint-name';
          div.appendChild(jointType);
          const parameters3 = joint.jointParameters3;
          this.#createSlider(parameters3, joint.device3, div, _ => {
            if (parameters3)
              parameters3.position = _.target.value;
            else
              joint.position3 = _.target.value;
            this.#view.x3dScene.render();
          });
          this.joints.appendChild(div);
        } else if (joint instanceof WbHinge2Joint) {
          div = document.createElement('div');
          div.className = 'proto-joint';

          div.appendChild(this.#createJointInfo('Hinge2joint 2: ', endPointName, joint.device2));
          const parameters2 = joint.jointParameters2;
          this.#createSlider(parameters2, joint.device2, div, _ => {
            if (parameters2)
              parameters2.position = _.target.value;
            else
              joint.position2 = _.target.value;
            this.#view.x3dScene.render();
          });
          this.joints.appendChild(div);
        }
      }
    }

    if (numberOfJoint === 0) {
      const title = document.createElement('h1');
      title.innerHTML = 'No joints';
      this.joints.appendChild(title);
    }
  }

  #createJointInfo(jointType, endPointName, devices) {
    const grid = document.createElement('div');
    grid.className = 'joint-details-grid';

    const jointTypeDiv = document.createElement('div');
    jointTypeDiv.className = 'proto-joint-name';
    jointTypeDiv.style.gridRow = '1 / 1';
    jointTypeDiv.style.gridColumn = '2 / 2';
    jointTypeDiv.innerHTML = jointType;
    grid.appendChild(jointTypeDiv);

    const endPointDiv = document.createElement('div');
    endPointDiv.className = 'proto-joint-name';
    endPointDiv.style.gridRow = '1 / 1';
    endPointDiv.style.gridColumn = '3 / 3';
    endPointDiv.innerHTML = this.#stringRemoveQuote(endPointName);
    grid.appendChild(endPointDiv);

    if (devices.length > 0) {
      const open = document.createElement('button');
      open.className = 'devices-button';
      open.isHidden = true;
      open.title = 'Show more information';
      grid.appendChild(open);

      for (let i = 0; i < devices.length; i++) {
        const row = i + 2;
        this.#createGridFiller(grid, row);

        const deviceType = document.createElement('div');
        if (devices[i] instanceof WbRotationalMotor)
          deviceType.innerHTML = 'RotationalMotor: ';
        else if (devices[i] instanceof WbLinearMotor)
          deviceType.innerHTML = 'LinearMotor: ';
        else if (devices[i] instanceof WbBrake)
          deviceType.innerHTML = 'Brake: ';
        else if (devices[i] instanceof WbPositionSensor)
          deviceType.innerHTML = 'PositionSensor:';

        deviceType.style.gridRow = row + ' / ' + row;
        deviceType.style.column = '2 / 2';
        deviceType.style.display = 'none';
        grid.appendChild(deviceType);

        const deviceName = document.createElement('div');
        deviceName.innerHTML = this.#stringRemoveQuote(devices[i].deviceName);
        deviceName.style.gridRow = row + ' / ' + row;
        deviceName.style.column = '3 / 3';
        deviceName.style.display = 'none';
        grid.appendChild(deviceName);
      }

      open.onclick = () => {
        const children = grid.childNodes;
        if (open.isHidden) {
          open.style.transform = 'rotate(90deg)';
          open.isHidden = false;
          open.title = 'Hide additional information';
          for (const child of children) {
            if (child.style.gridRow[0] > 1)
              child.style.display = 'block';
          }
        } else {
          open.style.transform = '';
          open.isHidden = true;
          open.title = 'Show more information';
          for (const child of children) {
            if (child.style.gridRow[0] > 1)
              child.style.display = 'none';
          }
        }
      };
    } else
      this.#createGridFiller(grid, 1);

    return grid;
  }

  #createGridFiller(grid, row) {
    const filler = document.createElement('div');
    filler.className = 'joint-grid-filler';
    filler.style.gridRow = row + ' / ' + row;
    filler.style.gridColumn = '1 / 1';
    grid.appendChild(filler);
    return filler;
  }

  #createSlider(parameters, device, parent, callback) {
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

    let motor;
    for (let i = 0; i < device.length; i++) {
      if (device[i] instanceof WbMotor) {
        motor = device[i];
        break;
      }
    }

    if (typeof motor !== 'undefined' && motor.minPosition !== motor.maxPosition) {
      minLabel.innerHTML = this.#decimalCount(motor.minPosition) > 2 ? motor.minPosition.toFixed(2) : motor.minPosition;
      slider.min = motor.minPosition;
      maxLabel.innerHTML = this.#decimalCount(motor.maxPosition) > 2 ? motor.maxPosition.toFixed(2) : motor.maxPosition;
      slider.max = motor.maxPosition;
    } else if (typeof parameters !== 'undefined' && parameters.minStop !== parameters.maxStop) {
      minLabel.innerHTML = this.#decimalCount(parameters.minStop) > 2 ? parameters.minStop.toFixed(2) : parameters.minStop;
      slider.min = parameters.minStop;
      maxLabel.innerHTML = this.#decimalCount(parameters.maxStop) > 2 ? parameters.maxStop.toFixed(2) : parameters.maxStop;
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

    slider.addEventListener('input', callback);

    sliderElement.appendChild(minLabel);
    sliderElement.appendChild(slider);
    sliderElement.appendChild(maxLabel);
    parent.appendChild(sliderElement);
  }

  #decimalCount(number) {
    const numberString = String(number);
    if (numberString.includes('.'))
      return numberString.split('.')[1].length;
    return 0;
  }
}
