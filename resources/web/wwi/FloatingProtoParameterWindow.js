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
import WbVacuumGripper from './nodes/WbVacuumGripper.js';
import WbBrake from './nodes/WbBrake.js';
import WbPositionSensor from './nodes/WbPositionSensor.js';
import NodeSelectorWindow from './NodeSelectorWindow.js';
import {SFNode, MFNode, vrmlFactory} from './protoVisualizer/Vrml.js';
import Node from './protoVisualizer/Node.js';
import WbPropeller from './nodes/WbPropeller.js';
import WbVector4 from './nodes/utils/WbVector4.js';

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
    this.node = protoManager.proto;
    this.headerText.innerHTML = this.node.name;
    this.#view = view;

    this.#mfId = 0;
    this.#rowId = 0;

    this.fieldsToExport = new Map();

    this.unsupportedRestrictions = [
      VRML.SFBool, VRML.SFNode, VRML.MFNode, VRML.MFBool,
      VRML.MFString, VRML.MFInt32, VRML.MFFloat, VRML.MFVec2f,
      VRML.MFVec3f, VRML.MFColor, VRML.MFRotation
    ];

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
      this.headerText.innerHTML = this.node.name;

      // populate the parameters based on the value of this.node (i.e. current active node)
      contentDiv.innerHTML = '';
      this.#rowNumber = 1;

      const keys = this.node.fieldsOrParameters().keys();
      for (let key of keys) {
        const parameter = this.node.fieldsOrParameters().get(key);
        if (parameter.hidden)
          continue;

        if (parameter.restrictions.length > 0 && !this.unsupportedRestrictions.includes(parameter.type))
          this.#createRestrictedField(key, contentDiv);
        else if (parameter.type === VRML.SFVec3f)
          this.#createSFVec3Field(key, contentDiv);
        else if (parameter.type === VRML.SFColor)
          this.#createSFColorField(key, contentDiv);
        else if (parameter.type === VRML.SFVec2f)
          this.#createSFVec2Field(key, contentDiv);
        else if (parameter.type === VRML.SFRotation)
          this.#createSFRotation(key, contentDiv);
        else if (parameter.type === VRML.SFString)
          this.#createSFStringField(key, contentDiv);
        else if (parameter.type === VRML.SFFloat)
          this.#createSFFloatField(key, contentDiv);
        else if (parameter.type === VRML.SFInt32)
          this.#createSFInt32Field(key, contentDiv);
        else if (parameter.type === VRML.SFBool)
          this.#createSFBoolField(key, contentDiv);
        else if (parameter.type === VRML.SFNode)
          this.#createSFNodeField(key, contentDiv);
        else if (parameter.type === VRML.MFNode)
          this.#createMFNodeField(key, contentDiv);
        else
          this.#createMFField(key, contentDiv);

        this.#rowNumber++;
      }

      if (this.node.isRoot) {
        this.#createDownloadButton(contentDiv);
        this.backBuffer = [];
      } else
        this.#createBackButton(contentDiv);

      this.#rowNumber++;
    }
  }

  #refreshParameterRow(parameter, mfId, isCreatingParameters) {
    const resetButton = document.getElementById('reset-' + parameter.name);
    if (!resetButton)
      return;

    if ((parameter.isTemplateRegenerator || parameter.value instanceof SFNode || parameter.value instanceof MFNode) &&
      !isCreatingParameters)
      this.updateDevicesTabs();

    if (parameter.isDefault())
      this.#disableResetButton(resetButton);
    else
      this.#enableResetButton(resetButton);

    if (parameter.value instanceof SFNode) {
      const currentNodeButton = document.getElementById('current-node-' + parameter.name);
      const deleteNodeButton = document.getElementById('delete-node-' + parameter.name);
      const configureNodeButton = document.getElementById('configure-node-' + parameter.name);

      if (parameter.value.value === null) {
        currentNodeButton.innerHTML = 'NULL';
        deleteNodeButton.style.display = 'none';
        configureNodeButton.style.display = 'none';
      } else {
        currentNodeButton.style.display = 'block';
        deleteNodeButton.style.display = 'block';
        configureNodeButton.style.display = 'block';
        configureNodeButton.title = 'Configure ' + parameter.value.value.name + ' node';
        currentNodeButton.innerHTML = parameter.value.value.name;
      }
    }

    if (parameter.value instanceof MFNode) {
      const nodes = document.getElementsByClassName('value-parameter mf-parameter mf-id-' + mfId);
      for (let i = 0; i < nodes.length; ++i) {
        const index = this.#rowToParameterIndex(nodes[i], mfId);
        const button = nodes[i].firstChild.firstChild;
        button.innerText = parameter.value.value[index].value.name;
      }
    }

    // note: SFNode/MFNode rows don't need refresh since the restriction and its handling is done in the node selector window
    if (parameter.restrictions.length > 0 && ![VRML.SFNode, VRML.MFNode].includes(parameter.type)) {
      const select = document.getElementById('select-' + parameter.name);
      for (const [i, restriction] of parameter.restrictions.entries()) {
        if (parameter.value && restriction.equals(parameter.value)) {
          select.options[i].selected = true;
          break;
        }
      }
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

  #createBackButton(parent) {
    const buttonContainer = document.createElement('span');
    buttonContainer.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    buttonContainer.style.gridColumn = '1 / 5';
    buttonContainer.style.justifySelf = 'center';
    buttonContainer.style.paddingTop = '15px';

    const backButton = document.createElement('button');
    backButton.innerHTML = 'Back';
    backButton.title = 'Return to the previous PROTO';
    backButton.onclick = () => {
      this.node = this.backBuffer.pop();
      this.populateProtoParameterWindow();
    };
    buttonContainer.appendChild(backButton);
    parent.appendChild(buttonContainer);
  }

  #createDownloadButton(parent) {
    const buttonContainer = document.createElement('span');
    buttonContainer.className = 'value-parameter';
    buttonContainer.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    buttonContainer.style.gridColumn = '4 / 4';

    if (typeof this.exportName === 'undefined')
      this.exportName = 'My' + this.#protoManager.proto.name;

    const input = document.createElement('input');
    input.type = 'text';
    input.title = 'New proto name';
    input.value = this.exportName;
    input.onchange = (e) => {
      this.exportName = e.target.value;
    };
    buttonContainer.appendChild(input);

    const downloadButton = document.createElement('button');
    downloadButton.innerHTML = 'Download';
    downloadButton.title = 'Download the new proto';
    downloadButton.onclick = () => {
      const data = this.#protoManager.exportProto(input.value, this.fieldsToExport);
      const downloadLink = document.createElement('a');
      downloadLink.download = this.exportName + '.proto';
      const file = new Blob([data], {
        type: 'text/plain'
      });
      downloadLink.href = window.URL.createObjectURL(file);
      downloadLink.click();
    };
    buttonContainer.appendChild(downloadButton);
    parent.appendChild(buttonContainer);
  }

  #createFieldCommonPart(key, parent) {
    const parameter = this.node.fieldsOrParameters().get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.key = key;
    p.inputs = [];
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    if (this.node.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, key);
      p.checkbox = exportCheckbox;
    } else
      p.style.marginLeft = '20px';

    return [p, parameter];
  }

  #createSFValue() {
    const value = document.createElement('p');
    value.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    value.style.gridColumn = '4 / 4';
    value.className = 'value-parameter';
    return value;
  };

  #createSFVec3Field(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const values = this.#createSFValue();

    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => this.#vector3OnChange(p), true));
    p.inputs.push(this.#createVectorInput('y', parameter.value.value.y, values, () => this.#vector3OnChange(p)));
    p.inputs.push(this.#createVectorInput('z', parameter.value.value.z, values, () => this.#vector3OnChange(p)));

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      p.inputs[2].value = parameter.defaultValue.value.z;
      this.#vector3OnChange(p);
    };

    parent.appendChild(p);
    parent.appendChild(values);

    this.#refreshParameterRow(parameter, undefined, true);
  }

  #createSFColorField(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const values = this.#createSFValue();

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      const defaultValue = parameter.defaultValue.value;
      input.value = this.rgbToHex(parseInt(defaultValue.r * 255), parseInt(initialValue.g * 255),
        parseInt(initialValue.b * 255));
      this.#colorOnChange(parameter, input.value);
    };

    const input = document.createElement('input');
    input.type = 'color';
    const initialValue = parameter.value.value;
    input.value = this.rgbToHex(parseInt(initialValue.r * 255), parseInt(initialValue.g * 255), parseInt(initialValue.b * 255));
    input.onchange = _ => this.#colorOnChange(parameter, _.target.value);
    values.appendChild(input);

    parent.appendChild(p);
    parent.appendChild(values);
  }

  #colorOnChange(parameter, hexValue) {
    const red = parseInt(hexValue.substring(1, 3), 16) / 255;
    const green = parseInt(hexValue.substring(3, 5), 16) / 255;
    const blue = parseInt(hexValue.substring(5, 7), 16) / 255;
    const newColor = {r: red, g: green, b: blue};
    parameter.setValueFromJavaScript(this.#view, newColor);
    this.#refreshParameterRow(parameter);
  }

  #colorComponentToHex(c) {
    const hex = c.toString(16);
    return hex.length === 1 ? '0' + hex : hex;
  }

  rgbToHex(r, g, b) {
    return '#' + this.#colorComponentToHex(r) + this.#colorComponentToHex(g) + this.#colorComponentToHex(b);
  }

  #createSFVec2Field(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const values = this.#createSFValue();

    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => this.#vector2OnChange(p), true));
    p.inputs.push(this.#createVectorInput('y', parameter.value.value.y, values, () => this.#vector2OnChange(p)));

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      this.#vector2OnChange(p);
    };

    parent.appendChild(p);
    parent.appendChild(values);
  }

  #vector2OnChange(node) {
    const object = {'x': this.#sanitizeNumber(node.inputs[0].value), 'y': this.#sanitizeNumber(node.inputs[1].value)};
    node.parameter.setValueFromJavaScript(this.#view, object);
    this.#refreshParameterRow(node.parameter);
  }

  #createMFField(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];

    const currentMfId = this.#mfId;
    const addButton = this.#createAddButtom(currentMfId);
    const hideShowButton = this.#createHideShowButtom(currentMfId, addButton);

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      const nodesToRemove = document.getElementsByClassName('mf-id-' + currentMfId);
      let maxRowNumber = 0;
      for (let i = nodesToRemove.length - 1; i >= 0; i--) {
        const rowNumber = this.#getRow(nodesToRemove[i]);
        if (rowNumber > maxRowNumber)
          maxRowNumber = rowNumber;
        nodesToRemove[i].parentNode.removeChild(nodesToRemove[i]);
      }

      const nodeModel = parameter.node.model;
      const parameterModel = nodeModel[parameter.node.isProto ? 'parameters' : 'fields'][parameter.name]['defaultValue'];
      parameter.value = vrmlFactory(parameter.type, parameterModel, true);
      const resetButtonRow = this.#getRow(resetButton);
      // two times because of the `add` button and plus one for the first `add` button.
      const maxRowNumberNeeded = parameter.value.value.length * 2 + 1 + resetButtonRow;

      // Need to offset the following rows by the difference to keep the coherency.
      if (maxRowNumber > maxRowNumberNeeded)
        this.#offsetNegativelyRows(resetButtonRow, maxRowNumber - maxRowNumberNeeded);
      else if (maxRowNumber < maxRowNumberNeeded)
        this.#offsetPositivelyRows(resetButtonRow + 1, maxRowNumberNeeded - maxRowNumber);

      this.#populateMFField(resetButton, parent, parameter, resetButtonRow, currentMfId, !hideShowButton.isHidden);

      this.#MFOnChange('value-parameter mf-parameter mf-id-' + currentMfId, parameter);
    };

    this.#rowNumber += this.#populateMFField(resetButton, parent, parameter, this.#rowNumber, currentMfId, false);

    parent.appendChild(p);
    parent.appendChild(hideShowButton);
    parent.appendChild(addButton);
    this.#mfId++;
    this.#refreshParameterRow(parameter, currentMfId, true);
  }

  #MFOnChange(className, parameter) {
    const elements = document.getElementsByClassName(className);
    let valuesMap = new Map();
    for (let i = 0; i < elements.length; i++) {
      const order = this.#getRow(elements[i]);

      let value;
      if (parameter.type === VRML.MFVec3f) {
        value = {x: this.#sanitizeNumber(elements[i].childNodes[0].childNodes[0].value),
          y: this.#sanitizeNumber(elements[i].childNodes[1].childNodes[0].value),
          z: this.#sanitizeNumber(elements[i].childNodes[2].childNodes[0].value)};
      } else if (parameter.type === VRML.MFVec2f) {
        value = {x: this.#sanitizeNumber(elements[i].childNodes[0].childNodes[0].value),
          y: this.#sanitizeNumber(elements[i].childNodes[1].childNodes[0].value)};
      } else if (parameter.type === VRML.MFString)
        value = elements[i].childNodes[0].value;
      else if (parameter.type === VRML.MFFloat || parameter.type === VRML.MFInt32)
        value = this.#sanitizeNumber(elements[i].childNodes[0].value);
      else if (parameter.type === VRML.MFBool)
        value = elements[i].childNodes[0].checked;
      else if (parameter.type === VRML.MFRotation) {
        value = {'x': this.#sanitizeNumber(elements[i].childNodes[0].childNodes[0].value),
          'y': this.#sanitizeNumber(elements[i].childNodes[1].childNodes[0].value),
          'z': this.#sanitizeNumber(elements[i].childNodes[2].childNodes[0].value),
          'a': this.#sanitizeNumber(elements[i].childNodes[3].childNodes[0].value)};
      } else if (parameter.type === VRML.MFColor) {
        const hexValue = elements[i].childNodes[0].value;
        const red = parseInt(hexValue.substring(1, 3), 16) / 255;
        const green = parseInt(hexValue.substring(3, 5), 16) / 255;
        const blue = parseInt(hexValue.substring(5, 7), 16) / 255;
        value = {r: red, g: green, b: blue};
      }

      valuesMap.set(order, value);
    }
    valuesMap = new Map([...valuesMap.entries()].sort((a, b) => a[0] - b[0]));

    // Separately printing only keys
    const array = [];
    let i = 0;
    for (let value of valuesMap.values())
      array[i++] = value;

    parameter.setValueFromJavaScript(this.#view, array);
    this.#refreshParameterRow(parameter);
  }

  #populateMFField(resetButton, parent, parameter, firstRow, mfId, isVisible) {
    this.#createAddRowSection(mfId, resetButton, firstRow, parent, parameter, isVisible);
    let numberOfRows = 1;
    for (let i = 0; i < parameter.value.value.length; i++) {
      numberOfRows++;

      if (parameter.type === VRML.MFNode) {
        this.#createMFNodeRow(parameter.value.value[i].value, firstRow + numberOfRows, parent, mfId, resetButton,
          parameter, isVisible);
      } else {
        this.#createMfRow(parameter.value.value[i].value, firstRow + numberOfRows, parent, mfId, resetButton,
          parameter, isVisible);
      }
      numberOfRows++;
    }

    return numberOfRows;
  }

  #createMfRow(value, row, parent, mfId, resetButton, parameter, isVisible) {
    const p = this.#createMfRowElement(row, mfId);

    if (isVisible)
      p.style.display = 'flex';

    if (parameter.type === VRML.MFFloat || parameter.type === VRML.MFInt32) {
      const input = document.createElement('input');
      input.type = 'number';
      input.value = value;

      if (parameter.type === VRML.MFInt32) {
        input.oninput = () => this.#intOnChange(input);
        input.step = 1;
      } else
        input.step = 0.1;

      input.onchange = () => this.#MFOnChange(p.className, parameter);

      input.style.width = '50px';
      p.appendChild(input);
    } else if (parameter.type === VRML.MFBool) {
      const input = document.createElement('input');
      input.type = 'checkbox';
      input.checked = value;
      input.className = 'bool-field';

      const boolText = document.createElement('span');
      boolText.style.verticalAlign = 'middle';
      this.#changeBoolText(boolText, input);

      input.onchange = () => {
        this.#MFOnChange(p.className, parameter);
        this.#changeBoolText(boolText, input);
      };

      p.appendChild(input);
      p.appendChild(boolText);
    } else if (parameter.type === VRML.MFVec2f) {
      this.#createVectorInput('x', value.x, p, () => this.#MFOnChange(p.className, parameter), true);
      this.#createVectorInput('y', value.y, p, () => this.#MFOnChange(p.className, parameter));
    } else if (parameter.type === VRML.MFVec3f) {
      this.#createVectorInput('x', value.x, p, () => this.#MFOnChange(p.className, parameter), true);
      this.#createVectorInput('y', value.y, p, () => this.#MFOnChange(p.className, parameter));
      this.#createVectorInput('z', value.z, p, () => this.#MFOnChange(p.className, parameter));
    } else if (parameter.type === VRML.MFRotation) {
      this.#createVectorInput('x', value.x, p, () => this.#MFOnChange(p.className, parameter), true);
      this.#createVectorInput('y', value.y, p, () => this.#MFOnChange(p.className, parameter));
      this.#createVectorInput('z', value.z, p, () => this.#MFOnChange(p.className, parameter));
      this.#createVectorInput('angle', value.a, p, () => this.#MFOnChange(p.className, parameter));
    } else if (parameter.type === VRML.MFColor) {
      const input = document.createElement('input');
      input.type = 'color';
      input.value = this.rgbToHex(parseInt(value.r * 255), parseInt(value.g * 255), parseInt(value.b * 255));
      input.onchange = _ => this.#MFOnChange(p.className, parameter);

      p.appendChild(input);
    } else if (parameter.type === VRML.MFString) {
      const input = document.createElement('input');
      input.type = 'text';
      input.onchange = () => this.#MFOnChange(p.className, parameter);
      input.value = this.#stringRemoveQuote(value);
      input.style.height = '20px';

      p.appendChild(input);
    }

    parent.appendChild(p);

    this.#createRemoveMFButton(p, () => this.#MFOnChange(p.className, parameter));

    // Add row
    const addRow = this.#createAddRowSection(mfId, resetButton, row, parent, parameter, isVisible);
    return [p, addRow];
  }

  #createRemoveMFButton(p, callback) {
    const removeButton = document.createElement('button');
    removeButton.className = 'remove-row-button';
    removeButton.title = 'Delete this row';
    removeButton.onclick = () => {
      const row = this.#getRow(removeButton.parentNode);
      this.#offsetNegativelyRows(row, 2);
      removeButton.parentNode.parentNode.removeChild(removeButton.parentNode);

      // remove the 'add new node row'
      const addNew = document.getElementById(p.id);
      addNew.parentNode.removeChild(addNew);
      if (typeof callback === 'function')
        callback();
    };
    p.appendChild(removeButton);
  }

  #createMFNodeField(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];

    const currentMfId = this.#mfId;
    const addButton = this.#createAddButtom(currentMfId);
    const hideShowButton = this.#createHideShowButtom(currentMfId, addButton);

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);

    resetButton.onclick = () => {
      // delete all existing rows in the interface
      const nodes = document.getElementsByClassName('mf-id-' + currentMfId);
      let maxRowNumber = 0;
      for (let i = nodes.length - 1; i >= 0; i--) {
        const rowNumber = this.#getRow(nodes[i]);
        if (rowNumber > maxRowNumber)
          maxRowNumber = rowNumber;
        nodes[i].parentNode.removeChild(nodes[i]);
      }

      // delete all existing nodes in the parameter
      for (let i = parameter.value.value.length - 1; i >= 0; --i)
        parameter.removeNode(this.#view, i);

      const protoModel = parameter.node.model;
      const parameterModel = protoModel[parameter.node.isProto ? 'parameters' : 'fields'][parameter.name]['defaultValue'];
      const mfnode = vrmlFactory(VRML.MFNode, parameterModel, true);
      const resetButtonRow = this.#getRow(resetButton);
      // two times because of the `add` button and plus one for the first `add` button.
      const maxRowNumberNeeded = mfnode.value.length * 2 + 1 + resetButtonRow;

      // Need to offset the following rows by the difference to keep the coherency.
      if (maxRowNumber > maxRowNumberNeeded)
        this.#offsetNegativelyRows(resetButtonRow, maxRowNumber - maxRowNumberNeeded);
      else if (maxRowNumber < maxRowNumberNeeded)
        this.#offsetPositivelyRows(resetButtonRow + 1, maxRowNumberNeeded - maxRowNumber);

      for (const [i, node] of mfnode.value.entries())
        parameter.insertNode(this.#view, node.value, i);

      this.#populateMFNode(resetButton, parent, parameter, resetButtonRow, currentMfId, false);
      this.#refreshParameterRow(parameter, currentMfId);
      hideShowButton.style.transform = '';
      hideShowButton.isHidden = true;
      hideShowButton.title = 'Show content';

      addButton.style = 'none';
    };

    this.#rowNumber += this.#populateMFNode(resetButton, parent, parameter, this.#rowNumber, currentMfId);

    parent.appendChild(p);
    parent.appendChild(hideShowButton);
    parent.appendChild(addButton);

    this.#mfId++;
    this.#refreshParameterRow(parameter, currentMfId, true);
  }

  #populateMFNode(resetButton, parent, parameter, firstRow, mfId, isVisible) {
    this.#createAddRowSection(mfId, resetButton, firstRow, parent, parameter, isVisible);
    let numberOfRows = 1;
    for (let i = 0; i < parameter.value.value.length; i++) {
      numberOfRows++;
      this.#createMFNodeRow(parameter.value.value[i].value, firstRow + numberOfRows, parent, mfId, resetButton,
        parameter, isVisible);
      numberOfRows++;
    }

    return numberOfRows;
  }

  #createMFNodeRow(value, row, parent, mfId, resetButton, parameter, isVisible) {
    const p = this.#createMfRowElement(row, mfId);

    if (isVisible)
      p.style.display = 'block';

    const buttonContainer = document.createElement('div');
    buttonContainer.style.display = 'flex';

    const currentNodeButton = document.createElement('button');
    currentNodeButton.className = 'sfnode-button';
    currentNodeButton.id = 'current-node-' + parameter.name;
    currentNodeButton.title = 'Select a node to insert';
    currentNodeButton.innerText = value.name;
    currentNodeButton.onclick = async() => {
      if (typeof this.nodeSelector === 'undefined')
        this.nodeSelector = new NodeSelectorWindow(this.parentNode, this.#protoManager.proto);

      this.nodeSelector.show(parameter, p, this.#MFNodeOnChange.bind(this), parent, mfId, resetButton);
      this.nodeSelectorListener = (event) => this.#hideNodeSelector(event);
      window.addEventListener('click', this.nodeSelectorListener, true);
    };

    const configureNodeButton = document.createElement('button');
    configureNodeButton.className = 'configure-button';
    configureNodeButton.id = 'configure-node-' + parameter.name;
    configureNodeButton.title = 'Edit node.';
    configureNodeButton.onclick = async() => {
      this.backBuffer.push(this.node);
      // determine node being selected among the list of nodes of the MF
      const index = this.#rowToParameterIndex(p, mfId);
      if (typeof index === 'undefined')
        throw new Error('The PROTO node to be configured is not defined, this should never be the case.');

      this.node = parameter.value.value[index].value;
      this.populateProtoParameterWindow();
    };

    buttonContainer.appendChild(currentNodeButton);
    buttonContainer.appendChild(configureNodeButton);

    p.appendChild(buttonContainer);

    parent.appendChild(p);

    this.#createRemoveMFButton(p, () => this.#MFNodeOnRemoval(parameter, undefined, p, undefined, mfId));

    // Add row
    const addRow = this.#createAddRowSection(mfId, resetButton, row, parent, parameter, isVisible);
    return [p, addRow];
  }

  async #MFNodeOnInsertion(parameter, url, element, parent, mfId, resetButton) {
    const index = this.#rowToParameterIndex(element, mfId);

    // generate node being inserted
    const node = await Node.createNode(url);
    parameter.insertNode(this.#view, node, index);

    const row = this.#getRow(element) + 1;
    this.#offsetPositivelyRows(row, 2);

    const newRows = this.#createMFNodeRow(node, row, parent, mfId, resetButton, parameter);
    newRows[0].style.display = 'flex';
    newRows[1].style.display = 'flex';

    this.#refreshParameterRow(parameter, mfId);
  }

  async #MFNodeOnRemoval(parameter, url, element, parent, mfId, resetButton) {
    const index = this.#rowToParameterIndex(element, mfId);

    // remove existing node
    parameter.removeNode(this.#view, index);

    this.#refreshParameterRow(parameter, mfId);
  }

  async #MFNodeOnChange(parameter, url, element, parent, mfId, resetButton) {
    const index = this.#rowToParameterIndex(element, mfId);

    // remove existing node
    parameter.removeNode(this.#view, index);
    // generate new node and insert it
    const node = await Node.createNode(url);
    parameter.insertNode(this.#view, node, index);

    this.#refreshParameterRow(parameter, mfId);
  }

  #rowToParameterIndex(element, mfId) {
    // since every MFNode parameter starts at a different grid-row, in order to determine the index from the
    // element rows (node buttons & add-row buttons), the value needs to be offset to account for it
    const nodes = document.getElementsByClassName('mf-parameter mf-id-' + mfId);
    let minRow = nodes.length > 0 ? Infinity : 0;
    for (let i = 0; i < nodes.length; ++i) {
      if (this.#getRow(nodes[i]) < minRow)
        minRow = this.#getRow(nodes[i]);
    }

    const row = this.#getRow(element) - minRow + 2;
    if (row % 2 === 0)
      return (row * 0.5) - 1;
    else
      return ((row - 1) * 0.5) - 1;
  }

  #createMfRowElement(row, mfId) {
    const p = document.createElement('p');
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '4 / 4';
    p.id = 'row-' + this.#rowId;
    p.className = 'value-parameter mf-parameter mf-id-' + mfId;

    return p;
  }

  #createHideShowButtom(currentMfId, addButton) {
    const hideShowButton = document.createElement('button');
    hideShowButton.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    hideShowButton.style.gridColumn = '4 / 4';
    hideShowButton.className = 'mf-expand-button';
    hideShowButton.title = 'Show content';
    hideShowButton.isHidden = true;

    hideShowButton.onclick = () => {
      const nodes = document.getElementsByClassName('mf-id-' + currentMfId);
      for (let i = 0; i < nodes.length; i++) {
        const element = nodes[i];
        if (element) {
          if (element.style.display === 'flex')
            element.style.display = 'none';
          else
            element.style.display = 'flex';
        }
      }

      if (hideShowButton.isHidden) {
        hideShowButton.style.transform = 'rotate(90deg)';
        hideShowButton.isHidden = false;
        hideShowButton.title = 'Hide content';
        addButton.style.display = 'block';
      } else {
        hideShowButton.style.transform = '';
        hideShowButton.isHidden = true;
        hideShowButton.title = 'Show content';
        addButton.style.display = 'none';
      }
    };

    return hideShowButton;
  }

  #createAddButtom(currentMfId, parameter) {
    const addButton = document.createElement('button');
    addButton.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    addButton.style.gridColumn = '4 / 4';
    addButton.className = 'mf-add-button';
    addButton.title = 'Add a new element at the end';
    addButton.innerText = '+';

    addButton.onclick = () => {
      const elements = document.getElementsByClassName('mf-id-' + currentMfId);
      let maxRow = 0;
      let lastAddRow;
      for (const item of elements) {
        let row = this.#getRow(item);
        if (maxRow < row) {
          maxRow = row;
          lastAddRow = item;
        }
      }
      lastAddRow.click();
    };

    return addButton;
  }

  #offsetNegativelyRows(row, offset) {
    const grid = document.getElementById('proto-parameter-content');
    for (let i = 0; i < grid.childNodes.length; i++) {
      const node = grid.childNodes[i];
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
      const node = grid.childNodes[i];
      const position = this.#getRow(node);
      if (position >= row) {
        const newPosition = position + offset;
        node.style.gridRow = '' + newPosition + ' / ' + newPosition;
      }
    }
  }

  #createAddRowSection(mfId, resetButton, row, parent, parameter, isVisible) {
    const addRow = document.createElement('button');
    addRow.onclick = async() => {
      if (parameter.type === VRML.MFNode) {
        if (typeof this.nodeSelector === 'undefined')
          this.nodeSelector = new NodeSelectorWindow(this.parentNode, this.#protoManager.proto);

        this.nodeSelector.show(parameter, addRow, this.#MFNodeOnInsertion.bind(this), parent, mfId, resetButton);
        this.nodeSelectorListener = (event) => this.#hideNodeSelector(event);
        window.addEventListener('click', this.nodeSelectorListener, true);
      } else {
        const row = this.#getRow(addRow) + 1;
        this.#offsetPositivelyRows(row, 2);
        let defaultValue;
        if (parameter.type === VRML.MFVec3f)
          defaultValue = {x: 0, y: 0, z: 0};
        else if (parameter.type === VRML.MFVec2f)
          defaultValue = {x: 0, y: 0};
        else if (parameter.type === VRML.MFString)
          defaultValue = '';
        else if (parameter.type === VRML.MFFloat || parameter.type === VRML.MFInt32)
          defaultValue = 0;
        else if (parameter.type === VRML.MFBool)
          defaultValue = false;
        else if (parameter.type === VRML.MFRotation)
          defaultValue = {x: 0, y: 0, z: 1, a: 0};
        else if (parameter.type === VRML.MFColor)
          defaultValue = {r: 0, g: 0, b: 0};

        const newRows = this.#createMfRow(defaultValue, row, parent, mfId, resetButton, parameter, true);
        this.#MFOnChange(newRows[0].className, parameter);

        newRows[0].style.display = 'flex';
        newRows[1].style.display = 'flex';
      }
    };

    addRow.style.gridColumn = '4 / 4';
    addRow.className = 'add-row mf-parameter mf-id-' + mfId;
    addRow.id = 'row-' + this.#rowId;
    addRow.title = 'Insert a new row here';
    if (isVisible)
      addRow.style.display = 'flex';

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

  #createSFRotation(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const values = this.#createSFValue();

    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => this.#rotationOnChange(p), true));
    p.inputs.push(this.#createVectorInput('y', parameter.value.value.y, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput('z', parameter.value.value.z, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput('a', parameter.value.value.a, values, () => this.#rotationOnChange(p)));

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      p.inputs[2].value = parameter.defaultValue.value.z;
      p.inputs[3].value = parameter.defaultValue.value.a;
      this.#rotationOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(values);

    this.#refreshParameterRow(parameter, undefined, true);
  }

  #createVectorInput(name, initialValue, parent, callback, first) {
    const span = document.createElement('span');
    span.title = name;
    if (!first)
      span.style.paddingLeft = '5px';
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
    const object = {
      'x': this.#sanitizeNumber(node.inputs[0].value),
      'y': this.#sanitizeNumber(node.inputs[1].value),
      'z': this.#sanitizeNumber(node.inputs[2].value),
      'a': this.#sanitizeNumber(node.inputs[3].value)
    };
    node.parameter.setValueFromJavaScript(this.#view, object);
    this.#refreshParameterRow(node.parameter);
  }

  #vector3OnChange(node) {
    const object = {
      'x': this.#sanitizeNumber(node.inputs[0].value),
      'y': this.#sanitizeNumber(node.inputs[1].value),
      'z': this.#sanitizeNumber(node.inputs[2].value)
    };
    node.parameter.setValueFromJavaScript(this.#view, object);
    this.#refreshParameterRow(node.parameter);
  }

  #createRestrictedField(key, parent) {
    const parameter = this.node.fieldsOrParameters().get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.parameter = parameter;
    p.key = key;
    p.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    p.style.gridColumn = '2 / 2';
    p.className = 'key-parameter';

    if (this.node.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, key);
      p.checkbox = exportCheckbox;
    } else
      p.style.marginLeft = '20px';

    const value = document.createElement('p');
    value.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    value.style.gridColumn = '4 / 4';
    value.className = 'value-parameter';

    const select = document.createElement('select');
    select.id = 'select-' + parameter.name;
    select.parameter = parameter;

    for (const item of parameter.restrictions) {
      let value;
      switch (parameter.type) {
        case VRML.SFString:
          value = this.#stringRemoveQuote(item.value);
          break;
        case VRML.SFFloat:
        case VRML.SFInt32:
          value = item.value;
          break;
        case VRML.SFVec2f:
        case VRML.SFVec3f:
        case VRML.SFColor:
        case VRML.SFRotation:
          value = item.toVrml(); // does what is needed, useless to create another ad-hoc method
          break;
        default:
          throw new Error('Unsupported parameter type: ', parameter.type);
      }

      const option = document.createElement('option');
      option.value = value;
      option.innerText = value;
      if (parameter.value && item.equals(parameter.value))
        option.selected = true;

      select.appendChild(option);
    }

    select.onchange = (e) => {
      const parameter = e.target.parameter;
      const selectionIndex = e.target.selectedIndex;
      parameter.setValueFromJavaScript(this.#view, parameter.restrictions[selectionIndex].toJS(false));
      this.#refreshParameterRow(parameter);
    };
    value.appendChild(select);
    p.input = select;

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    this.#disableResetButton(resetButton);
    resetButton.onclick = () => {
      // we can use stringify because SFNodes/MFNodes are handled separately (through the node selection window)
      parameter.setValueFromJavaScript(this.#view, JSON.parse(JSON.stringify(parameter.defaultValue.value)));
      this.#refreshParameterRow(parameter);
    };

    parent.appendChild(p);
    parent.appendChild(value);

    this.#refreshParameterRow(parameter, undefined, true);
  }

  #createSFStringField(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const value = this.#createSFValue();

    const input = document.createElement('input');
    input.type = 'text';

    const string = parameter.value.value;

    input.value = this.#stringRemoveQuote(string);
    input.style.height = '20px';
    value.appendChild(input);

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      input.value = this.#stringRemoveQuote(parameter.defaultValue.value);
      this.#stringOnChange(p);
    };

    input.onchange = () => this.#stringOnChange(p);

    p.input = input;

    parent.appendChild(p);
    parent.appendChild(value);

    this.#refreshParameterRow(parameter, undefined, true);
  }

  #stringOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.value);
    this.#refreshParameterRow(node.parameter);
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
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const value = this.#createSFValue();

    const input = document.createElement('input');
    input.type = 'number';
    input.step = 0.1;
    input.value = parameter.value.value;
    input.style.width = '50px';

    input.onchange = () => this.#floatOnChange(p);
    p.input = input;
    value.appendChild(input);

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      input.value = parameter.defaultValue.value;
      this.#floatOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(value);

    this.#refreshParameterRow(parameter, undefined, true);
  }

  #createSFNodeField(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const value = this.#createSFValue();

    const buttonContainer = document.createElement('div');
    buttonContainer.style.display = 'flex';

    const currentNodeButton = document.createElement('button');
    currentNodeButton.className = 'sfnode-button';
    currentNodeButton.id = 'current-node-' + parameter.name;
    currentNodeButton.title = 'Select a node to insert';
    currentNodeButton.onclick = async() => {
      if (typeof this.nodeSelector === 'undefined')
        this.nodeSelector = new NodeSelectorWindow(this.parentNode, this.#protoManager.proto);

      this.nodeSelector.show(parameter, p, this.#sfnodeOnChange.bind(this));
      this.nodeSelectorListener = (event) => this.#hideNodeSelector(event);
      window.addEventListener('click', this.nodeSelectorListener, true);
    };

    const deleteNodeButton = document.createElement('button');
    deleteNodeButton.className = 'delete-button';
    deleteNodeButton.id = 'delete-node-' + parameter.name;
    deleteNodeButton.title = 'Remove node.';
    deleteNodeButton.onclick = () => {
      parameter.setValueFromJavaScript(this.#view, null);
      this.#refreshParameterRow(parameter);
    };

    const configureNodeButton = document.createElement('button');
    configureNodeButton.className = 'configure-button';
    configureNodeButton.id = 'configure-node-' + parameter.name;
    configureNodeButton.title = 'Edit node.';
    configureNodeButton.onclick = async() => {
      if (parameter.value.value === null)
        return;

      this.backBuffer.push(this.node);
      this.node = parameter.value.value;
      this.populateProtoParameterWindow();
    };

    buttonContainer.appendChild(currentNodeButton);
    buttonContainer.appendChild(configureNodeButton);
    buttonContainer.appendChild(deleteNodeButton);
    value.append(buttonContainer);

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      if (parameter.defaultValue.value === null)
        parameter.setValueFromJavaScript(this.#view, null);
      else {
        // note: in order to have a node instance for "value" that is independent from that of "defaultValue",
        // a new node needs to be created. However, it cannot be created from the model of the node itself
        // (in cProtoModels), as the PROTO that references this node might be setting extra parameters, like:
        // -- field SFNode appearance Leather { textureTransform TextureTransform { scale 10 10 } }
        // this information is however only available on the parent PROTO side, hence why the parameter
        // model needs to be retrieved and used as initialization when creating the node instance (i.e. before
        // the internal body is created as these extra parameters might yield different results)
        const nodeModel = parameter.node.model;
        const model = nodeModel[parameter.node.isProto ? 'parameters' : 'fields'][parameter.name]['defaultValue'];
        const sfnode = vrmlFactory(VRML.SFNode, model, true);
        parameter.setValueFromJavaScript(this.#view, sfnode.value);
      }
      this.#refreshParameterRow(parameter);
    };

    parent.appendChild(p);
    parent.appendChild(value);

    this.#refreshParameterRow(parameter, undefined, true);
  }

  async #sfnodeOnChange(parameter, url) {
    const node = await Node.createNode(url);
    parameter.setValueFromJavaScript(this.#view, node);
    this.#refreshParameterRow(parameter);
  }

  #hideNodeSelector(event) {
    if (typeof this.nodeSelector !== 'undefined' && !this.nodeSelector.nodeSelector.contains(event.target)) {
      this.nodeSelector.hide();
      window.removeEventListener('click', this.nodeSelectorListener, true);
    }
  }

  #floatOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, this.#sanitizeNumber(node.input.value));
    this.#refreshParameterRow(node.parameter);
  }

  #createSFInt32Field(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const value = this.#createSFValue();

    const input = document.createElement('input');
    input.type = 'number';
    input.step = 1;
    input.value = parameter.value.value;
    input.style.width = '50px';

    input.oninput = () => this.#intOnChange(input);
    input.onchange = () => this.#floatOnChange(p);

    p.input = input;
    value.appendChild(input);

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      input.value = parameter.defaultValue.value;
      this.#floatOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(value);

    this.#refreshParameterRow(parameter, undefined, true);
  }

  #intOnChange(input) {
    input.value = input.value.replace(/[^0-9-]/g, '');
    // eslint-disable-next-line
    input.value = input.value.replace(/(\..*)\-/g, '$1');
  }

  #createSFBoolField(key, parent) {
    const results = this.#createFieldCommonPart(key, parent);
    const p = results[0];
    const parameter = results[1];
    const value = this.#createSFValue();

    const input = document.createElement('input');
    input.type = 'checkbox';
    input.checked = parameter.value.value;
    input.className = 'bool-field';

    p.input = input;
    value.appendChild(input);

    const boolText = document.createElement('span');
    boolText.style.verticalAlign = 'middle';
    this.#changeBoolText(boolText, input);
    value.appendChild(boolText);

    input.onchange = () => {
      this.#changeBoolText(boolText, input);
      this.#boolOnChange(p);
    };

    const resetButton = this.#createResetButton(parent, p.style.gridRow, parameter.name);
    resetButton.onclick = () => {
      input.checked = parameter.defaultValue.value;
      this.#boolOnChange(p);
      this.#changeBoolText(boolText, input);
    };
    parent.appendChild(p);
    parent.appendChild(value);

    this.#refreshParameterRow(parameter, undefined, true);
  }

  #changeBoolText(boolText, input) {
    if (input.checked)
      boolText.innerHTML = 'TRUE';
    else
      boolText.innerHTML = 'FALSE';
  }

  #boolOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.checked);
    this.#refreshParameterRow(node.parameter);
  }

  #createResetButton(parentNode, row, id) {
    const resetButton = document.createElement('button');
    resetButton.className = 'reset-field-button';
    resetButton.id = 'reset-' + id;
    resetButton.title = 'Reset to initial value';
    resetButton.style.gridColumn = '3 / 3';
    resetButton.style.gridRow = row;

    parentNode.appendChild(resetButton);
    return resetButton;
  }

  #createCheckbox(parent, key) {
    const exportCheckbox = document.createElement('input');
    exportCheckbox.type = 'checkbox';
    exportCheckbox.className = 'export-checkbox';
    exportCheckbox.title = 'Field to be exposed';
    exportCheckbox.key = key;
    exportCheckbox.style.gridRow = '' + this.#rowNumber + ' / ' + this.#rowNumber;
    exportCheckbox.style.gridColumn = '1 / 1';
    exportCheckbox.onchange = (event) => this.fieldsToExport.set(exportCheckbox.key, exportCheckbox.checked);
    if (this.fieldsToExport.has(key))
      exportCheckbox.checked = this.fieldsToExport.get(key);

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
      if (device instanceof WbDevice) {
        numberOfDevices++;

        const div = document.createElement('div');
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

    this.tab2.style.display = (numberOfDevices === 0) ? 'none' : 'block';
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
    map.set('VacuumGripper', []);

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
    else if (device instanceof WbVacuumGripper)
      this.devicesList.get('VacuumGripper').push(div);
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
    this.coupledSlidersList = [];
    const nodes = WbWorld.instance.nodes;
    const keys = nodes.keys();
    let numberOfJoint = 0;
    for (const key of keys) {
      const joint = nodes.get(key);
      if (joint instanceof WbJoint) {
        numberOfJoint++;

        let div = document.createElement('div');
        div.className = 'proto-joint';

        const endPointName = joint.solidEndPoint() ? joint.solidEndPoint().name : numberOfJoint;
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
        this.#createSlider(parameters, joint.device, div, value => {
          if (parameters)
            parameters.position = value;
          else
            joint.position = value;
          this.#view.x3dScene.render();
        });

        this.joints.appendChild(div);

        if (joint instanceof WbBallJoint) {
          div = document.createElement('div');
          div.className = 'proto-joint';

          div.appendChild(this.#createJointInfo('BallJoint 2: ', endPointName, joint.device2));
          const parameters2 = joint.jointParameters2;
          this.#createSlider(parameters2, joint.device2, div, value => {
            if (parameters2)
              parameters2.position = value;
            else
              joint.position2 = value;
            this.#view.x3dScene.render();
          });
          this.joints.appendChild(div);

          div.appendChild(this.#createJointInfo('BallJoint 3: ', endPointName, joint.device3));
          const parameters3 = joint.jointParameters3;
          this.#createSlider(parameters3, joint.device3, div, value => {
            if (parameters3)
              parameters3.position = value;
            else
              joint.position3 = value;
            this.#view.x3dScene.render();
          });
          this.joints.appendChild(div);
        } else if (joint instanceof WbHinge2Joint) {
          div = document.createElement('div');
          div.className = 'proto-joint';

          div.appendChild(this.#createJointInfo('Hinge2joint 2: ', endPointName, joint.device2));
          const parameters2 = joint.jointParameters2;
          this.#createSlider(parameters2, joint.device2, div, value => {
            if (parameters2)
              parameters2.position = value;
            else
              joint.position2 = value;
            this.#view.x3dScene.render();
          });
          this.joints.appendChild(div);
        }
      } else if (joint instanceof WbPropeller) {
        numberOfJoint++;

        let div = document.createElement('div');
        div.className = 'proto-joint';

        let helixName;
        let helix;
        if (joint.children.length > 1) {
          helixName = joint.children[1].name; // slow helix
          helix = joint.children[1];
        } else if (joint.children.length === 1) {
          helixName = joint.children[0].name;
          helix = joint.children[0];
        } else
          helixName = 'propeller';

        div.appendChild(this.#createJointInfo('Propeller: ', helixName, joint.device));
        if (helix) {
          this.#createSlider(undefined, joint.device, div, value => {
            helix.rotation = new WbVector4(helix.rotation.x, helix.rotation.y, helix.rotation.z, value);
            this.#view.x3dScene.render();
          });
        }
        this.joints.appendChild(div);
      }
    }

    if (numberOfJoint === 0) {
      this.tab1.style.display = 'none';
      this.tab2.style.left = '33%';
      this.tab2.textContent = 'Devices';
    } else {
      this.tab1.style.display = 'block';
      this.tab2.style.left = '66%';
      this.tab2.textContent = 'Other devices';
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
              child.style.display = 'flex';
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

    slider.addEventListener('input', _ => {
      if (typeof callback === 'function')
        callback(_.target.value);

      if (slider.motorName?.includes('::')) {
        const coupledMotorName = slider.motorName.split('::')[0];
        for (let i = 0; i < this.coupledSlidersList.length; i++) {
          if (this.coupledSlidersList[i].motorName.startsWith(coupledMotorName)) {
            const newValue = slider.value * this.coupledSlidersList[i].multiplier / slider.multiplier;
            this.coupledSlidersList[i].value = newValue;
            this.coupledSlidersList[i].callback(newValue);
          }
        }
      }
    });

    sliderElement.appendChild(minLabel);
    sliderElement.appendChild(slider);
    sliderElement.appendChild(maxLabel);

    if (typeof motor !== 'undefined') {
      if (motor.deviceName.includes('::')) {
        slider.motorName = motor.deviceName;
        slider.callback = callback;
        slider.multiplier = motor.multiplier;

        this.coupledSlidersList.push(slider);
      }
    }
    parent.appendChild(sliderElement);
  }

  #decimalCount(number) {
    const numberString = String(number);
    if (numberString.includes('.'))
      return numberString.split('.')[1].length;
    return 0;
  }

  #sanitizeNumber(numberString) {
    const number = parseFloat(numberString);
    if (isNaN(number))
      return 0;
    return number;
  }

  updateDevicesTabs() {
    this.populateJointTab();
    this.populateDeviceTab();
  }
}
