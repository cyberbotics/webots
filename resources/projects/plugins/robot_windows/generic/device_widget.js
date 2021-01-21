/* global appendNewElement: false */
/* global TimeplotWidget: false */
/* global PlotWidget: false */
/* global RadarWidget: false */

function DeviceWidget(basicTimeStep, device) {
  this.device = device;
  this.basicTimeStep = basicTimeStep;
  this.plots = null;
  this.image = null;
  this.firstUpdate = true;
  this.initialize(device);
}

DeviceWidget.widgets = {}; // Dictionary {deviceName -> DeviceWidget }
DeviceWidget.commands = []; // Commands to be sent to the C library.
DeviceWidget.motorWidgets = {}; // Dictionary {deviceName -> DeviceWidget }
DeviceWidget.motorCommands = []; // Motor commands to be sent to the C library.
DeviceWidget.touchedCheckboxes = [];
DeviceWidget.supportedDeviceTypes = [
  'Accelerometer', 'Camera', 'Compass', 'DistanceSensor', 'GPS', 'Gyro',
  'InertialUnit', 'Lidar', 'LightSensor', 'LinearMotor', 'PositionSensor',
  'Radar', 'RangeFinder', 'RotationalMotor', 'TouchSensor'
];

DeviceWidget.prototype.initialize = function(device) {
  if (document.getElementById(device.htmlName))
    return; // check if already exists

  let div = '<div id="' + device.htmlName + '" class="device">';
  div += '<h2>';
  if (device.type !== 'DifferentialWheels')
    div += '<input type="checkbox" title="Enable/disable this device." id="' + device.htmlName + '-enable-checkbox" device="' + device.htmlName + '" onclick="DeviceWidget.checkboxCallback(this)" />';
  div += device.htmlName + '<span id="' + device.htmlName + '-label"></span></h2>';
  if (device.type === 'Camera' && device.recognition === 1) {
    if (device.segmentation === 1) {
      div += '<div class="device-option-enable-label">';
      div += '<h2 class="device-option-item-enable-label">';
    } else
      div += '<h2 class="device-option-enable-label">';
    div += '<input type="checkbox" title="Enable/disable the recognition functionality." id="' + device.htmlName + '-recognition-checkbox" device="' + device.htmlName + '" onclick="cameraRecognitionCheckboxCallback(this)"/>';
    div += '<span>Recognition</span>';
    div += '</h2>';
    if (device.segmentation === 1) {
      div += '<h2 class="device-option-item-enable-label">';
      div += '<input type="checkbox" title="Enable/disable the segmentation functionality." id="' + device.htmlName + '-segmentation-checkbox" device="' + device.htmlName + '" onclick="cameraSegmentationCheckboxCallback(this)" disabled/>';
      div += '<span>Segmentation</span>';
      div += '</h2>';
      div += '</div>';
    }
  } else if (device.type === 'Lidar') {
    div += '<h2 class="device-option-enable-label">';
    div += '<input type="checkbox" title="Enable/disable the lidar point cloud." id="' + device.htmlName + '-cloud-point-checkbox" device="' + device.htmlName + '" onclick="pointCloudCheckboxCallback(this)"/>';
    div += '<span>Point Cloud</span>';
    div += '</h2>';
  }
  div += '<div id="' + device.htmlName + '-content" class="device-content"/>';
  div += '</div>';

  appendNewElement(device.type + '-layout', div);

  if (device.type === 'Accelerometer')
    this.createGeneric3DDevice(device, TimeplotWidget.prototype.AutoRangeType.STRETCH, -20.0, 20.0, '[m/s^2]');
  else if (device.type === 'Camera')
    this.createGenericImageDevice(device);
  else if (device.type === 'Compass')
    this.createGeneric3DDevice(device, TimeplotWidget.prototype.AutoRangeType.NONE, -1.0, 1.0, '');
  else if (device.type === 'DifferentialWheels')
    this.createDifferentialWheels(device);
  else if (device.type === 'DistanceSensor')
    this.createGeneric1DDevice(device, TimeplotWidget.prototype.AutoRangeType.NONE, device.minValue, device.maxValue, 'Raw');
  else if (device.type === 'GPS')
    this.createGeneric3DDevice(device, TimeplotWidget.prototype.AutoRangeType.STRETCH, -1.0, 1.0, '[m]');
  else if (device.type === 'Gyro')
    this.createGeneric3DDevice(device, TimeplotWidget.prototype.AutoRangeType.STRETCH, -20.0, 20.0, '[rad/s]');
  else if (device.type === 'InertialUnit')
    this.createGeneric3DDevice(device, TimeplotWidget.prototype.AutoRangeType.NONE, -Math.PI, Math.PI, '[rad]');
  else if (device.type === 'Lidar')
    this.createGenericImageDevice(device);
  else if (device.type === 'LightSensor')
    this.createGeneric1DDevice(device, TimeplotWidget.prototype.AutoRangeType.STRETCH, 0, 1024, 'Raw');
  else if (device.type === 'PositionSensor')
    this.createGeneric1DDevice(device, TimeplotWidget.prototype.AutoRangeType.JUMP, -Math.PI, Math.PI, 'Angle [Rad]');
  else if (device.type === 'Radar')
    this.createRadar(device);
  else if (device.type === 'RangeFinder')
    this.createGenericImageDevice(device);
  else if (device.type === 'RotationalMotor' || device.type === 'LinearMotor') {
    let minPosition = device.minPosition;
    let maxPosition = device.maxPosition;
    let autoRange = TimeplotWidget.prototype.AutoRangeType.NONE;
    if (minPosition === 0.0 && maxPosition === 0.0) {
      minPosition = device.type === 'RotationalMotor' ? -Math.PI : -1.0;
      maxPosition = device.type === 'RotationalMotor' ? Math.PI : 1.0;
      autoRange = device.type === 'RotationalMotor' ? TimeplotWidget.prototype.AutoRangeType.JUMP : TimeplotWidget.prototype.AutoRangeType.STRETCH;
    }
    this.createMotor(device, autoRange, minPosition, maxPosition, device.type === 'RotationalMotor' ? 'Angle [rad]' : 'Distance [m]');
  } else if (device.type === 'TouchSensor') {
    if (device.sensorType === 'force-3d')
      this.createGeneric3DDevice(device, TimeplotWidget.prototype.AutoRangeType.STRETCH, -100.0, 100.0, '[N]');
    else if (device.sensorType === 'force')
      this.createGeneric1DDevice(device, TimeplotWidget.prototype.AutoRangeType.STRETCH, 0, 20.0, 'Raw');
    else // bumper
      this.createGeneric1DDevice(device, TimeplotWidget.prototype.AutoRangeType.STRETCH, 0, 1.0, 'Raw');
  }
};

DeviceWidget.prototype.createRadar = function(device) {
  this.plots = [new RadarWidget(document.getElementById(device.name + '-content'), device)];
};

DeviceWidget.prototype.createMotor = function(device, autoRange, minValue, maxValue, yLabel) {
  let customStyle = '';
  if (navigator.appVersion.indexOf('Linux') !== -1)
    customStyle = ' style="width:17px;"';
  const mean = 0.5 * (maxValue + minValue);
  const step = 0.01 * (maxValue - minValue); // 1%
  const slider = appendNewElement(device.name + '-content',
    '<input type="range" min="' + minValue + '" max="' + maxValue + '" value="' + mean + '" step="' + step + '"' +
    ' class="motor-slider"' + customStyle +
    ' id="' + device.htmlName + '-slider"' +
    ' device="' + device.htmlName + '"' +
    ' disabled=true' +
    '>');
  slider.oninput = function() { DeviceWidget.motorSetPosition(device.name, slider.value); };
  slider.onmousedown = function() { DeviceWidget.motorSetPosition(device.name, slider.value); };
  slider.onmouseup = function() { DeviceWidget.motorUnsetPosition(device.name); };
  slider.onmouseleave = function() { DeviceWidget.motorUnsetPosition(device.name); };
  const widget = new TimeplotWidget(document.getElementById(device.name + '-content'), this.basicTimeStep, autoRange, {'min': minValue, 'max': maxValue}, {'x': 'Time [s]', 'y': yLabel}, device);
  widget.setLabel(document.getElementById(device.name + '-label'));
  widget.setSlider(slider);
  this.plots = [widget];
  DeviceWidget.motorWidgets[device.name] = this;
};

DeviceWidget.prototype.createDifferentialWheels = function(device) {
  const buttonsData = [ // tag name, button tool tip, button text
    ['left_forward', 'Left-forward', '&#8598;'],
    ['forward', 'Forwads', '&#8593;'],
    ['right_forward', 'Right-forward', '&#8599;'],
    ['left', 'Left', '&#8592;'],
    ['stop', 'Stop', 'O'],
    ['right', 'Right', '&#8594;'],
    ['left_backward', 'Left-backward', '&#8601;'],
    ['backward', 'Backward', '&#8595;'],
    ['right_backward', 'Right-backward', '&#8600;']
  ];
  buttonsData.forEach(function(buttonData) {
    appendNewElement(device.name + '-content',
      '<button class="differential-wheels-button" ' +
        'title="' + buttonData[1] + '" ' +
        'onmousedown="DeviceWidget.differentialWheelsSetSpeedTag(\'' + buttonData[0] + '\')" ' +
        'onmouseup="DeviceWidget.differentialWheelsResetSpeedTag(\'' + buttonData[0] + '\')" ' +
        'onmouseleave="DeviceWidget.differentialWheelsResetSpeedTag(\'' + buttonData[0] + '\')" ' +
        '>' + buttonData[2] +
      '</button>'
    );
  });
};

DeviceWidget.prototype.createGenericImageDevice = function(device) {
  this.image = appendNewElement(device.name + '-content', '<div id="' + device.htmlName + '-image" class="image"></div>');
};

DeviceWidget.prototype.createGeneric1DDevice = function(device, autoRange, minY, maxY, labelY) {
  const widget = new TimeplotWidget(document.getElementById(device.name + '-content'), this.basicTimeStep, autoRange, {'min': minY, 'max': maxY}, {'x': 'Time [s]', 'y': labelY}, device);
  widget.setLabel(document.getElementById(device.name + '-label'));
  this.plots = [widget];
};

DeviceWidget.prototype.createGeneric3DDevice = function(device, autoRange, minRange, maxRange, units) {
  appendNewElement(device.name,
    '<select onChange="DeviceWidget.comboboxCallback(this)" class="view-selector" device="' + device.htmlName + '">' +
    '  <option>Time</option>' +
    '  <option>XY</option>' +
    '  <option>YZ</option>' +
    '  <option>XZ</option>' +
    '</select>'
  );
  const plotAutoRange = autoRange !== TimeplotWidget.prototype.AutoRangeType.NONE;

  const widgetTime = new TimeplotWidget(document.getElementById(device.name + '-content'), this.basicTimeStep, autoRange, {'min': minRange, 'max': maxRange}, {'x': 'Time [s]', 'y': 'Raw'}, device);
  widgetTime.setLabel(document.getElementById(device.name + '-label'));
  const widgetXY = new PlotWidget(document.getElementById(device.name + '-content'), plotAutoRange, {'x': 0, 'y': 1}, {'min': minRange, 'max': maxRange}, {'min': minRange, 'max': maxRange}, {'x': 'x ' + units, 'y': 'y ' + units}, device);
  const widgetYZ = new PlotWidget(document.getElementById(device.name + '-content'), plotAutoRange, {'x': 1, 'y': 2}, {'min': minRange, 'max': maxRange}, {'min': minRange, 'max': maxRange}, {'x': 'y ' + units, 'y': 'z ' + units}, device);
  const widgetXZ = new PlotWidget(document.getElementById(device.name + '-content'), plotAutoRange, {'x': 0, 'y': 2}, {'min': minRange, 'max': maxRange}, {'min': minRange, 'max': maxRange}, {'x': 'x ' + units, 'y': 'z ' + units}, device);

  widgetXY.show(false);
  widgetYZ.show(false);
  widgetXZ.show(false);

  this.plots = [widgetTime, widgetXY, widgetYZ, widgetXZ];
};

DeviceWidget.prototype.enable = function(enabled) {
  let checkbox = document.getElementById(this.device.htmlName + '-enable-checkbox');
  if (checkbox && checkbox.checked !== enabled) {
    checkbox.checked = enabled;
    DeviceWidget.checkboxCallback(checkbox);
  }
};

DeviceWidget.prototype.refresh = function() {
  if (this.plots) {
    this.plots.forEach(function(widget) {
      widget.refresh();
    });
  }
};

DeviceWidget.prototype.resize = function() {
  if (this.plots) {
    this.plots.forEach(function(widget) {
      if (typeof widget.resize === 'function')
        widget.resize();
    });
  }
};

DeviceWidget.createWidget = function(basicTimeStep, device) {
  const widget = new DeviceWidget(basicTimeStep, device);
  DeviceWidget.widgets[device.name] = widget;
};

DeviceWidget.updateMotorSlider = function(deviceName, enabled) {
  const motorSlider = document.getElementById(deviceName + '-slider');
  if (motorSlider)
    motorSlider.disabled = !enabled;
};

DeviceWidget.checkboxCallback = function(checkbox) {
  if (checkbox.checked)
    DeviceWidget.commands.push(checkbox.getAttribute('device') + ':enable');
  else
    DeviceWidget.commands.push(checkbox.getAttribute('device') + ':disable');
  DeviceWidget.touchedCheckboxes.push(checkbox);
  DeviceWidget.updateMotorSlider(checkbox.getAttribute('device'), checkbox.checked);
};

DeviceWidget.cameraRecognitionCheckboxCallback = function(checkbox) {
  const deviceName = checkbox.getAttribute('device');
  if (checkbox.checked)
    DeviceWidget.commands.push(deviceName + ':recognitionEnable');
  else
    DeviceWidget.commands.push(deviceName + ':recognitionDisable');
  const segmentationCheckbox = document.getElementById(deviceName + '-segmentation-checkbox');
  if (segmentationCheckbox)
    segmentationCheckbox.disabled = !checkbox.checked;
  DeviceWidget.touchedCheckboxes.push(checkbox);
};

DeviceWidget.cameraSegmentationCheckboxCallback = function(checkbox) {
  if (checkbox.checked)
    DeviceWidget.commands.push(checkbox.getAttribute('device') + ':segmentationEnable');
  else
    DeviceWidget.commands.push(checkbox.getAttribute('device') + ':segmentationDisable');
  DeviceWidget.touchedCheckboxes.push(checkbox);
};

DeviceWidget.pointCloudCheckboxCallback = function(checkbox) {
  if (checkbox.checked)
    DeviceWidget.commands.push(checkbox.getAttribute('device') + ':pointCloudEnable');
  else
    DeviceWidget.commands.push(checkbox.getAttribute('device') + ':pointCloudDisable');
  DeviceWidget.touchedCheckboxes.push(checkbox);
};

DeviceWidget.comboboxCallback = function(combobox) {
  const devicePlots = DeviceWidget.widgets[combobox.getAttribute('device')].plots;
  devicePlots.forEach(function(widget) {
    widget.show(false);
  });
  devicePlots[combobox.selectedIndex].show(true);
};

DeviceWidget.motorSetPosition = function(deviceName, value) {
  DeviceWidget.motorWidgets[deviceName].plots.forEach(function(widget) {
    widget.blockSliderUpdate(true);
  });
  DeviceWidget.motorCommands[deviceName] = value;
};

DeviceWidget.motorUnsetPosition = function(deviceName) {
  DeviceWidget.motorWidgets[deviceName].plots.forEach(function(widget) {
    widget.blockSliderUpdate(false);
  });
  delete DeviceWidget.motorCommands[deviceName];
};

DeviceWidget.differentialWheelsSetSpeedTag = function(tag) {
  DeviceWidget.differentialWheelsTag = tag;
};

DeviceWidget.differentialWheelsResetSpeedTag = function(_tag) {
  DeviceWidget.differentialWheelsTag = '';
};

DeviceWidget.updateDeviceWidgets = function(data) {
  if (data.devices == null)
    return;
  Object.keys(data.devices).forEach(function(key) {
    const value = data.devices[key];
    key = key.replace(/&quot;/g, '"');

    const checkbox = document.getElementById(key + '-enable-checkbox');
    const widget = DeviceWidget.widgets[key];
    if (!widget || !(widget.firstUpdate || checkbox.checked))
      return;

    widget.firstUpdate = false;

    if (value.update !== undefined && widget.plots) {
      widget.plots.forEach(function(plot) {
        value.update.forEach(function(u) {
          plot.addValue({'x': u.time, 'y': u.value });
        });
        plot.refresh();
      });
      if (checkbox && value.update.length > 0)
        DeviceWidget.applyToUntouchedCheckbox(checkbox, true);
    } else if (value.image && widget.image) {
      widget.image.style.backgroundImage = 'url("' + value.image + '")';
      if (checkbox)
        DeviceWidget.applyToUntouchedCheckbox(checkbox, true);
      if (value.cloudPointEnabled !== undefined) {
        const cloudPointCheckbox = document.getElementById(key + '-cloud-point-checkbox');
        DeviceWidget.applyToUntouchedCheckbox(cloudPointCheckbox, value.cloudPointEnabled);
      }
      if (value.recognitionEnabled !== undefined) {
        const recognitionCheckbox = document.getElementById(key + '-recognition-checkbox');
        DeviceWidget.applyToUntouchedCheckbox(recognitionCheckbox, value.recognitionEnabled);
        const segmentationCheckbox = document.getElementById(key + '-segmentation-checkbox');
        if (segmentationCheckbox)
          segmentationCheckbox.disabled = !value.recognitionEnabled;
      }
      if (value.segmentationEnabled !== undefined) {
        const segmentationCheckbox = document.getElementById(key + '-segmentation-checkbox');
        DeviceWidget.applyToUntouchedCheckbox(segmentationCheckbox, value.segmentationEnabled);
      }
    } else if (value.targets !== undefined && widget.plots) {
      widget.plots[0].refreshTargets(value.targets);
      if (checkbox && value.targets.length > 0)
        DeviceWidget.applyToUntouchedCheckbox(checkbox, true);
    }
  });
};

DeviceWidget.applyToUntouchedCheckbox = function(checkbox, state) {
  const checkboxIndex = DeviceWidget.touchedCheckboxes.indexOf(checkbox);
  if (checkboxIndex >= 0)
    DeviceWidget.touchedCheckboxes.splice(checkboxIndex, 1);
  else
    checkbox.checked = state;
  DeviceWidget.updateMotorSlider(checkbox.getAttribute('device'), state);
};

DeviceWidget.pushDeviceCommands = function(robotName) {
  if (DeviceWidget.differentialWheelsTag)
    DeviceWidget.commands.push(protect(robotName, [':', ',']) + ':' + DeviceWidget.differentialWheelsTag);
  Object.keys(DeviceWidget.motorCommands).forEach(function(deviceName) {
    DeviceWidget.commands.push(protect(deviceName, [':', ',']) + ':position=' + DeviceWidget.motorCommands[deviceName]);
  });
};

// Protect all the character instances in the string by prefixing a '\' character.
function protect(str, chars) {
  let protectedStr = str;
  for (let c = 0; c < chars.length; c++) {
    const char = chars[c];
    protectedStr = protectedStr.replace(new RegExp(char, 'g'), '\\' + char);
  }
  return protectedStr;
}
