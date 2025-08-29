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

DeviceWidget.deviceNameToType = {}; // Dictionary {deviceName: deviceType }
DeviceWidget.widgets = {}; // Dictionary {deviceType: {deviceName: DeviceWidget} }
DeviceWidget.commands = []; // Commands to be sent to the C library.
DeviceWidget.motorCommands = []; // Motor commands to be sent to the C library.
DeviceWidget.touchedCheckboxes = [];
DeviceWidget.supportedDeviceTypes = [
  'Accelerometer', 'Altimeter', 'Camera', 'Compass', 'DistanceSensor', 'GPS', 'Gyro',
  'InertialUnit', 'Lidar', 'LightSensor', 'LinearMotor', 'PositionSensor', 'Radar',
  'RangeFinder', 'RotationalMotor', 'TouchSensor', 'VacuumGripper'
];

DeviceWidget.prototype.initialize = function(device) {
  if (document.getElementById(device.htmlName))
    return; // check if already exists

  let div = '<div id="' + device.htmlName + '" class="device">';
  div += '<h2>';
  div += '<input type="checkbox" title="Enable/disable this device." id="' + device.htmlName + '-enable-checkbox" device="' + device.htmlName + '" onclick="DeviceWidget.checkboxCallback(this)" />';
  div += device.htmlName + '<span id="' + device.htmlName + '-label"></span></h2>';
  if (device.type === 'Camera' && device.recognition === 1) {
    if (device.segmentation === 1) {
      div += '<div class="device-option-enable-label">';
      div += '<h2 class="device-option-item-enable-label">';
    } else
      div += '<h2 class="device-option-enable-label">';
    div += '<input type="checkbox" title="Enable/disable the recognition functionality." id="' + device.htmlName + '-recognition-checkbox" device="' + device.htmlName + '" onclick="DeviceWidget.cameraRecognitionCheckboxCallback(this)"/>';
    div += '<span>Recognition</span>';
    div += '</h2>';
    if (device.segmentation === 1) {
      div += '<h2 class="device-option-item-enable-label">';
      div += '<input type="checkbox" title="Enable/disable the segmentation functionality." id="' + device.htmlName + '-segmentation-checkbox" device="' + device.htmlName + '" onclick="DeviceWidget.cameraSegmentationCheckboxCallback(this)" disabled/>';
      div += '<span>Segmentation</span>';
      div += '</h2>';
      div += '</div>';
    }
  } else if (device.type === 'Lidar') {
    div += '<h2 class="device-option-enable-label">';
    div += '<input type="checkbox" title="Enable/disable the lidar point cloud." id="' + device.htmlName + '-cloud-point-checkbox" device="' + device.htmlName + '" onclick="DeviceWidget.pointCloudCheckboxCallback(this)"/>';
    div += '<span>Point Cloud</span>';
    div += '</h2>';
  } else if (device.type === 'VacuumGripper') {
    div += '<h2 class="device-option-enable-label">';
    div += '<input type="checkbox" title="Turn on/off the vacuum gripper." id="' + device.htmlName + '-turn-on-checkbox" device="' + device.htmlName + '" onclick="DeviceWidget.vacuumGripperTurnOnCallback(this)"/>';
    div += '<span>Turned on</span>';
    div += '</h2>';
  }
  div += '<div id="' + device.htmlName + '-content" class="device-content"/>';
  div += '</div>';
  div += '<div class="device-background"/>';

  appendNewElement(device.type + '-layout', div);

  if (device.type === 'Accelerometer')
    this.createGeneric3DDevice(device, TimeplotWidget.AutoRangeType.STRETCH, -20.0, 20.0, '[m/s^2]');
  else if (device.type === 'Altimeter')
    this.createGeneric1DDevice(device, TimeplotWidget.AutoRangeType.STRETCH, 0, 0.5, '[m]');
  else if (device.type === 'Camera')
    this.createGenericImageDevice(device);
  else if (device.type === 'Compass')
    this.createGeneric3DDevice(device, TimeplotWidget.AutoRangeType.NONE, -1.0, 1.0, '');
  else if (device.type === 'DistanceSensor')
    this.createGeneric1DDevice(device, TimeplotWidget.AutoRangeType.NONE, device.minValue, device.maxValue, 'Raw');
  else if (device.type === 'GPS')
    this.createGeneric3DDevice(device, TimeplotWidget.AutoRangeType.STRETCH, -1.0, 1.0, '[m]');
  else if (device.type === 'Gyro')
    this.createGeneric3DDevice(device, TimeplotWidget.AutoRangeType.STRETCH, -20.0, 20.0, '[rad/s]');
  else if (device.type === 'InertialUnit')
    this.createGeneric3DDevice(device, TimeplotWidget.AutoRangeType.NONE, -Math.PI, Math.PI, '[rad]');
  else if (device.type === 'Lidar')
    this.createGenericImageDevice(device);
  else if (device.type === 'LightSensor')
    this.createGeneric1DDevice(device, TimeplotWidget.AutoRangeType.STRETCH, 0, 1024, 'Raw');
  else if (device.type === 'PositionSensor')
    this.createGeneric1DDevice(device, TimeplotWidget.AutoRangeType.ADAPT, -Math.PI, Math.PI, 'Angle [Rad]');
  else if (device.type === 'Radar')
    this.createRadar(device);
  else if (device.type === 'RangeFinder')
    this.createGenericImageDevice(device);
  else if (device.type === 'RotationalMotor' || device.type === 'LinearMotor') {
    let minPosition = device.minPosition;
    let maxPosition = device.maxPosition;
    let autoRange = TimeplotWidget.AutoRangeType.NONE;
    if (minPosition === 0.0 && maxPosition === 0.0) {
      minPosition = device.type === 'RotationalMotor' ? -Math.PI : -1.0;
      maxPosition = device.type === 'RotationalMotor' ? Math.PI : 1.0;
      autoRange = device.type === 'RotationalMotor' ? TimeplotWidget.AutoRangeType.ADAPT : TimeplotWidget.AutoRangeType.STRETCH;
    }
    this.createMotor(device, autoRange, minPosition, maxPosition, device.type === 'RotationalMotor' ? 'Angle [rad]' : 'Distance [m]');
  } else if (device.type === 'TouchSensor') {
    if (device.sensorType === 'force-3d')
      this.createGeneric3DDevice(device, TimeplotWidget.AutoRangeType.STRETCH, -100.0, 100.0, '[N]');
    else if (device.sensorType === 'force')
      this.createGeneric1DDevice(device, TimeplotWidget.AutoRangeType.STRETCH, 0, 20.0, 'Raw');
    else // bumper
      this.createGeneric1DDevice(device, TimeplotWidget.AutoRangeType.STRETCH, 0, 1.0, 'Raw');
  } else if (device.type === 'VacuumGripper')
      this.createGeneric1DDevice(device, TimeplotWidget.AutoRangeType.NONE, -0.2, 1.2, 'Presence');
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
    '<input type="range" orient="vertical" min="' + minValue + '" max="' + maxValue + '" value="' + mean + '" step="' + step + '"' +
    ' class="motor-slider"' + customStyle +
    ' id="' + device.htmlName + '-slider"' +
    ' device="' + device.htmlName + '"' +
    ' disabled=true' +
    '>');
  slider.oninput = function() { DeviceWidget.motorSetPosition(device.type, device.name, slider.value); };
  slider.onmousedown = function() { DeviceWidget.motorSetPosition(device.type, device.name, slider.value); };
  slider.onmouseup = function() { DeviceWidget.motorUnsetPosition(device.type, device.name); };
  slider.onmouseleave = function() { DeviceWidget.motorUnsetPosition(device.type, device.name); };
  const widget = new TimeplotWidget(document.getElementById(device.name + '-content'), this.basicTimeStep, autoRange, {'min': minValue, 'max': maxValue}, {'x': 'Time [s]', 'y': yLabel}, device);
  widget.setLabel(document.getElementById(device.name + '-label'));
  widget.setSlider(slider);
  this.plots = [widget];
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
    '<select onChange="DeviceWidget.comboboxCallback(this)" class="view-selector" deviceName="' + device.htmlName + '" deviceType="' + device.type + '">' +
    '  <option>Time</option>' +
    '  <option>XY</option>' +
    '  <option>YZ</option>' +
    '  <option>XZ</option>' +
    '</select>'
  );
  const plotAutoRange = autoRange !== TimeplotWidget.AutoRangeType.NONE;

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
    this.plots.forEach(function(plot) {
      if (typeof plot.refresh === 'function')
        plot.refresh();
    });
  }
};

DeviceWidget.prototype.refreshLabels = function() {
  if (this.plots) {
    this.plots.forEach(function(plot) {
      if (typeof plot.refreshLabels === 'function')
        plot.refreshLabels();
    });
  }
};

DeviceWidget.prototype.resize = function() {
  if (this.plots) {
    this.plots.forEach(function(plot) {
      if (typeof plot.resize === 'function')
        plot.resize();
    });
  }
};

DeviceWidget.createWidget = function(basicTimeStep, device) {
  const widget = new DeviceWidget(basicTimeStep, device);
  if (!DeviceWidget.widgets[device.type])
    DeviceWidget.widgets[device.type] = {};
  DeviceWidget.widgets[device.type][device.name] = widget;
  DeviceWidget.deviceNameToType[device.name] = device.type;
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

DeviceWidget.vacuumGripperTurnOnCallback = function(checkbox) {
  if (checkbox.checked)
    DeviceWidget.commands.push(checkbox.getAttribute('device') + ':vacuumGripperTurnOn');
  else
    DeviceWidget.commands.push(checkbox.getAttribute('device') + ':vacuumGripperTurnOff');
  DeviceWidget.touchedCheckboxes.push(checkbox);
};

DeviceWidget.comboboxCallback = function(combobox) {
  const type = combobox.getAttribute('deviceType');
  const name = combobox.getAttribute('deviceName');
  const devicePlots = DeviceWidget.widgets[type][name].plots;
  devicePlots.forEach(function(widget) {
    widget.show(false);
  });
  devicePlots[combobox.selectedIndex].show(true);
  devicePlots[combobox.selectedIndex].refresh();
};

DeviceWidget.motorSetPosition = function(deviceType, deviceName, value) {
  DeviceWidget.widgets[deviceType][deviceName].plots.forEach(function(widget) {
    widget.blockSliderUpdate(true);
  });
  DeviceWidget.motorCommands[deviceName] = value;
};

DeviceWidget.motorUnsetPosition = function(deviceType, deviceName) {
  DeviceWidget.widgets[deviceType][deviceName].plots.forEach(function(widget) {
    widget.blockSliderUpdate(false);
  });
  delete DeviceWidget.motorCommands[deviceName];
};

DeviceWidget.updateDeviceWidgets = function(data, selectedDeviceType) {
  if (data.devices == null)
    return;
  let resquestTabUpdate = false;
  Object.keys(data.devices).forEach(function(deviceName) {
    const value = data.devices[deviceName];
    deviceName = deviceName.replace(/&quot;/g, '"');
    const deviceType = DeviceWidget.deviceNameToType[deviceName];
    if (!deviceType)
      return;

    const checkbox = document.getElementById(deviceName + '-enable-checkbox');
    const widget = DeviceWidget.widgets[deviceType][deviceName];
    if (!checkbox || !widget || !(widget.firstUpdate || checkbox.checked))
      return;

    if (!resquestTabUpdate && selectedDeviceType === deviceType)
      resquestTabUpdate = true;
    widget.firstUpdate = false;

    if (value.update !== undefined && widget.plots) {
      widget.plots.forEach(function(plot) {
        value.update.forEach(function(u) {
          plot.addValue({'x': u.time, 'y': u.value });
        });
      });
      if (checkbox && value.update.length > 0)
        DeviceWidget.applyToUntouchedCheckbox(checkbox, true);
    } else if (value.image && widget.image) {
      const img = new Image();
      img.src = value.image;
      img.decode()
      .then(() => {
        widget.image.style.backgroundImage = 'url("' + img.src + '")';
      })
      if (checkbox)
        DeviceWidget.applyToUntouchedCheckbox(checkbox, true);
      if (value.cloudPointEnabled !== undefined) {
        const cloudPointCheckbox = document.getElementById(deviceName + '-cloud-point-checkbox');
        DeviceWidget.applyToUntouchedCheckbox(cloudPointCheckbox, value.cloudPointEnabled);
      }
      if (value.recognitionEnabled !== undefined) {
        const recognitionCheckbox = document.getElementById(deviceName + '-recognition-checkbox');
        DeviceWidget.applyToUntouchedCheckbox(recognitionCheckbox, value.recognitionEnabled);
        const segmentationCheckbox = document.getElementById(deviceName + '-segmentation-checkbox');
        if (segmentationCheckbox)
          segmentationCheckbox.disabled = !value.recognitionEnabled;
      }
      if (value.segmentationEnabled !== undefined) {
        const segmentationCheckbox = document.getElementById(deviceName + '-segmentation-checkbox');
        DeviceWidget.applyToUntouchedCheckbox(segmentationCheckbox, value.segmentationEnabled);
      }
      if (value.vaccumGripperOn !== undefined) {
        const vacuumGripperCheckbox = document.getElementById(deviceName + '-turn-on-checkbox');
        DeviceWidget.applyToUntouchedCheckbox(vacuumGripperCheckbox, value.vaccumGripperOn);
      }
    } else if (value.targets !== undefined && widget.plots) {
      widget.plots[0].refreshTargets(value.targets);
      if (checkbox && value.targets.length > 0)
        DeviceWidget.applyToUntouchedCheckbox(checkbox, true);
    }
  });
  return resquestTabUpdate;
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
