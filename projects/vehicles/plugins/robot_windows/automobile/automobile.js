import RobotWindow from 'https://cyberbotics.com/wwi/R2022b/RobotWindow.js';
/* global DeviceWidget: false */
/* global TimeplotWidget: false */
/* global VehicleTimeplotWidget: false */
/* global OverviewWidget: false */
/* global menuTabCallback, openMenu, closeMenu, addTab, addSettingsTab, appendNewElement, refreshSelectedTab */
/* global configureDevices, setupWindow, windowIsHidden, parseJSONMessage, roundLabel */
/* eslint no-unused-vars: ["error", { "varsIgnorePattern": "Callback", "argsIgnorePattern": "^_"}] */

var robotName = '';
var basicTimeStep = -1;
var commands = []; // Commands to be sent to the C library.
window.widgets = {}; // Dictionary {deviceType -> {deviceName -> DeviceWidget }}
window.selectedDeviceType = null;
var automobileWidgets = {}; // Dictionary {tabName -> automobile widget }

const DRIVER_SPEED_MODE = 0;
const DRIVER_TORQUE_MODE = 1; // DRIVER_UNDEFINED_MODE == -1
var driverControlMode;
var overviewWidget = null;

window.updateControlMode = function(controlMode) {
  if (driverControlMode === controlMode)
    return;

  driverControlMode = controlMode;
  const isTorqueMode = driverControlMode === DRIVER_TORQUE_MODE;
  const isSpeedMode = driverControlMode === DRIVER_SPEED_MODE;
  let labelText = !isTorqueMode ? 'Engine model only available in torque control' : '';
  document.getElementById('throttle-label').textContent = labelText;
  document.getElementById('rpm-label').textContent = labelText;
  document.getElementById('throttle-enable-checkbox').disabled = !isTorqueMode;
  document.getElementById('rpm-enable-checkbox').disabled = !isTorqueMode;
  labelText = (isSpeedMode || isTorqueMode) ? 'Target speed: -' : 'Vehicle speed not set using the Driver library';
  document.getElementById('target-speed-label').textContent = labelText;
  document.getElementById('speed-enable-checkbox').disabled = !isSpeedMode && !isTorqueMode;
  overviewWidget.updateControlMode(isSpeedMode, isTorqueMode);
}

window.vehicleCheckboxCallback = function(checkbox) {
  if (checkbox.checked)
    commands.push(checkbox.getAttribute('device') + ':enable');
  else {
    commands.push(checkbox.getAttribute('device') + ':disable');
    if (checkbox.getAttribute('device') === 'Speed')
      document.getElementById('target-speed-label').textContent = 'Target speed: -';
  }
}

window.addDriverInfo = function(device, label, min, max) {
  if (document.getElementById(device.name))
    return; // check if already exists

  const lowerDeviceName = device.name.toLowerCase();

  let div = '<div id="' + device.name + '" class="vehicle-device device">';
  div += '<h2>';
  div += '<input type="checkbox" title="Enable/disable this plot." id="' + lowerDeviceName + '-enable-checkbox" device="' + device.name + '" onclick="vehicleCheckboxCallback(this)" />';
  div += device.name + '<span id="' + device.name + '-label"></span></h2>';
  div += '<div id="' + device.name + '-content" class="vehicle-device-content device-content"/>';
  div += '</div>';

  appendNewElement(device.name + '-layout', div);

  let decimals = 3;
  let plotLabels = {'x': 'Time [s]', 'y': label};
  let autoRange = TimeplotWidget.AutoRangeType.STRETCH;
  if (lowerDeviceName === 'steering') {
    plotLabels['legend'] = ['steering angle', 'right steering angle', 'left steering angle'];
    decimals = 4;
  } else if (lowerDeviceName === 'encoders') {
    plotLabels['legend'] = ['front right', 'front left', 'rear right', 'rear left'];
    decimals = 1;
    autoRange = TimeplotWidget.AutoRangeType.ADAPT;
  }

  const widget = new VehicleTimeplotWidget(document.getElementById(device.name + '-content'), basicTimeStep, autoRange, {'min': min, 'max': max}, plotLabels, device, decimals);
  widget.setLabel(document.getElementById(device.name + '-label'));
  automobileWidgets[lowerDeviceName] = widget;
}

window.setDeviceModeCallback = function(switchButton, deviceType) {
  const messageHeader = 'device-control-mode:' + deviceType;
  const message = messageHeader + ':' + (switchButton.checked ? '1' : '0');
  for (let i = 0; i < commands.length; ++i) {
    if (commands[i].startsWith(messageHeader)) {
      commands[i] = message;
      return;
    }
  }
  commands.push(message);

  // force widgets refresh when they are shown.
  let tabWidgets = window.widgets[deviceType];
  Object.keys(tabWidgets).forEach(function(deviceName) {
    const widget = tabWidgets[deviceName];
    if (widget) {
      const checkbox = document.getElementById(widget.device.name + '-enable-checkbox');
      DeviceWidget.checkboxCallback(checkbox);
    }
  });
}

function configure(data) {
  robotName = data.name;
  basicTimeStep = data.basicTimeStep;

  window.robotWindow.setTitle('Vehicle robot window [' + robotName + ']');

  if (data.devices == null) {
    document.getElementById('no-controller-label').innerHTML = 'No devices.';
    closeMenu();
    return;
  }

  document.getElementById('no-controller-label').style.display = 'none';

  // Overview tab
  overviewWidget = new OverviewWidget(document.getElementById('overview-section'));

  addTab('Speed');
  appendNewElement('Speed-layout', '<h2><span id="target-speed-label" style="color:red"></span></h2>');
  addDriverInfo({name: 'Speed', htmlName: 'Speed'}, '[km/h]', -20.0, 20.0);

  addTab('Steering');
  addDriverInfo({name: 'Steering'}, '[rad]', -0.1, 0.1);

  addTab('Encoders');
  addDriverInfo({name: 'Encoders'}, '[rad]', 0, 200);

  addTab('Brake');
  addDriverInfo({name: 'Brake'}, '[%]', 0, 1);

  addTab('Throttle');
  appendNewElement('Throttle-layout', '<h2><span id="throttle-label"></span></h2>');
  addDriverInfo({name: 'Throttle'}, '[%]', 0, 1);

  addTab('RPM');
  appendNewElement('RPM-layout', '<h2><span id="rpm-label"></span></h2>');
  addDriverInfo({name: 'RPM'}, '[RPM]', 0, 200);

  // Parse the devices once to prepare the device type list.
  // Devices widgets will be added to window.widgets.
  window.widgets = {};
  configureDevices(data, true);

  addSettingsTab();

  window.widgets = Object.assign(window.widgets, {'overview': {'overview': overviewWidget}});
  Object.keys(automobileWidgets).forEach(function(deviceName) {
    const widget = automobileWidgets[deviceName];
    window.widgets[widget.device.name] = {};
    window.widgets[widget.device.name][deviceName] = widget;
  });

  // Set the focus on the Overview tab
  menuTabCallback('overview');
  openMenu();
}

function update(data) {
  if (data.vehicle != null) {
    let refreshRequired = false;
    Object.keys(data.vehicle).forEach(function(key) {
      key = key.replace(/&quot;/g, '"');

      if (key === 'control-mode') {
        updateControlMode(data.vehicle[key]);
        return;
      }

      const checkbox = document.getElementById(key + '-enable-checkbox');
      const widget = key === 'overview' ? overviewWidget : automobileWidgets[key];
      if (!checkbox || !widget || !(widget.firstUpdate || checkbox.checked))
        return;

      // Request autmomobile-specific tab refresh only if modified.
      if (!refreshRequired && window.selectedDeviceType.toLowerCase() === key.toLowerCase())
        refreshRequired = true;
      widget.firstUpdate = false;

      if (key === 'overview')
        overviewWidget.updateInformation(data.vehicle.overview);
      else if (key === 'speed') {
        Object.keys(data.vehicle.speed).forEach(function(attribute) {
          const attributeValue = data.vehicle.speed[attribute];
          attribute = attribute.replace(/&quot;/g, '"');
          if (attribute === 'cruising-speed') {
            let label = document.getElementById('target-speed-label');
            label.textContent = 'Target speed: ' + roundLabel(attributeValue) + ' km/h';
          } else if (attribute === 'update') {
            let plot = widget.timeplot;
            attributeValue.forEach(function(u) {
              plot.addValue({'x': u.time, 'y': u.value });
            });
          }
        });
      } else if (key === 'steering' || key === 'encoders' || key === 'brake' || key === 'throttle' || key === 'rpm') {
        const time = data.vehicle[key]['time'];
        const measurement = data.vehicle[key]['value'];
        let plot = widget.timeplot;
        plot.addValue({'x': time, 'y': measurement });
      }
    });
    if (refreshRequired)
      refreshSelectedTab();
  }

  if (DeviceWidget.updateDeviceWidgets(data, window.selectedDeviceType))
    refreshSelectedTab();
}

function receive(message, _robot) {
  let data = '';
  if (message.indexOf('configure ') === 0) {
    data = parseJSONMessage(message.substring(10));
    if (data)
      configure(data);
  } else if (windowIsHidden)
    return;
  else if (message.indexOf('configure-vehicle ') === 0) {
    data = parseJSONMessage(message.substring(17));
    if (data) {
      overviewWidget.setStaticInformation(data);
      overviewWidget.resize();
    }
  } else if (message.indexOf('update ') === 0) {
    data = parseJSONMessage(message.substring(7));
    if (data)
      update(data);
  } else
    console.log("Unexpected message received: '" + message + "'");

  DeviceWidget.pushDeviceCommands();

  commands = commands.concat(DeviceWidget.commands);
  if (commands.length !== 0) {
    window.robotWindow.send(commands.join());
    DeviceWidget.commands = [];
    commands = [];
  }
}

window.onload = function() {
  window.robotWindow = new RobotWindow();
  window.robotWindow.setTitle('Vehicle robot window');
  window.robotWindow.receive = receive;
  setupWindow();
};
