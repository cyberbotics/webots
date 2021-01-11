/* global webots: false */
/* global TimeplotWidget: false */
/* global OverviewWidget: false */
/* global appendNewElement: false */
/* global createGeneric1DDevice: false */
/* global createGeneric3DDevice: false */
/* global openMenu: false */
/* global closeMenu: false */
/* global menuTabCallback: false */
/* global addTab: false */
/* global roundLabel: false */
/* global configureDevices: false */
/* global pushDeviceCommands: false */
/* global updateDevices: false */
/* exported basicTimeStep */
/* eslint no-unused-vars: ["error", { "varsIgnorePattern": "Callback", "argsIgnorePattern": "^_"}] */

var basicTimeStep = -1;
var robotName = '';
var commands = []; // Commands to be sent to the C library.
var widgets = {}; // Dictionary {deviceName -> [TimeplotWidget0, PlotWidget0, PlotWidget1, ...] }

const DRIVER_SPEED_MODE = 0;
// const DRIVER_TORQUE_MODE = 1;
var driverControlMode;
var overviewWidget = null;

function updateControlMode(controlMode) {
  if (driverControlMode === controlMode)
    return;

  driverControlMode = controlMode;
  const isSpeedMode = driverControlMode === DRIVER_SPEED_MODE;
  let labelText = isSpeedMode ? 'No engine model in speed control' : '';
  document.getElementById('throttle-label').textContent = labelText;
  document.getElementById('rpm-label').textContent = labelText;
  document.getElementById('throttle-enable-checkbox').disabled = isSpeedMode;
  document.getElementById('rpm-enable-checkbox').disabled = isSpeedMode;
  overviewWidget.updateControlMode(isSpeedMode);
}

function vehicleCheckboxCallback(checkbox) {
  if (checkbox.checked)
    commands.push(checkbox.getAttribute('device') + ':enable');
  else {
    commands.push(checkbox.getAttribute('device') + ':disable');
    if (checkbox.getAttribute('device') === 'Speed')
      document.getElementById('target-speed-label').textContent = 'Target speed: -';
  }
}

function addDriverInfo(device, label, min, max) {
  if (document.getElementById(name))
    return; // check if already exists

  let div = '<div id="' + device.name + '" class="vehicle-device device">';
  div += '<h2>';
  console.log('addDriverInfo ' + device.name.toLowerCase() + '-enable-checkbox');
  div += '<input type="checkbox" title="Enable/disable this plot." id="' + device.name.toLowerCase() + '-enable-checkbox" device="' + device.name + '" onclick="vehicleCheckboxCallback(this)" />';
  div += device.name + '<span id="' + device.name + '-label"></span></h2>';
  div += '<div id="' + device.name + '-content" class="vehicle-device-content device-content"/>';
  div += '</div>';

  appendNewElement(device.name + '-layout', div);

  let decimals = 3;
  let plotLabels = {'x': 'Time [s]', 'y': label};
  if (device.name.toLowerCase() === 'steering') {
    plotLabels['legend'] = ['steering angle', 'right steering angle', 'left steering angle'];
    decimals = 4;
  } else if (device.name.toLowerCase() === 'encoders') {
    plotLabels['legend'] = ['front right', 'front left', 'rear right', 'rear left'];
    decimals = 1;
  }

  const widget = new TimeplotWidget(document.getElementById(device.name + '-content'), basicTimeStep, TimeplotWidget.prototype.AutoRangeType.STRETCH, {'min': min, 'max': max}, plotLabels, device, decimals);
  widget.vehicleStyle = true;
  widget.setLabel(document.getElementById(device.name + '-label'));
  widgets[device.name.toLowerCase()] = [widget];
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
  overviewWidget = new OverviewWidget(document.getElementById('overview'));

  // Speed tab
  addTab('Speed');
  appendNewElement('Speed-layout', '<h2><span id="target-speed-label" style="color:red"></span></h2>');
  addDriverInfo({name: 'Speed', htmlName: 'Speed'}, '[km/h]', -20.0, 20.0);

  addTab('Steering');
  addDriverInfo({name: 'Steering'}, '[rad]', -0.001, 0.001);

  addTab('Encoders');
  addDriverInfo({name: 'Encoders'}, '[%]', 0, 0.001);

  addTab('Brake');
  addDriverInfo({name: 'Brake'}, '[%]', 0, 0.001);

  addTab('Throttle');
  appendNewElement('Throttle-layout', '<h2><span id="throttle-label"></span></h2>');
  addDriverInfo({name: 'Throttle'}, '[%]', 0, 0.001);

  addTab('RPM');
  appendNewElement('RPM-layout', '<h2><span id="rpm-label"></span></h2>');
  addDriverInfo({name: 'RPM'}, '[RPM]', 0, 0.001);

  // Parse the devices once to prepare the device type list.
  configureDevices(data);

  // Set the focus on the Overview tab
  menuTabCallback('overview');
  openMenu();
}

function update(data) {
  if (data.vehicle != null) {
    Object.keys(data.vehicle).forEach(function(key) {
      key = key.replace(/&quot;/g, '"');

      if (key === 'control-mode') {
        updateControlMode(data.vehicle[key]);
        return;
      }

      const checkbox = document.getElementById(key + '-enable-checkbox');
      // update only if tab is visible?
      if (!checkbox || !checkbox.checked)
        return;

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
            widgets[key].forEach(function(widget) {
              attributeValue.forEach(function(u) {
                widget.addValue({'x': u.time, 'y': u.value });
              });
              widget.refresh();
            });
          }
        });
      } else if (key === 'steering' || key === 'encoders' || key === 'brake' || key === 'throttle' || key === 'rpm') {
        const time = data.vehicle[key]['time'];
        const measurement = data.vehicle[key]['value'];
        widgets[key].forEach(function(widget) {
          widget.addValue({'x': time, 'y': measurement });
          widget.refresh();
        });
      }
    });
  }

  if (data.devices)
    updateDevices(data);
}

function receive(message, _robot) {
  let data = '';
  if (message.indexOf('configure ') === 0) {
    try {
      data = JSON.parse(message.substring(10));
    } catch (e) {
      console.log('C to JS protocol error:');
      console.log(e);
      console.log(message);
    }
    if (data)
      configure(data);
  } else if (message.indexOf('configure-vehicle ') === 0) {
    try {
      data = JSON.parse(message.substring(17));
    } catch (e) {
      console.log('C to JS protocol error:');
      console.log(e);
      console.log(message);
    }
    if (data) {
      overviewWidget.setStaticInformation(data);
      overviewWidget.resize();
    }
  } else if (message.indexOf('update ') === 0) {
    try {
      data = JSON.parse(message.substring(7));
    } catch (e) {
      console.log('C to JS protocol error:');
      console.log(e);
      console.log(message);
    }
    if (data)
      update(data);
  } else
    console.log("Unexpected message received: '" + message + "'");

  pushDeviceCommands();

  if (Object.keys(commands).length !== 0) {
    window.robotWindow.send(commands.join());
    commands = [];
  }
}

window.onresize = function() {
  overviewWidget.resize();
};

window.onload = function() {
  // get .device-content { with: XXXpx; height: XXXpx; }
  const tmp = document.createElement('div');
  tmp.classList.add('device-content');
  document.body.appendChild(tmp);
  const deviceContentWidth = window.getComputedStyle(document.body.lastChild).getPropertyValue('width').replace(/px$/g, '');
  const deviceContentHeight = window.getComputedStyle(document.body.lastChild).getPropertyValue('height').replace(/px$/g, '');
  document.body.removeChild(document.body.lastChild);

  window.robotWindow = webots.window();
  window.robotWindow.setTitle('Vehicle robot window');
  window.robotWindow.receive = receive;
  window.robotWindow.send('configure { "imageMaxWidth": ' + deviceContentWidth + ', "imageMaxHeight": ' + deviceContentHeight + ' }');
};
