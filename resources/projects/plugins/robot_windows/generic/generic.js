/* global DeviceWidget: false */
/* global menuTabCallback, openMenu, closeMenu, addSettingsTab, refreshSelectedTab */
/* global configureDevices, setupWindow, windowIsHidden, parseJSONMessage */
/* eslint no-unused-vars: ["error", { "varsIgnorePattern": "Callback", "argsIgnorePattern": "^_"}] */

import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';

var robotName = '';
var commands = [];
var configured = false;
window.widgets = {}; // Dictionary {deviceName -> DeviceWidget }
window.selectedDeviceType = null;

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
  Object.keys(window.widgets[deviceType]).forEach(function(deviceName) {
    const widget = window.widgets[deviceType][deviceName];
    if (widget) {
      const checkbox = document.getElementById(widget.device.name + '-enable-checkbox');
      DeviceWidget.checkboxCallback(checkbox);
    }
  });
};

function configure(data) {
  robotName = data.name;
  window.robotWindow.setTitle(robotName + ' robot window');

  if (data.devices == null) {
    document.getElementById('no-controller-label').innerHTML = 'No devices.';
    closeMenu();
    return;
  }

  // Parse the devices once to prepare the device type list.
  const deviceTypes = configureDevices(data, true);

  addSettingsTab();

  // Set the focus on the first deviceType menu.
  if (deviceTypes.length > 0) {
    menuTabCallback(deviceTypes[0]);
    openMenu();
    document.getElementById('no-controller-label').style.display = 'none';
  } else {
    document.getElementById('no-controller-label').innerHTML = 'No devices supported in the robot window.';
    closeMenu();
  }
}

function receive(message, _robot) {
  let data = '';
  if (message.indexOf('configure ') === 0) {
    // The robot window should be configured only once.
    // However, it may happen that it receives several configure messages in the following cases:
    // - if several tabs open the same robot windows, or
    // - if a robot window is opened and refreshed before is has received the config message.
    if (configured)
      return;
    data = parseJSONMessage(message.substring(10));
    if (data) {
      configure(data);
      configured = true;
    }
  } else if (windowIsHidden)
    return;
  else if (message.indexOf('update ') === 0) {
    data = parseJSONMessage(message.substring(7));
    if (data) {
      if (DeviceWidget.updateDeviceWidgets(data, window.selectedDeviceType))
        refreshSelectedTab();
    }
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
  window.robotWindow.setTitle('Generic robot window');
  window.robotWindow.receive = receive;
  setupWindow();
};
