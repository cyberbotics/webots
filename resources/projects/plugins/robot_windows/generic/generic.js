/* global webots: false */
/* global DeviceWidget: false */
/* global menuTabCallback, openMenu, closeMenu, addSettingsTab */
/* global configureDevices, setupWindow, windowIsHidden, parseJSONMessage */
/* global widgets */

var robotName = '';

function configure(data) {
  robotName = data.name;
  window.robotWindow.setTitle('Generic robot window [' + robotName + ']');

  if (data.devices == null) {
    document.getElementById('no-controller-label').innerHTML = 'No devices.';
    closeMenu();
    return;
  }

  // Parse the devices once to prepare the device type list.
  const deviceTypes = configureDevices(data);

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
    data = parseJSONMessage(message.substring(10));
    if (data)
      configure(data);
  } else if (windowIsHidden)
    return;
  else if (message.indexOf('update ') === 0) {
    data = parseJSONMessage(message.substring(7));
    if (data)
      DeviceWidget.updateDeviceWidgets(data, widgets);
  } else
    console.log("Unexpected message received: '" + message + "'");

  DeviceWidget.pushDeviceCommands();

  if (Object.keys(DeviceWidget.commands).length !== 0) {
    window.robotWindow.send(DeviceWidget.commands.join());
    DeviceWidget.commands = [];
  }
}

window.onload = function() {
  window.robotWindow = webots.window();
  window.robotWindow.setTitle('Generic robot window');
  window.robotWindow.receive = receive;
  setupWindow();
};
