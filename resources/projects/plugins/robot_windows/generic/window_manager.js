/* global robotName: false */
/* global DeviceWidget: false */
/* exported menuTabCallback */
/* exported openMenu */
/* exported closeMenu */
/* exported configureDevices */
/* exported addSettingsTab */
/* exported parseJSONMessage */
/* exported enableAllDevicesCallback */
/* exported setupWindow */

var windowIsHidden;
var widgets = {}; // Dictionary {deviceName -> DeviceWidget }

function menuTabCallback(deviceType) {
  let i, x, tablinks;
  x = document.getElementsByClassName('devices-container');
  for (i = 0; i < x.length; ++i)
    x[i].style.display = 'none';
  tablinks = document.getElementsByClassName('menu-button');
  for (i = 0; i < tablinks.length; ++i)
    tablinks[i].className = tablinks[i].className.replace(' menu-button-selected', '');
  document.getElementById(deviceType).style.display = 'block';
  document.getElementById(deviceType + '-menu-button').className += ' menu-button-selected';

  // force widgets refresh when they are shown.
  Object.keys(widgets).forEach(function(deviceName) {
    const widget = widgets[deviceName];
    if (widget && widget.device.type === deviceType)
      widget.refresh();
  });
}

function openMenu() {
  document.getElementById('menu').style.display = 'flex';
  document.getElementById('menu-open-button').style.display = 'none';
  document.getElementById('content').style.marginLeft = document.getElementById('menu').offsetWidth + 'px';
}

function closeMenu() {
  document.getElementById('menu-open-button').style.display = 'inline';
  document.getElementById('menu').style.display = 'none';
  document.getElementById('content').style.marginLeft = '0px';
}

function appendNewElement(id, newElement) {
  const tmp = document.createElement('tmp');
  tmp.innerHTML = newElement;
  const container = document.getElementById(id);
  container.appendChild(tmp.firstChild);
  return container.childNodes[container.childNodes.length - 1];
}

function addTab(type, isDevice = true) {
  if (document.getElementById(type))
    return; // check if already exists

  let buttonsDiv;
  if (isDevice && type !== 'RotationalMotor' && type !== 'LinearMotor' && type !== 'DifferentialWheels') {
    buttonsDiv = '<div id="' + type + '-buttons" class="device-buttons">' +
      '<input type="button" value="Disable all" id="' + type + '-disable-button" class="device-button" onclick="enableAllDevicesCallback(\'' + type + '\', false)"/>' +
      '<input type="button" value="Enable all" id="' + type + '-enable-button" class="device-button" onclick="enableAllDevicesCallback(\'' + type + '\', true)"/>' +
    '</div>';
  } else
    buttonsDiv = '';

  appendNewElement('content',
    '<div id="' + type + '" class="devices-container animate-left">' +
      '<h1>' + type + '</h1>' + buttonsDiv +
      '<section id="' + type + '-layout" class="devices-layout"/>' +
    '</div>'
  );
  appendNewElement('menu',
    '<button id="' + type + '-menu-button" class="menu-button tablink" onclick="menuTabCallback(\'' + type + '\')">' + type + '</button>'
  );
}

function enableAllDevicesCallback(deviceType, enabled) {
  Object.keys(widgets).forEach(function(deviceName) {
    if (!deviceType || widgets[deviceName].device.type === deviceType)
      widgets[deviceName].enable(enabled);
  });
};

function configureDevices(data) {
  if (data.devices == null) {
    document.getElementById('no-controller-label').innerHTML = 'No devices.';
    closeMenu();
    return;
  }

  // Parse the devices once to prepare de device type list.
  let deviceTypes = [];
  if (data.type === 'DifferentialWheels')
    deviceTypes.push(data.type);
  data.devices.forEach(function(device) {
    if (DeviceWidget.supportedDeviceTypes.indexOf(device.type) >= 0 && deviceTypes.indexOf(device.type) < 0)
      deviceTypes.push(device.type);
  });

  // Sort the deviceTypes alphabetically.
  deviceTypes.sort(alphabetical);

  // Create a device type container per device types.
  deviceTypes.forEach(function(deviceType) {
    addTab(deviceType);
  });

  // Create a device container per device.
  if (data.type === 'DifferentialWheels') {
    const deviceName = robotName.replace(/&quot;/g, '"');
    DeviceWidget.createWidget(data.basicTimeStep, {'name': deviceName, 'type': data.type });
  }
  data.devices.forEach(function(device) {
    if (DeviceWidget.supportedDeviceTypes.indexOf(device.type) >= 0) {
      device.htmlName = device.name;
      device.name = device.name.replace(/&quot;/g, '"');
      device.model = device.model.replace(/&quot;/g, '"');
      DeviceWidget.createWidget(data.basicTimeStep, device);
    }
  });

  widgets = Object.assign({}, widgets, DeviceWidget.widgets);
  return deviceTypes;
}

function addSettingsTab() {
  addTab('Settings', false);
  let div = '<div id="settings" class="settings">';
  div += '<h2>Generic robot window settings</h2>';
  div += '<div class="settings-content">Refresh rate: <input type="number" id="refresh-rate-number" value="32" min="2" max="2000" step="8"> ms</div>';
  div += '<div class="settings-content"><input type="button" value="Disable all devices" id="disable-all-button" class="device-button" onclick="enableAllDevicesCallback(null, false)"/></div>';
  div += '</div>';
  appendNewElement('Settings-layout', div);
  document.getElementById('refresh-rate-number').addEventListener('input', function(e) {
    window.robotWindow.send('refresh-rate ' + e.target.value);
  });
}

function setupWindow() {
  // get .device-content { with: XXXpx; height: XXXpx; }
  const tmp = document.createElement('div');
  tmp.classList.add('device-content');
  document.body.appendChild(tmp);
  const deviceContentWidth = window.getComputedStyle(document.body.lastChild).getPropertyValue('width').replace(/px$/g, '');
  const deviceContentHeight = window.getComputedStyle(document.body.lastChild).getPropertyValue('height').replace(/px$/g, '');
  document.body.removeChild(document.body.lastChild);

  windowIsHidden = document.visibilityState === 'hidden';
  window.robotWindow.send('configure { "imageMaxWidth": ' + deviceContentWidth + ', "imageMaxHeight": ' + deviceContentHeight + ', "hidden": ' + (windowIsHidden ? 1 : 0) + ' }');
  document.addEventListener('visibilitychange', function() {
    windowIsHidden = document.visibilityState === 'hidden';
    window.robotWindow.send('window ' + (windowIsHidden ? 'hidden' : 'visible'));
  });
}

function alphabetical(a, b) {
  const A = a.toLowerCase();
  const B = b.toLowerCase();
  if (A < B)
    return -1;
  else if (A > B)
    return 1;
  else
    return 0;
}

function parseJSONMessage(message) {
  let data;
  try {
    data = JSON.parse(message);
  } catch (e) {
    console.log('C to JS protocol error:');
    console.log(e);
    console.log(message);
  }
  return data;
}
