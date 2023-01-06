/* exported dropDownMenu */
/* exported onEnableAll */
/* exported wifiConnect */
/* exported wifiDisconnect */

import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';

window.onload = function() {
  let progressBar = document.getElementById('uploadProgressBar');
  progressBar.style.visibility = 'hidden';
  window.robotWindow = new RobotWindow();
  window.robotWindow.receive = function(message, robot) {
    if (message.indexOf('configure ') === 0) {
      let configure;
      try {
        configure = JSON.parse(message.substring(10));
      } catch (e) {
        console.log(e);
        console.log('In: ' + message);
      }
      robotLayout(configure);
    } else if (message.indexOf('ports') === 0) {
      updateDropDownMenu(document.getElementById('mode'), message);
      updateDropDownMenu(document.getElementById('upload'), message);
    } else if (message.startsWith('upload ')) {
      let uploadCommand = message.substring(7);
      let progressBar = document.getElementById('uploadProgressBar');
      if (uploadCommand === 'complete') {
        progressBar.className = 'uploadProgressBarCompleted';
        progressBar.value = 100;
        setTimeout(
          function() {
            progressBar.style.visibility = 'hidden';
          },
          3000
        );
      } else if (uploadCommand === 'reset') {
        progressBar.className = 'uploadProgressBarReset';
        progressBar.value = 100;
        progressBar.style.visibility = 'visible';
      } else {
        progressBar.className = 'uploadProgressBarUpload';
        progressBar.value = uploadCommand;
        progressBar.style.visibility = 'visible';
      }
    } else if (message.indexOf('update ') === 0) {
      let data = JSON.parse(message.substring(7));
      if (data && data.devices.hasOwnProperty('camera') && data.devices['camera'].hasOwnProperty('image'))
        document.getElementById('camera').src = data.devices['camera']['image'] + '#' + new Date().getTime();
    } else { // sensor values
      let values = message.split(' ');
      document.getElementById('ps0').innerHTML = values[0];
      document.getElementById('ps1').innerHTML = values[1];
      document.getElementById('ps2').innerHTML = values[2];
      document.getElementById('ps3').innerHTML = values[3];
      document.getElementById('ps4').innerHTML = values[4];
      document.getElementById('ps5').innerHTML = values[5];
      document.getElementById('ps6').innerHTML = values[6];
      document.getElementById('ps7').innerHTML = values[7];
      document.getElementById('ls0').innerHTML = values[8];
      document.getElementById('ls1').innerHTML = values[9];
      document.getElementById('ls2').innerHTML = values[10];
      document.getElementById('ls3').innerHTML = values[11];
      document.getElementById('ls4').innerHTML = values[12];
      document.getElementById('ls5').innerHTML = values[13];
      document.getElementById('ls6').innerHTML = values[14];
      document.getElementById('ls7').innerHTML = values[15];
      document.getElementById('tof').innerHTML = values[16];
      document.getElementById('left speed').innerHTML = values[17];
      document.getElementById('right speed').innerHTML = values[18];
      document.getElementById('left wheel position').innerHTML = values[19].replace(/_/g, ' ');
      document.getElementById('right wheel position').innerHTML = values[20].replace(/_/g, ' ');
      document.getElementById('accelerometer x').innerHTML = values[21];
      document.getElementById('accelerometer y').innerHTML = values[22];
      document.getElementById('accelerometer z').innerHTML = values[23];
      document.getElementById('gyro x').innerHTML = values[24];
      document.getElementById('gyro y').innerHTML = values[25];
      document.getElementById('gyro z').innerHTML = values[26];
      if (values.length > 27) {
        // optional ground sensors available
        setGroundSensorValue('gs0', values[27]);
        setGroundSensorValue('gs1', values[28]);
        setGroundSensorValue('gs2', values[29]);
      }
    }
};
  window.robotWindow.send('configure');
  document.getElementById('file-selector').onchange = function(e) {
    let filename = e.target.files[0].name;
    if (filename.lastIndexOf('.hex') !== filename.length - 4) {
      console.log(filename + ': Unsupported file type, should end with a .hex prefix');
      return;
    }
    let reader = new FileReader();
    reader.readAsText(e.target.files[0]);
    reader.onloadend = function(event) {
      window.robotWindow.send('upload ' + selectedPort + ' ' + event.target.result);
    };
  };
  onchange = 'onFileUpload();';
};

window.setGroundSensorValue = function(id, valueString) {
  let value = parseInt(valueString);
  let elem = document.getElementById(id);
  let elemLabel = document.getElementById(id + ' label');
  if (value < 0) {
    // show label
    elem.style.backgroundColor = 'inherit';
    elem.style.borderWidth = '0px';
    elemLabel.innerHTML = id;
  } else {
    // show value
    let hexString = value.toString(16);
    elem.style.backgroundColor = '#' + hexString + hexString + hexString;
    elem.style.borderWidth = '1px';
    elemLabel.innerHTML = '';
  }
}
window.updateDropDownMenu = function(menu, value) {
  let values = value.split(' ');
  while (menu.childNodes.length)
    menu.removeChild(menu.firstChild);
  let a;
  for (let i = 1; i < values.length; i++) {
    let v = values[i];
    if (v.indexOf('\\\\.\\') === 0)
      v = v.substring(4);
    a = document.createElement('A');
    a.innerHTML = v;
    a.href = '#';
    a.setAttribute('name', values[i]);
    menu.appendChild(a);
  }
  a = document.createElement('A');
  a.innerHTML = 'Refresh';
  a.href = '#';
  a.setAttribute('name', 'refresh');
  menu.appendChild(a);
}

// Close the dropdown if the user clicks outside of it
let selectedPort = 'simulation';

window.onclick = function(event) {
  if (!event.target.classList.contains('dropdown-button'))
    hideAllDropDownMenus();
  if (event.target.nodeName === 'A') {
    let menu = event.target.parentNode.id;
    let action = event.target.getAttribute('name');
    if (action !== 'refresh') {
      if (menu === 'mode') {
        let button = document.getElementById('mode-button');
        let previousValue = button.innerHTML;
        let previousName = button.getAttribute('name');
        button.innerHTML = event.target.innerHTML + ' &#x2BC6;';
        button.setAttribute('name', event.target.getAttribute('name'));
        let mode = event.target.parentNode;
        mode.removeChild(event.target);
        let a = document.createElement('A');
        a.innerHTML = previousValue.substring(0, previousValue.length - 2);
        a.href = '#';
        a.setAttribute('name', previousName);
        mode.insertBefore(a, mode.childNodes[0]); // prepend
        if (action === 'simulation') {
          a = document.createElement('A');
          a.innerHTML = 'Refresh';
          a.href = '#';
          a.setAttribute('name', 'refresh');
          mode.appendChild(a);
          selectedPort = 'simulation';
        } else {
          mode.removeChild(mode.lastChild); // Refresh
          selectedPort = 'remote control';
          action = 'remote control ' + action;
        }
      } else if (menu === 'upload') {
        document.getElementById('file-selector').click();
        selectedPort = action;
        action = '';
      }
    }
    if (action)
      window.robotWindow.send(action);
  }
};
window.hideAllDropDownMenus = function(){
  let dropdowns = document.getElementsByClassName('dropdown-content');
  let i;
  for (i = 0; i < dropdowns.length; i++) {
    let openDropdown = dropdowns[i];
    if (openDropdown.classList.contains('show'))
      openDropdown.classList.remove('show');
  }
}
window.dropDownMenu = function(id){
  hideAllDropDownMenus();
  if (selectedPort === 'remote control' && id === 'upload') {
    console.log('Please revert to simulation before uploading HEX to the e-puck robot.');
    return;
  }
  document.getElementById(id).classList.toggle('show');
}
window.wifiConnect = function(){
  let button = document.getElementById('connect');
  button.innerHTML = 'disconnect';
  button.onclick = wifiDisconnect;
  window.robotWindow.send('connect ' + document.getElementById('ip address').value);
}
window.wifiDisconnect = function(){
  let button = document.getElementById('connect');
  button.innerHTML = 'connect';
  button.onclick = wifiConnect;
  window.robotWindow.send('disconnect');
}

window.robotLayout = function(configure){
  window.robotWindow.setTitle(configure.name);
  if (configure.model === 'GCtronic e-puck2') { // e-puck2: Wifi remote control only
    let ipAddress = document.getElementById('ip address');
    ipAddress.style.visibility = 'visible';
    let connect = document.getElementById('connect');
    connect.style.visibility = 'visible';

    let image = document.getElementById('robot image');
    image.src = 'images/e-puck2.png';

    let tof = document.getElementById('tof');
    tof.style.visibility = 'visible';
  } else { // first e-puck: use Bluetooth communication only
    let uploadButton = document.getElementById('upload hex');
    uploadButton.style.visibility = 'visible';
    let simulationButton = document.getElementById('simulation');
    simulationButton.style.visibility = 'visible';

    let image = document.getElementById('robot image');
    image.src = 'images/e-puck.png';
  }
}
window.onEnableAll = function(){
  window.robotWindow.send('enable');
}
