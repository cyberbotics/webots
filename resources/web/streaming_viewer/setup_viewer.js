import {webots} from "./../wwi/webots.js";

/* global webots: false */

var view = null;
var ipInput = null;
var portInput = null;
var connectButton = null;
var modeSelect = null;
var broadcast = null;

var mobileDevice = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
if (mobileDevice) {
  let head = document.getElementsByTagName('head')[0];
  let jqueryTouch = document.createElement('script');
  jqueryTouch.setAttribute('type', 'text/javascript');
  jqueryTouch.setAttribute('src', 'https://www.cyberbotics.com/jquery-ui/1.11.4/jquery.ui.touch-punch.min.js');
  head.appendChild(jqueryTouch);

  var mobileCss = document.createElement('link');
  mobileCss.setAttribute('rel', 'stylesheet');
  mobileCss.setAttribute('type', 'text/css');
  mobileCss.setAttribute('href', 'https://www.cyberbotics.com/wwi/R2021a/wwi_mobile.css');
  head.appendChild(mobileCss);
}

function init() {
  ipInput = document.getElementById('IPInput');
  connectButton = document.getElementById('ConnectButton');
  modeSelect = document.getElementById('mode');
  broadcast = document.getElementById('broadcast')
}

function connect() {
  // This `streaming viewer` setups a broadcast streaming where the simulation is shown but it is not possible to control it.
  // For any other use, please refer to the documentation:
  // https://www.cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website
  let playerDiv = document.getElementById('playerDiv');
  view = new webots.View(playerDiv, mobileDevice);
  view.broadcast = broadcast.checked;
  view.setTimeout(-1); // disable timeout that stops the simulation after a given time
  const streamingMode = modeSelect.options[modeSelect.selectedIndex].value;
<<<<<<< .merge_file_7c6ldo
<<<<<<< .merge_file_mOAfy0
  view.open('ws://' + ipInput.value + ':' + portInput.value, streamingMode);
=======
  view.open(ipInput.value, streamingMode);
>>>>>>> .merge_file_qLeKX6
=======
  view.open(ipInput.value, streamingMode);
>>>>>>> .merge_file_kn6A6s
  view.onquit = disconnect;
  connectButton.value = 'Disconnect';
  connectButton.onclick = disconnect;
  ipInput.disabled = true;
<<<<<<< .merge_file_7c6ldo
<<<<<<< .merge_file_mOAfy0
  portInput.disabled = true;
=======
>>>>>>> .merge_file_qLeKX6
=======
>>>>>>> .merge_file_kn6A6s
  modeSelect.disabled = true;
  broadcast.disabled = true;
}

function disconnect() {
  view.close();
  view = null;
  let playerDiv = document.getElementById('playerDiv');
  playerDiv.innerHTML = null;
  connectButton.value = 'Connect';
  connectButton.onclick = connect;
  ipInput.disabled = false;
<<<<<<< .merge_file_7c6ldo
<<<<<<< .merge_file_mOAfy0
  portInput.disabled = false;
=======
>>>>>>> .merge_file_qLeKX6
=======
>>>>>>> .merge_file_kn6A6s
  modeSelect.disabled = false;
  broadcast.disabled = false;
}

window.addEventListener('load', init, false);
document.getElementById('ConnectButton').addEventListener('click', connect);
