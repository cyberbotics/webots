import {webots} from './../wwi/webots.js';

let view = null;
let ipInput = null;
let connectButton = null;
let modeSelect = null;
let broadcast = null;

const mobileDevice = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
if (mobileDevice) {
  let head = document.getElementsByTagName('head')[0];
  let jqueryTouch = document.createElement('script');
  jqueryTouch.setAttribute('type', 'text/javascript');
  jqueryTouch.setAttribute('src', 'https://www.cyberbotics.com/jquery-ui/1.11.4/jquery.ui.touch-punch.min.js');
  head.appendChild(jqueryTouch);

  let mobileCss = document.createElement('link');
  mobileCss.setAttribute('rel', 'stylesheet');
  mobileCss.setAttribute('type', 'text/css');
  mobileCss.setAttribute('href', 'https://www.cyberbotics.com/wwi/R2021b/wwi_mobile.css');
  head.appendChild(mobileCss);
}

function init() {
  ipInput = document.getElementById('IPInput');
  connectButton = document.getElementById('ConnectButton');
  modeSelect = document.getElementById('mode');
  broadcast = document.getElementById('broadcast');

  connectButton.onclick = connect;
}

function connect() {
  // This `streaming viewer` setups a broadcast streaming where the simulation is shown but it is not possible to control it.
  // For any other use, please refer to the documentation:
  // https://www.cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website
  let playerDiv = document.getElementById('playerDiv');
  if (!view)
    view = new webots.View(playerDiv, mobileDevice);
  view.broadcast = broadcast.checked;
  view.setTimeout(-1); // disable timeout that stops the simulation after a given time
  const streamingMode = modeSelect.options[modeSelect.selectedIndex].value;

  view.onready = _ => {
    connectButton.value = 'Disconnect';
    connectButton.onclick = disconnect;
    connectButton.disabled = false;
  };

  view.open(ipInput.value, streamingMode);
  view.onquit = disconnect;

  ipInput.disabled = true;
  modeSelect.disabled = true;
  broadcast.disabled = true;
  connectButton.disabled = true;
}

function disconnect() {
  view.close();
  let toolbar = document.getElementById('toolBar');
  if (toolbar !== 'undefined' && toolbar !== null)
    toolbar.parentNode.removeChild(toolbar);

  connectButton.value = 'Connect';
  connectButton.onclick = connect;
  ipInput.disabled = false;
  modeSelect.disabled = false;
  broadcast.disabled = false;
}

window.addEventListener('load', init, false);
