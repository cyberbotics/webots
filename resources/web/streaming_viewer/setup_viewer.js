let ipInput = null;
let connectButton = null;
let modeSelect = null;
let broadcast = null;

const mobileDevice = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
if (mobileDevice) {
  let head = document.getElementsByTagName('head')[0];

  let mobileCss = document.createElement('link');
  mobileCss.setAttribute('rel', 'stylesheet');
  mobileCss.setAttribute('type', 'text/css');
  mobileCss.setAttribute('href', 'https://www.cyberbotics.com/wwi/R2023b/css/wwi_mobile.css');
  head.appendChild(mobileCss);
}

function init() {
  ipInput = document.getElementById('IP-input');
  connectButton = document.getElementById('connect-button');
  modeSelect = document.getElementById('mode');
  broadcast = document.getElementById('broadcast');

  connectButton.onclick = connect;
}

function connect() {
  const defaultThumbnail = 'https://cyberbotics.com/wwi/R2023b/images/loading/default_thumbnail.png';
  const streamingMode = modeSelect.options[modeSelect.selectedIndex].value;
  const webotsView = document.getElementsByTagName('webots-view')[0];
  webotsView.onready = onConnect;
  webotsView.ondisconnect = onDisconnect;
  webotsView.connect(ipInput.value, streamingMode, broadcast.checked, mobileDevice, -1, defaultThumbnail);

  ipInput.disabled = true;
  modeSelect.disabled = true;
  broadcast.disabled = true;
  connectButton.disabled = true;
}

function onConnect() {
  connectButton.value = 'Disconnect';
  connectButton.onclick = disconnect;
  connectButton.disabled = false;
}

function onDisconnect() {
  connectButton.value = 'Connect';
  connectButton.onclick = connect;

  ipInput.disabled = false;
  modeSelect.disabled = false;
  broadcast.disabled = false;
  connectButton.disabled = false;
}

function disconnect() {
  document.getElementsByTagName('webots-view')[0].close();
}

init();
