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
  mobileCss.setAttribute('href', 'https://www.cyberbotics.com/wwi/R2021b/css/wwi_mobile.css');
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
  const streamingMode = modeSelect.options[modeSelect.selectedIndex].value;
  document.getElementsByTagName('webots-streaming')[0].connect(ipInput.value, streamingMode, broadcast.checked, mobileDevice, changeToDisconnect, changeToConnect);

  ipInput.disabled = true;
  modeSelect.disabled = true;
  broadcast.disabled = true;
  connectButton.disabled = true;
}

function changeToDisconnect() {
  connectButton.value = 'Disconnect';
  connectButton.onclick = disconnect;
  connectButton.disabled = false;
}

function changeToConnect() {
  connectButton.value = 'Connect';
  connectButton.onclick = connect;
}

function disconnect() {
  document.getElementsByTagName('webots-streaming')[0].disconnect();

  ipInput.disabled = false;
  modeSelect.disabled = false;
  broadcast.disabled = false;
}

init();
