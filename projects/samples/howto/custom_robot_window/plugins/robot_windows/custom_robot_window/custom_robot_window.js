/* global webots */
/* eslint no-unused-vars: ['error', { 'varsIgnorePattern': 'handleBodyLEDCheckBox|toggleStopCheckbox' }] */

// Log a message in the console widget.
function log(message) {
  var ul = document.getElementById('console');
  var li = document.createElement('li');
  li.appendChild(document.createTextNode(message));
  ul.appendChild(li);
}

// The window user has toggled the "Stop motors" checkbox.
// This information is sent to the robot.
function toggleStopCheckbox(obj) {
  if (obj.checked) {
    obj.parentNode.classList.add('checked');
    obj.parentNode.lastChild.innerHTML = 'Start Motors';
    window.robotWindow.send('stop motors');
    log('Stop motors.');
  } else {
    obj.parentNode.classList.remove('checked');
    obj.parentNode.lastChild.innerHTML = 'Stop Motors';
    window.robotWindow.send('release motors');
    log('Release motors.');
  }
}

// A message coming from the robot has been received.
function receive(message) {
  if (message.startsWith('ps0: ')) {
    var ps0Value = parseFloat(message.substr(5));
    var progress = document.getElementById('distanceSensorProgressBar');
    progress.value = ps0Value;
  }
}

// Initialize the Webots window class in order to communicate with the robot.
window.onload = function() {
  log('HTML page loaded.');
  window.robotWindow = webots.window();
  window.robotWindow.setTitle('Custom HTML robot window');
  window.robotWindow.receive = receive;
};
