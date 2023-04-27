import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';

/* eslint no-unused-vars: ['error', { 'varsIgnorePattern': 'handleBodyLEDCheckBox|toggleStopCheckbox' }] */

// Log a message in the console widget.
window.log = function(message) {
  var ul = document.getElementById('console');
  var li = document.createElement('li');
  li.appendChild(document.createTextNode(message));
  ul.appendChild(li);
}

// The window user has toggled the "Stop motors" checkbox.
// This information is sent to the robot.
window.toggleStopCheckbox =  function(obj) {
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
function receive(message, robot) {
  if (message.startsWith('ps0: ')) {
    var ps0Value = parseFloat(message.substr(5));
    var progress = document.getElementById('distanceSensorProgressBar');
    progress.value = ps0Value;
  }
}

// Initialize the RobotWindow class in order to communicate with the robot.
window.onload = function() {
  log('HTML page loaded.');
  window.robotWindow = new RobotWindow();
  window.robotWindow.setTitle('Custom HTML robot window');
  window.robotWindow.receive = receive;
};
