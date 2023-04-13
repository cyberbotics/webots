import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';

/* global sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */

window.robotWindow = new RobotWindow();
const benchmarkName = 'Humanoid Marathon';
let distance = 0.0;

window.robotWindow.receive = function(message, robot) {
  if (message.startsWith('run:')) {
    const values = message.split(':');
    distance = parseFloat(values[1]);
    const battery = parseFloat(values[2]);
    document.getElementById('distance-display').innerHTML = distance.toFixed(3);
    document.getElementById('battery-display').innerHTML = battery.toFixed(2);
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, distance, metricToString)) {
      document.getElementById('distance-display').style.color = 'red';
      document.getElementById('battery-display').style.color = 'red';
    }
  } else if (message.startsWith('record:OK:')) {
    document.getElementById('distance-display').style.fontWeight = 'bold';
    document.getElementById('battery-display').style.fontWeight = 'bold';
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    document.getElementById('distance-display').style.color = 'red';
    document.getElementById('battery-display').style.color = 'red';
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseFloat(s).toFixed(3) + ' m';
  }
};
