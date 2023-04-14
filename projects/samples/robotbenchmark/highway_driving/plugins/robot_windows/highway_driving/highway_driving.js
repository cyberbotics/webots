import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';

/* global sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */

const benchmarkName = 'Highway Driving';
let timeValue = 0;
let distanceValue = 0;
window.robotWindow = new RobotWindow();

window.updateSensorsVisualization = function() {
  window.robotWindow.send('sensors visualization:' + document.getElementById('sensors_visualization_checkbox').checked);
}

window.robotWindow.receive = function(message, robot) {
  if (this.robot === undefined) {
    this.robot = robot;
    updateSensorsVisualization();
  }

  // updates the metric
  if (message.startsWith('update:')) {
    const values = message.substr(7).trim().split(' ');
    timeValue = parseFloat(values[0]);
    distanceValue = parseFloat(values[1]);
    document.getElementById('time-display').innerHTML = parseSecondsIntoReadableTime(timeValue);
    document.getElementById('distance-display').innerHTML = distanceValue.toFixed(3) + 'm';
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, distanceValue, metricToString)) {
      document.getElementById('time-display').style.color = 'red';
      document.getElementById('distance-display').style.color = 'red';
    }
  } else if (message.startsWith('record:OK:')) {
    document.getElementById('time-display').style.color = 'bold';
    document.getElementById('distance-display').style.color = 'bold';
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    document.getElementById('time-display').style.color = 'red';
    document.getElementById('distance-display').style.color = 'red';
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseFloat(s).toFixed(3) + ' m';
  }

  function parseSecondsIntoReadableTime(timeInSeconds) {
    const minutes = timeInSeconds / 60;
    const absoluteMinutes = Math.floor(minutes);
    const m = absoluteMinutes > 9 ? absoluteMinutes : '0' + absoluteMinutes;
    const seconds = (minutes - absoluteMinutes) * 60;
    const absoluteSeconds = Math.floor(seconds);
    const s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
    let cs = Math.floor((seconds - absoluteSeconds) * 100);
    if (cs < 10)
      cs = '0' + cs;
    return m + ':' + s + ':' + cs;
  }
};
