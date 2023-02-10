import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';
/* global sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */

window.robotWindow = new RobotWindow();
const benchmarkName = 'Pit Escape';
let benchmarkPerformance = 0;

window.robotWindow.receive = function(message, robot) {
  // updates the metric
  if (message.startsWith('update:')) {
    benchmarkPerformance = message.substr(7);
    document.getElementById('performance-display').innerHTML = benchmarkPerformance;
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, benchmarkPerformance / 100, metricToString))
      document.getElementById('performance-display').style.color = 'red';
  } else if (message.startsWith('record:OK:')) {
    document.getElementById('performance-display').style.fontWeight = 'bold';
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    document.getElementById('performance-display').style.color = 'red';
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(value) {
    return (100 * value).toFixed(2) + '%';
  }
};
