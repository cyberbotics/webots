import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';
/* global sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */

window.robotWindow = new RobotWindow();
const benchmarkName = 'Robot Programming';
let benchmarkPerformance = 0;

if (window.navigator.platform.startsWith('Mac'))
  document.getElementById('saveShortcut').innerHTML = 'Cmd-S';

  window.robotWindow.receive = function(message, robot) {
  if (message.startsWith('percent:'))
    document.getElementById('achievement').innerHTML = metricToString(parseFloat(message.substr(8)));
  else if (message.startsWith('stop:')) {
    benchmarkPerformance = parseFloat(message.substr(5));
    document.getElementById('achievement').innerHTML = metricToString(benchmarkPerformance);
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, benchmarkPerformance, metricToString))
      document.getElementById('achievement').style.color = 'red';
  } else if (message.startsWith('record:OK:')) {
    document.getElementById('achievement').style.fontWeight = 'bold';
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    document.getElementById('achievement').style.color = 'red';
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(metric) {
    return (metric * 100).toFixed(2) + '%';
  }
};
