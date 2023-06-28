import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';

/* global sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */

window.robotWindow = new RobotWindow();
const benchmarkName = 'Humanoid Sprint';
let humanoidSprintTimeString;
let humanoidSprintTime;

window.robotWindow.receive = function(message, robot) {
  if (message.startsWith('time:')) {
    humanoidSprintTime = parseFloat(message.substr(5));
    humanoidSprintTimeString = parseSecondsIntoReadableTime(humanoidSprintTime);
    document.getElementById('time-display').innerHTML = humanoidSprintTimeString;
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, -humanoidSprintTime, metricToString))
      document.getElementById('time-display').style.color = 'red';
  } else if (message.startsWith('record:OK:')) {
    document.getElementById('time-display').style.fontWeight = 'bold';
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    document.getElementById('time-display').style.color = 'red';
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseSecondsIntoReadableTime(-parseFloat(s));
  }

  function parseSecondsIntoReadableTime(timeInSeconds) {
    const minutes = timeInSeconds / 60;
    const absoluteMinutes = Math.floor(minutes);
    const m = absoluteMinutes > 9 ? absoluteMinutes : '0' + absoluteMinutes;
    const seconds = (minutes - absoluteMinutes) * 60;
    const absoluteSeconds = Math.floor(seconds);
    const s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
    let cs = Math.floor((Math.round(seconds * 100) / 100 - absoluteSeconds) * 100);
    if (cs < 10)
      cs = '0' + cs;
    return m + ':' + s + ':' + cs;
  }
};
