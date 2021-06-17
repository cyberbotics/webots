/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */

const benchmarkName = 'Inverted Pendulum';
let timeString;
let invertedPendulumTime;

webots.window('inverted_pendulum').receive = function(message, robot) {
  if (message.startsWith('time:')) {
    invertedPendulumTime = parseFloat(message.substr(5));
    timeString = parseSecondsIntoReadableTime(invertedPendulumTime);
    document.getElementById('time-display').innerHTML = timeString;
  } else if (message.startsWith('force:')) {
    const f = parseFloat(message.substr(6));
    document.getElementById('force-display').innerHTML = f.toFixed(2);
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, invertedPendulumTime, metricToString)) {
      document.getElementById('time-display').style.color = 'red';
      document.getElementById('force-display').style.color = 'red';
    }
  } else if (message.startsWith('record:OK:')) {
    document.getElementById('time-display').style.fontWeight = 'bold';
    document.getElementById('force-display').style.fontWeight = 'bold';
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    document.getElementById('time-display').style.color = 'red';
    document.getElementById('force-display').style.color = 'red';
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseSecondsIntoReadableTime(parseFloat(s));
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
