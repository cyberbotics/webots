/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */

const benchmarkName = 'Visual tracking';
let hitRateString;
let hitRate;

webots.window('visual_tracking').receive = function(message, robot) {
  if (message.startsWith('hits:')) {
    const rate = message.substr(5);
    const hitValues = rate.split('/');
    document.getElementById('hits-display').innerHTML = zeroFilledInteger(hitValues[0], 3);
    document.getElementById('frames-display').innerHTML = zeroFilledInteger(hitValues[1], 3);
    hitRate = hitValues[0] / hitValues[1];
    hitRateString = (100 * hitRate).toFixed(2);
    document.getElementById('rate-display').innerHTML = hitRateString;
  } else if (message.startsWith('setup:')) {
    const setup = message.substr(6);
    const values = setup.split(';');
    document.getElementById('frame-step-display').innerHTML = values[1];
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, hitRate, metricToString)) {
      document.getElementById('rate-display').style.color = 'red';
      document.getElementById('hits-display').style.color = 'red';
      document.getElementById('frames-display').style.color = 'red';
    }
  } else if (message.startsWith('record:OK:')) {
    document.getElementById('rate-display').style.fontWeight = 'bold';
    document.getElementById('hits-display').style.fontWeight = 'bold';
    document.getElementById('frames-display').style.fontWeight = 'bold';
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    document.getElementById('rate-display').style.color = 'red';
    document.getElementById('hits-display').style.color = 'red';
    document.getElementById('frames-display').style.color = 'red';
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function zeroFilledInteger(x, width) {
    return (new Array(width).join('0') + x).substr(-width);
  }

  function metricToString(value) {
    return (100 * value).toFixed(2) + '%';
  }
};
