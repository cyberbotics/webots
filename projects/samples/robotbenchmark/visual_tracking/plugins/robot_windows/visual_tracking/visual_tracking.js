/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */
$('#infotabs').tabs();

var benchmarkName = 'Visual tracking';
var hitRateString;
var hitRate;

webots.window('visual_tracking').receive = function(message, robot) {
  if (message.startsWith('hits:')) {
    var rate = message.substr(5);
    var hitValues = rate.split('/');
    $('#hits-display').html(zeroFilledInteger(hitValues[0], 3));
    $('#frames-display').html(zeroFilledInteger(hitValues[1], 3));
    hitRate = hitValues[0] / hitValues[1];
    hitRateString = (100 * hitRate).toFixed(2);
    $('#rate-display').html(hitRateString);
  } else if (message.startsWith('setup:')) {
    var setup = message.substr(6);
    var values = setup.split(';');
    $('#meter-error-display').html(values[0]);
    $('#frame-step-display').html(values[1]);
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, hitRate, metricToString)) {
      $('#rate-display').css('color', 'red');
      $('#hits-display').css('color', 'red');
      $('#frames-display').css('color', 'red');
    }
  } else if (message.startsWith('record:OK:')) {
    $('#rate-display').css('font-weight', 'bold');
    $('#hits-display').css('font-weight', 'bold');
    $('#frames-display').css('font-weight', 'bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#rate-display').css('color', 'red');
    $('#hits-display').css('color', 'red');
    $('#frames-display').css('color', 'red');
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
