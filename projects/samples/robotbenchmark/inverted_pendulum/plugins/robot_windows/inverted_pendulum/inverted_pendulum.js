/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */
$('#infotabs').tabs();

var benchmarkName = 'Inverted Pendulum';
var timeString;
var invertedPendulumTime;

webots.window('inverted_pendulum').receive = function(message, robot) {
  if (message.startsWith('time:')) {
    invertedPendulumTime = parseFloat(message.substr(5));
    timeString = parseSecondsIntoReadableTime(invertedPendulumTime);
    $('#time-display').html(timeString);
  } else if (message.startsWith('force:')) {
    var f = parseFloat(message.substr(6));
    $('#force-display').html(f.toFixed(2));
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, invertedPendulumTime, metricToString)) {
      $('#time-display').css('color', 'red');
      $('#force-display').css('color', 'red');
    }
  } else if (message.startsWith('record:OK:')) {
    $('#time-display').css('font-weight', 'bold');
    $('#force-display').css('font-weight', 'bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#time-display').css('color', 'red');
    $('#force-display').css('color', 'red');
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseSecondsIntoReadableTime(parseFloat(s));
  }

  function parseSecondsIntoReadableTime(timeInSeconds) {
    var minutes = timeInSeconds / 60;
    var absoluteMinutes = Math.floor(minutes);
    var m = absoluteMinutes > 9 ? absoluteMinutes : '0' + absoluteMinutes;
    var seconds = (minutes - absoluteMinutes) * 60;
    var absoluteSeconds = Math.floor(seconds);
    var s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
    var cs = Math.floor((seconds - absoluteSeconds) * 100);
    if (cs < 10)
      cs = '0' + cs;
    return m + ':' + s + ':' + cs;
  }
};
