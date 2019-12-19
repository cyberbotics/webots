/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */
$('#infotabs').tabs();

var benchmarkName = 'Pick and Place';
var timeValue = 0;

webots.window('pick_and_place').receive = function(message, robot) {
  // updates the metric
  if (message.startsWith('update:')) {
    timeValue = parseFloat(message.substr(7));
    $('#time-display').html(parseSecondsIntoReadableTime(timeValue));
  } else if (message === 'stop') {
    $('#performance-display').html(parseSecondsIntoReadableTime(timeValue));
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, -timeValue, metricToString))
      $('#time-display').css('color', 'red');
  } else if (message.startsWith('record:OK:')) {
    $('#time-display').css('color', 'bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#time-display').css('color', 'red');
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseSecondsIntoReadableTime(-parseFloat(s));
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
