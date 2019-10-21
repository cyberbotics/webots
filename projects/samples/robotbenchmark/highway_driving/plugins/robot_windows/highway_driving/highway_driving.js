/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */
$('#infotabs').tabs();

var benchmarkName = 'Highway Driving';
var timeValue = 0;
var distanceValue = 0;

function updateSensorsVisualization() {
  webots.window('highway_driving').send('sensors visualization:' + document.getElementById('sensors_visualization_checkbox').checked, webots.window('highway_driving').robot);
}

webots.window('highway_driving').receive = function(message, robot) {
  if (this.robot === undefined) {
    this.robot = robot;
    updateSensorsVisualization();
  }

  // updates the metric
  if (message.startsWith('update:')) {
    var values = message.substr(7).trim().split(' ');
    timeValue = parseFloat(values[0]);
    distanceValue = parseFloat(values[1]);
    $('#time-display').html(parseSecondsIntoReadableTime(timeValue));
    $('#distance-display').html(distanceValue.toFixed(3) + 'm');
  } else if (message === 'stop') {
    $('#performance-display').html(distanceValue);
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, distanceValue, metricToString)) {
      $('#time-display').css('color', 'red');
      $('#distance-display').css('color', 'red');
      $('#performance-display').css('color', 'red');
    }
  } else if (message.startsWith('record:OK:')) {
    $('#time-display').css('color', 'bold');
    $('#distance-display').css('color', 'bold');
    $('#performance-display').css('color', 'bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#time-display').css('color', 'red');
    $('#distance-display').css('color', 'red');
    $('#performance-display').css('color', 'red');
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseFloat(s).toFixed(3) + ' m';
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
