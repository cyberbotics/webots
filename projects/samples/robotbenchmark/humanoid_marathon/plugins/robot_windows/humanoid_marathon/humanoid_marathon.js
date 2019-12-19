/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */
$('#infotabs').tabs();

var benchmarkName = 'Humanoid Marathon';
var distance = 0.0;

webots.window('humanoid_marathon').receive = function(message, robot) {
  if (message.startsWith('run:')) {
    var values = message.split(':');
    distance = parseFloat(values[1]);
    var battery = parseFloat(values[2]);
    $('#distance-display').html(distance.toFixed(3));
    $('#battery-display').html(battery.toFixed(2));
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, distance, metricToString)) {
      $('#distance-display').css('color', 'red');
      $('#battery-display').css('color', 'red');
    }
  } else if (message.startsWith('record:OK:')) {
    $('#distance-display').css('font-weight', 'bold');
    $('#battery-display').css('font-weight', 'bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#distance-display').css('color', 'red');
    $('#battery-display').css('color', 'red');
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseFloat(s).toFixed(3) + ' m';
  }
};
