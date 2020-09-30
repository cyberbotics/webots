/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */
$('#infotabs').tabs();

var benchmarkName = 'Robot Programming';
var benchmarkPerformance = 0;

if (window.navigator.platform.startsWith('Mac'))
  $('#saveShortcut').html('Cmd-S');

webots.window('robot_programming').receive = function(message, robot) {
  if (message.startsWith('percent:'))
    $('#achievement').html(metricToString(parseFloat(message.substr(8))));
  else if (message.startsWith('stop:')) {
    benchmarkPerformance = parseFloat(message.substr(5));
    $('#achievement').html(metricToString(benchmarkPerformance));
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, benchmarkPerformance, metricToString))
      $('#achievement').css('color', 'red');
  } else if (message.startsWith('record:OK:')) {
    $('#achievement').css('font-weight', 'bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#achievement').css('color', 'red');
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(metric) {
    return (metric * 100).toFixed(2) + '%';
  }
};
