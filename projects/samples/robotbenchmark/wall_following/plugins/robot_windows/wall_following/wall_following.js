$('#infotabs').tabs();

var benchmarkName = "Wall Following";
var performance = 0;

webots.window('wall_following').receive = function(message, robot) {
  // updates the metric
  if (message.startsWith('update:')) {
    performance = message.substr(7);
    $('#performance-display').html(performance);
  } else if (message == 'stop') {
    if (typeof sendBenchmarkRecord === "undefined" || !sendBenchmarkRecord(robot, this, benchmarkName, performance / 100, metricToString))
      $('#performance-display').css('color','red');
  } else if (message.startsWith('record:OK:')) {
    $('#performance-display').css('font-weight','bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#performance-display').css('color','red');
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(value) {
    return (100 * value).toFixed(2) + "%";
  }
}
