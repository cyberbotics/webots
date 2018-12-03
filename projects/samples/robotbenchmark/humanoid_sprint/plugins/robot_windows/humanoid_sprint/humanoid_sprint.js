$('#infotabs').tabs();

var benchmarkName = "Humanoid Sprint";
var humanoid_sprint_time_string;
var humanoid_sprint_time;

webots.window('humanoid_sprint').receive = function(message, robot) {
  if (message.startsWith('time:')) {
    humanoid_sprint_time = parseFloat(message.substr(5));
    humanoid_sprint_time_string = parseSecondsIntoReadableTime(humanoid_sprint_time);
    $('#time-display').html(humanoid_sprint_time_string);
  } else if (message == 'stop') {
    if (typeof sendBenchmarkRecord === "undefined" || !sendBenchmarkRecord(robot, this, benchmarkName, -humanoid_sprint_time, metricToString))
      $('#time-display').css('color','red');
  } else if (message.startsWith('record:OK:')) {
    $('#time-display').css('font-weight','bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#time-display').css('color','red');
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(s) {
    return parseSecondsIntoReadableTime(-parseFloat(s));
  }

  function parseSecondsIntoReadableTime(s) {
    var hours = s / 3600;
    var absoluteHours = Math.floor(hours);
    var minutes = (hours - absoluteHours) * 60;
    var absoluteMinutes = Math.floor(minutes);
    var m = absoluteMinutes > 9 ? absoluteMinutes : '0' +  absoluteMinutes;
    var seconds = (minutes - absoluteMinutes) * 60;
    var absoluteSeconds = Math.floor(seconds);
    var s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
    var cs = Math.floor((seconds - absoluteSeconds) * 100);
    if (cs < 10)
      cs = '0' + cs;
    return m + ':' + s + ':' + cs;
  }
}
