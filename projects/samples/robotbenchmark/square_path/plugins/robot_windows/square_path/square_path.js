/* global webots, sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */
$('#infotabs').tabs();

var benchmarkName = 'Square Path';
var benchmarkPerformance = 0;

// recovers the bottom canvas, used for everything but the robot
var squareContext = $('#square-canvas')[0].getContext('2d');

// draws the square
squareContext.strokeStyle = '#BBBBBB';
squareContext.strokeRect(50, 50, 100, 100);

// adds segment numbers
squareContext.fillStyle = '#BBBBBB';
squareContext.font = '20px Georgia';
squareContext.textAlign = 'center';
squareContext.textBaseline = 'middle';
squareContext.fillText('1', 40, 100);
squareContext.fillText('3', 160, 100);
squareContext.fillText('2', 100, 40);
squareContext.fillText('4', 100, 165);

// recovers robot canvas, used to draw the "robot"
var robotContext = $('#robot-canvas')[0].getContext('2d');
robotContext.fillStyle = '#000000';
robotContext.textAlign = 'center';
robotContext.textBaseline = 'middle';
robotContext.font = '8px Georgia';

// path is drawn in red
squareContext.fillStyle = '#FF0000';

var lastX = 0;
var lastY = 0;

webots.window('square_path').receive = function(message, robot) {
  // updates the metric
  if (message.startsWith('update:')) {
    var i;
    var segmentIndex = 7;
    for (i = 0; i < 4; ++i) {
      var s = message.substr(segmentIndex, 6);
      segmentIndex += 7;
      $('#segment' + i + '-display').html(metricToString(s));
    }

    benchmarkPerformance = parseFloat(message.substr(segmentIndex));
    $('#average-display').html(metricToString(benchmarkPerformance));

  // adds a point to the path
  } else if (message.startsWith('point:')) {
    var x = 50 + parseInt(message.substr(11, 4));
    var y = 150 - parseInt(message.substr(6, 4));

    // if the point hasn't changed, we don't need to do anything
    if (x !== lastX || y !== lastY) {
      // adds a dot to the path
      squareContext.fillRect(x, y, 1, 1);

      // erases the last position of the robot
      robotContext.clearRect(lastX - 5, lastY - 5, 10, 10);
      // draws the robot using the circle character
      robotContext.fillText('\u25CF', x, y);

      lastX = x;
      lastY = y;
    }
  } else if (message === 'stop') {
    if (typeof sendBenchmarkRecord === 'undefined' || !sendBenchmarkRecord(robot, this, benchmarkName, benchmarkPerformance, metricToString)) {
      $('#average-display').css('color', 'red');
      for (i = 0; i < 4; ++i)
        $('#segment' + i + '-display').css('color', 'red');
    }
  } else if (message.startsWith('record:OK:')) {
    $('#average-display').css('font-weight', 'bold');
    for (i = 0; i < 4; ++i)
      $('#segment' + i + '-display').css('font-weight', 'bold');
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    $('#average-display').css('color', 'red');
    for (i = 0; i < 4; ++i)
      $('#segment' + i + '-display').css('color', 'red');
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(value) {
    return (100 * value).toFixed(2) + '%';
  }
};
