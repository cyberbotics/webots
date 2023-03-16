import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';
/* global sendBenchmarkRecord, showBenchmarkRecord, showBenchmarkError */

window.robotWindow = new RobotWindow();
const benchmarkName = 'Square Path';
let benchmarkPerformance = 0;

// recovers the bottom canvas, used for everything but the robot
const squareContext = document.getElementById('square-canvas').getContext('2d');

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
const robotContext = document.getElementById('robot-canvas').getContext('2d');
robotContext.fillStyle = '#000000';
robotContext.textAlign = 'center';
robotContext.textBaseline = 'middle';
robotContext.font = '8px Georgia';

// path is drawn in red
squareContext.fillStyle = '#FF0000';

let lastX = 0;
let lastY = 0;

window.robotWindow.receive = function(message, robot) {
  // updates the metric
  if (message.startsWith('update:')) {
    let segmentIndex = 7;
    for (let i = 0; i < 4; ++i) {
      const s = message.substr(segmentIndex, 6);
      segmentIndex += 7;
      document.getElementById('segment' + i + '-display').innerHTML = metricToString(s);
    }

    benchmarkPerformance = parseFloat(message.substr(segmentIndex));
    document.getElementById('average-display').innerHTML = metricToString(benchmarkPerformance);

  // adds a point to the path
  } else if (message.startsWith('point:')) {
    const x = 50 + parseInt(message.substr(11, 4));
    const y = 150 - parseInt(message.substr(6, 4));

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
      document.getElementById('average-display').style.color = 'red';
      for (let i = 0; i < 4; ++i)
        document.getElementById('segment' + i + '-display').style.color = 'red';
    }
  } else if (message.startsWith('record:OK:')) {
    document.getElementById('average-display').style.fontWeight = 'bold';
    for (let i = 0; i < 4; ++i)
      document.getElementById('segment' + i + '-display').style.fontWeight = 'bold';
    showBenchmarkRecord(message, benchmarkName, metricToString);
  } else if (message.startsWith('record:Error:')) {
    document.getElementById('average-display').style.color = 'red';
    for (let i = 0; i < 4; ++i)
      document.getElementById('segment' + i + '-display').style.color = 'red';
    showBenchmarkError(message, benchmarkName);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");

  function metricToString(value) {
    return (100 * value).toFixed(2) + '%';
  }
};
