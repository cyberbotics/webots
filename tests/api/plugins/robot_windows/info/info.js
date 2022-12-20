import RobotWindow from 'https://cyberbotics.com/wwi/R2023b/RobotWindow.js';

window.robotWindow = new RobotWindow();

window.onload = function() {
  window.robotWindow.receive = function(message, robot) {
    if (message.startsWith('test')) {
      document.querySelector('#text-display').innerHTML = message;
      window.robotWindow.send('Answer: ' + message);
    } else
      console.log("Received unknown message for robot '" + robot + "': '" + message + "'");
  };
  window.robotWindow.send("configure");
}
