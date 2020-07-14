webots.window('info').receive = function(message, robot) {
  if (message.startsWith('test')) {
    document.querySelector("#text-display").innerHTML = message;
    this.send("Answer: " + message, robot);
  } else
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");
}
