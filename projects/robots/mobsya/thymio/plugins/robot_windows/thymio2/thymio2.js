import RobotWindow from '../../../../../../../resources/web/wwi/RobotWindow.js';

let robotName = "Thymio II";
window.robot_window = new RobotWindow("thymio2");
console.log(robot_window);

window.onload = function() {

window.robot_window.receive = function(value, robot) {
    if (value.indexOf("configure ") === 0) {
      try {
        var configure = JSON.parse(value.substring(10));
        console.log("configure" + configure);
      } catch(e) {
        console.log(e);
        console.log("In: " + value);
      }
      robotLayout(configure);
    } else { // sensor values
      var values = value.split(" ");
      document.getElementById("prox.horizontal.0").innerHTML = values[0];
      document.getElementById("prox.horizontal.1").innerHTML = values[1];
      document.getElementById("prox.horizontal.2").innerHTML = values[2];
      document.getElementById("prox.horizontal.3").innerHTML = values[3];
      document.getElementById("prox.horizontal.4").innerHTML = values[4];
      document.getElementById("prox.horizontal.5").innerHTML = values[5];
      document.getElementById("prox.horizontal.6").innerHTML = values[6];
      document.getElementById("prox.ground.0").innerHTML = values[7];
      document.getElementById("prox.ground.1").innerHTML = values[8];
    }
  }
  window.robot_window.send("configure", robotName); //TODO: receive configure
}

window.addEventListener("mouseup", onmouseup);

function get_button(x,y) {
  if (x >= 114 && x <= 141) {
    if (y >=72 && y <= 96)
      return "forward";
    else if (y >= 107 && y <= 136)
      return "center";
    else if (y >= 146 && y <= 171)
      return "backward";
  } else if (y >= 107 && y <= 136) {
    if (x >= 79 && x <= 104)
      return "left";
    else if (x >= 153 && x <= 178)
      return "right"
  }
  return "";
}

window.show_click= function(event) {
  var mouse_click = document.getElementById("mouse_click");
  mouse_click.style.left = (event.clientX - mouse_click.width/2) + "px";
  mouse_click.style.top = (event.clientY - mouse_click.height/2) + "px";
  mouse_click.style.display = 'block';
}
window.thymio_onmousedown = function(event) {
  var button = get_button(event.offsetX, event.offsetY);
  if (button) {
    show_click(event);
    window.robot_window.send("mousedown " + button, robotName);
  }
}
window.clap_onmousedown = function(event) {
  show_click(event);
  window.robot_window.send("mousedown clap", robotName);
}

window.tap_onmousedown = function(event) {
  show_click(event);
  window.robot_window.send("mousedown tap", robotName);
}

window.onmouseup = function(event) {
  document.getElementById("mouse_click").style.display='none';
  window.robot_window.send("mouseup", robotName);
}

window.robotLayout = function(configure) {
  window.robot_window.setTitle("Robot: " + configure.name, configure.name);
}
