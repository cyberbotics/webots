window.onload = function() {
  window.robotWindow = webots.window("thymio2");
  window.robotWindow.receive = function(value, robot) {
    if (value.indexOf("configure ") === 0) {
      try {
        var configure = JSON.parse(value.substring(10));
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
  window.robotWindow.send("configure");
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

function show_click(event) {
  var mouse_click = document.getElementById("mouse_click");
  mouse_click.style.left = (event.clientX - mouse_click.width/2) + "px";
  mouse_click.style.top = (event.clientY - mouse_click.height/2) + "px";
  mouse_click.style.display = 'block';
}

function thymio_onmousedown(event) {
  var button = get_button(event.offsetX, event.offsetY);
  if (button) {
    show_click(event);
    window.robotWindow.send("mousedown " + button);
  }
}

function clap_onmousedown(event) {
  show_click(event);
  window.robotWindow.send("mousedown clap");
}

function tap_onmousedown(event) {
  show_click(event);
  window.robotWindow.send("mousedown tap");
}

function onmouseup(event) {
  document.getElementById("mouse_click").style.display='none';
  window.robotWindow.send("mouseup");
}

function robotLayout(configure) {
  window.robotWindow.setTitle("Robot: " + configure.name, configure.name);
}
