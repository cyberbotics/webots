var robotWindow = null;

function checkboxCallback(checkbox) {
  if (checkbox.checked)
    robotWindow.send(checkbox.getAttribute('marker') + ':enable');
  else
    robotWindow.send(checkbox.getAttribute('marker') + ':disable');
}

function sliderCallback(slider) {
  var marker = slider.getAttribute('marker');
  var label = document.getElementById('slider_value_' + marker).innerHTML = slider.value;
  robotWindow.send(marker + ':radius:' + slider.value);
}

function colorCallback(color) {
  robotWindow.send(color.getAttribute('marker') + ':color:' + color.value);
}

webots.window('c3d_viewer_window').receive = function(message, robot) {
  robotWindow = this;
  var names = message.split(" ");
  for (var i = 0; i < names.length; i++) {
    document.body.innerHTML +='<input type="checkbox" title="Show/hide this marker." marker="' + names[i] + '" onclick="checkboxCallback(this)" checked/>';
    document.body.innerHTML += ' ' + names[i];
    document.body.innerHTML += '<input type="range" min="0.001" max="0.1" step = "0.001" value="0.01" data-show-value="true" class="slider" title="Radius of the marker." marker="' + names[i] + '" onchange="sliderCallback(this)">';
    document.body.innerHTML += '<span id = "slider_value_' + names[i] + '">0.001</div>';
    document.body.innerHTML += '<input type="color" marker="' + names[i] + '" value="#ff0000" onchange="colorCallback(this)"></br>';
  }
}
