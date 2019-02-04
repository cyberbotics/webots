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
  var isVirtual = false;
  if (message.startsWith('virtual_markers:'))
    isVirtual = true;
  var toSlice = isVirtual ? 'virtual_markers:'.length : 'markers:'.length;
  var names = message.slice(toSlice).split(" ");
  var div = isVirtual ? document.getElementById('virtual_markers') : document.getElementById('markers');
  div.innerHTML = ''
  for (var i = 0; i < names.length; i++) {
    div.innerHTML += '<div class="markerDiv">';
    div.innerHTML += '<input type="checkbox" class="visibilityCheckbox" title="Show/hide this marker." marker="' + names[i] + '" onclick="checkboxCallback(this)"' + (isVirtual ? '' : ' checked') + '/>';
    div.innerHTML += ' ' + names[i];
    div.innerHTML += '<input type="range" class="radiusSlider" min="0.001" max="0.1" step = "0.001" value="0.01" data-show-value="true" class="slider" title="Radius of the marker." marker="' + names[i] + '" onchange="sliderCallback(this)"/>';
    div.innerHTML += '<span id = "slider_value_' + names[i] + '">0.001</span>';
    div.innerHTML += '<input type="color" class="colorSelector" marker="' + names[i] + '" value="#ff0000" onchange="colorCallback(this)"/>';
    div.innerHTML += '</div>';
  }
}
