var robotWindow = null;

function checkboxCallback(checkbox) {
  if (checkbox.checked)
    robotWindow.send(checkbox.getAttribute('marker') + ':enable');
  else
    robotWindow.send(checkbox.getAttribute('marker') + ':disable');
}

webots.window('c3d_viewer_window').receive = function(message, robot) {
  robotWindow = this;
  names = message.split(" ");
  for (var i = 0; i < names.length; i++) {
    document.body.innerHTML +='<input type="checkbox" title="Show/hide this marker." marker="' + names[i] + '" onclick="checkboxCallback(this)" checked/>';
    document.body.innerHTML += ' ' + names[i] + '</br>';
  }
}
