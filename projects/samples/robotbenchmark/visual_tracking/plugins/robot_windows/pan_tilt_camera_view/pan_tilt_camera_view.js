/* global webots */
$('#robottabs').tabs();

webots.window('pan_tilt_camera_view').receive = function(message, robot) {
  // image format: image[<device name>]:<URI image data>
  if (message.startsWith('image')) {
    var label = message.substring(message.indexOf('[') + 1, message.indexOf(']'));
    var imageElement = document.getElementById('robot-' + label);
    if (imageElement != null)
      imageElement.setAttribute('src', message.substring(message.indexOf(':') + 1));

    if (label === 'camera') {
      // remove warning if needed
      var element = document.getElementById('robottab-camera-warning');
      if (element != null)
        element.parentNode.removeChild(element);
    }
  } else {
    if (message.length > 200)
      message = message.substr(0, 200);
    console.log("Received unknown message for robot '" + robot + "': '" + message + "'");
  }
};
