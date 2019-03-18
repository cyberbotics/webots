/* global DialogWindow */
'use strict';

function RobotWindow(parent, mobile, name) {
  DialogWindow.call(this, parent, name);
  this.name = name;
  this.panel.id = name;
  this.panel.className = 'webotsTabContainer';

  var clampedSize = DialogWindow.clampDialogSize({left: 5, top: 5, width: 400, height: 400});
  this.params.width = clampedSize.width;
  this.params.height = clampedSize.height;
  this.params.close = null;
  this.params.position = {at: 'left+5 top+5', my: 'left top', of: this.parent};
  this.params.title = 'Robot Window';

  $(this.panel).dialog(this.params).dialogExtend({maximizable: mobile});
}

RobotWindow.prototype = {
  constructor: RobotWindow,

  setProperties: function(properties) {
    $(this.panel).dialog(properties);
  },

  geometry: function() {
    var webotsTabs = this.panel.getElementsByClassName('webotsTabs');
    var activeTabIndex = -1;
    if (webotsTabs.length > 0)
      activeTabIndex = $(webotsTabs[0]).tabs('option', 'active');
    return {
      width: $(this.panel).dialog('option', 'width'),
      height: $(this.panel).dialog('option', 'height'),
      position: $(this.panel).dialog('option', 'position'),
      activeTabIndex: activeTabIndex,
      open: this.isOpen()
    };
  },

  restoreGeometry: function(data) {
    $(this.panel).dialog({
      width: data.width,
      height: data.height,
      position: data.position
    });
    var webotsTabs = this.panel.getElementsByClassName('webotsTabs');
    if (data.activeTabIndex >= 0 && webotsTabs.length > 0)
      $(webotsTabs[0]).tabs('option', 'active', data.activeTabIndex);
  },

  destroy: function() {
    this.close();
    this.panel.parentNode.removeChild(this.panel);
    this.panel = null;
  },

  setContent: function(content) {
    $(this.panel).html(content);
  },

  open: function() {
    $(this.panel).dialog('open');
  },

  isOpen: function() {
    return $(this.panel).dialog('isOpen');
  },

  close: function() {
    $(this.panel).dialog('close');
  },

  send: function(message, robot) {
    // TODO
    // webots.currentView.stream.socket.send('robot:' + robot + ':' + message);
    // if (webots.currentView.real_timeButton.style.display === 'inline') // if paused, make a simulation step
    //  webots.currentView.stream.socket.send('step'); // so that the robot controller handles the message
    // FIXME: there seems to be a bug here: after that step, the current time is not incremented in the web interface,
    // this is because the next 'application/json:' is not received, probably because it gets overwritten by the
    // answer to the robot message...
  },

  receive: function(message, robot) { // to be overriden
    console.log("Robot window '" + this.name + "' received message from Robot '" + robot + "': " + message);
  }
};
