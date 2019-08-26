/* global webots, DialogWindow */
'use strict';

class RobotWindow extends DialogWindow { // eslint-disable-line no-unused-vars
  constructor(parent, mobile, name) {
    super(parent, mobile);
    this.name = name;
    this.panel.id = name;
    this.panel.className = 'webotsTabContainer';

    var clampedSize = DialogWindow.clampDialogSize({left: 5, top: 5, width: 400, height: 400});
    this.params.width = clampedSize.width;
    this.params.height = clampedSize.height;
    this.params.close = null;
    this.params.position = {at: 'left+5 top+5', my: 'left top', of: this.parent};
    this.params.title = 'Robot Window';

    $(this.panel).dialog(this.params).dialogExtend({maximizable: !mobile});
  }

  setProperties(properties) {
    $(this.panel).dialog(properties);
  }

  geometry() {
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
  }

  restoreGeometry(data) {
    $(this.panel).dialog({
      width: data.width,
      height: data.height,
      position: data.position
    });
    var webotsTabs = this.panel.getElementsByClassName('webotsTabs');
    if (data.activeTabIndex >= 0 && webotsTabs.length > 0)
      $(webotsTabs[0]).tabs('option', 'active', data.activeTabIndex);
  }

  destroy() {
    this.close();
    this.panel.parentNode.removeChild(this.panel);
    this.panel = null;
  }

  setContent(content) {
    $(this.panel).html(content);
  }

  open() {
    $(this.panel).dialog('open');
  }

  isOpen() {
    return $(this.panel).dialog('isOpen');
  }

  close() {
    $(this.panel).dialog('close');
  }

  send(message, robot) {
    webots.currentView.sendRobotMessage(message, robot);
  }

  receive(message, robot) { // to be overriden
    console.log("Robot window '" + this.name + "' received message from Robot '" + robot + "': " + message);
  }
}
