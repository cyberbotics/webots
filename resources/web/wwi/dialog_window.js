/* global webots */
'use strict';

class DialogWindow { // eslint-disable-line no-unused-vars
  constructor(parent, mobile) {
    this.mobile = mobile;
    this.parent = parent;
    this.panel = document.createElement('div');
    parent.appendChild(this.panel);

    this.params = {
      appendTo: parent,
      open: () => { this.openDialog(this.panel); },
      autoOpen: false,
      resizeStart: this.disablePointerEvents,
      resizeStop: this.enablePointerEvents,
      dragStart: this.disablePointerEvents,
      dragStop: this.enablePointerEvents
    };
    if (this.mobile)
      this.addMobileDialogAttributes(this.params, this.panel);
  }

  clampDialogSize(preferredGeometry) {
    if (typeof $('#playerDiv').height === 'undefined' || typeof $('#playerDiv').width === 'undefined')
      return preferredGeometry;

    var maxHeight = $('#playerDiv').height() - preferredGeometry.top - $('#toolBar').height() - 20; // 20 is chosen arbitrarily
    var maxWidth = $('#playerDiv').width() - preferredGeometry.left - 20; // 20 is chosen arbitrarily
    var height = preferredGeometry.height;
    var width = preferredGeometry.width;
    if (maxHeight < height)
      height = maxHeight;
    if (maxWidth < width)
      width = maxWidth;
    return {width: width, height: height};
  }

  openDialog(dialog) {
    this.resizeDialogOnOpen(dialog);
    $(dialog).parent().css('opacity', 0.9);
    $(dialog).parent().hover(() => {
      $(dialog).css('opacity', 0.99);
    }, (event) => {
      $(dialog).css('opacity', 0.9);
    });
  }

  resizeDialogOnOpen(dialog) {
    var w = $(dialog).parent().width();
    var h = $(dialog).parent().height();
    var clampedSize = this.clampDialogSize({left: 0, top: 0, width: w, height: h});
    if (clampedSize.width < w)
      $(dialog).dialog('option', 'width', clampedSize.width);
    if (clampedSize.height < h)
      $(dialog).dialog('option', 'height', clampedSize.height);
  }

  mobileCreateDialog() {
    // mobile only setup
    var closeButton = $('button:contains("WbClose")');
    closeButton.html('');
    closeButton.removeClass('ui-button-text-only');
    closeButton.addClass('mobile-dialog-close-button');
    closeButton.addClass('ui-button-icon-primary');
    closeButton.prepend('<span class="ui-icon ui-icon-closethick"></span>');
  }

  addMobileDialogAttributes(params, panel) {
    params.dialogClass = 'mobile-no-default-buttons';
    params.create = () => { this.mobileCreateDialog(); };
    params.buttons = { 'WbClose': () => { $(panel).dialog('close'); } };
  }

  // The following two functions are used to make the resize and drag of the dialog
  // steady (i.e., not loose the grab while resizing/dragging the dialog quickly).
  disablePointerEvents() {
    document.body.style['pointer-events'] = 'none';
  }

  enablePointerEvents() {
    document.body.style['pointer-events'] = 'auto';
  }
}

webots.alert = (title, message, callback) => {
  webots.currentView.ondialogwindow(true);
  var parent = webots.currentView.view3D;
  var panel = document.getElementById('webotsAlert');
  if (!panel) {
    panel = document.createElement('div');
    panel.id = 'webotsAlert';
    parent.appendChild(panel);
  }
  panel.innerHTML = message;
  $('#webotsAlert').dialog({
    title: title,
    resizeStart: this.disablePointerEvents,
    resizeStop: this.enablePointerEvents,
    dragStart: this.disablePointerEvents,
    dragStop: this.enablePointerEvents,
    appendTo: parent,
    open: this.openDialog,
    modal: true,
    width: 400, // enough room to display the social network buttons in a line
    buttons: {
      Ok: () => { $('#webotsAlert').dialog('close'); }
    },
    close: () => {
      if (typeof callback === 'function')
        callback();
      webots.currentView.ondialogwindow(false);
      $('#webotsAlert').remove();
    }
  });
};

webots.confirm = (title, message, callback) => {
  webots.currentView.ondialogwindow(true);
  var parent = webots.currentView.view3D;
  var panel = document.createElement('div');
  panel.id = 'webotsConfirm';
  panel.innerHTML = message;
  parent.appendChild(panel);
  $('#webotsConfirm').dialog({
    title: title,
    resizeStart: this.disablePointerEvents,
    resizeStop: this.enablePointerEvents,
    dragStart: this.disablePointerEvents,
    dragStop: this.enablePointerEvents,
    appendTo: parent,
    open: this.openDialog,
    modal: true,
    width: 400, // enough room to display the social network buttons in a line
    buttons: {
      Ok: () => { $('#webotsConfirm').dialog('close'); callback(); },
      Cancel: () => { $('#webotsConfirm').dialog('close'); }
    },
    close: () => { $('#webotsConfirm').dialog('destroy').remove(); webots.currentView.ondialogwindow(false); }});
};
