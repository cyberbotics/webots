/* global webots */
'use strict';

function DialogWindow(parent, mobile) {
  this.mobile = mobile;
  this.parent = parent;
  this.panel = document.createElement('div');
  parent.appendChild(this.panel);

  var that = this;
  this.params = {
    appendTo: parent,
    open: function() { DialogWindow.openDialog(that.panel); },
    autoOpen: false,
    resizeStart: DialogWindow.disablePointerEvents,
    resizeStop: DialogWindow.enablePointerEvents,
    dragStart: DialogWindow.disablePointerEvents,
    dragStop: DialogWindow.enablePointerEvents
  };
  if (this.mobile)
    DialogWindow.addMobileDialogAttributes(this.params, this.panel);
}

DialogWindow.prototype = {
  constructor: DialogWindow
};

DialogWindow.clampDialogSize = function(preferredGeometry) {
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
};

DialogWindow.openDialog = function(dialog) {
  DialogWindow.resizeDialogOnOpen(dialog);
  $(dialog).parent().css('opacity', 0.9);
  $(dialog).parent().hover(function() {
    $(dialog).css('opacity', 0.99);
  }, function(event) {
    $(dialog).css('opacity', 0.9);
  });
};

DialogWindow.resizeDialogOnOpen = function(dialog) {
  var w = $(dialog).parent().width();
  var h = $(dialog).parent().height();
  var clampedSize = this.clampDialogSize({left: 0, top: 0, width: w, height: h});
  if (clampedSize.width < w)
    $(dialog).dialog('option', 'width', clampedSize.width);
  if (clampedSize.height < h)
    $(dialog).dialog('option', 'height', clampedSize.height);
};

DialogWindow.mobileCreateDialog = function() {
  // mobile only setup
  var closeButton = $('button:contains("WbClose")');
  closeButton.html('');
  closeButton.removeClass('ui-button-text-only');
  closeButton.addClass('mobile-dialog-close-button');
  closeButton.addClass('ui-button-icon-primary');
  closeButton.prepend('<span class="ui-icon ui-icon-closethick"></span>');
};

DialogWindow.addMobileDialogAttributes = function(params, panel) {
  params.dialogClass = 'mobile-no-default-buttons';
  params.create = function() { DialogWindow.mobileCreateDialog(); };
  params.buttons = { 'WbClose': function() { $(panel).dialog('close'); } };
};

// The following two functions are used to make the resize and drag of the dialog
// steady (i.e., not loose the grab while resizing/dragging the dialog quickly).
DialogWindow.disablePointerEvents = function() {
  document.body.style['pointer-events'] = 'none';
};

DialogWindow.enablePointerEvents = function() {
  document.body.style['pointer-events'] = 'auto';
};

webots.alert = function(title, message, callback) {
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
    resizeStart: DialogWindow.disablePointerEvents,
    resizeStop: DialogWindow.enablePointerEvents,
    dragStart: DialogWindow.disablePointerEvents,
    dragStop: DialogWindow.enablePointerEvents,
    appendTo: parent,
    open: DialogWindow.openDialog,
    modal: true,
    width: 400, // enough room to display the social network buttons in a line
    buttons: {
      Ok: function() { $(this).dialog('close'); }
    },
    close: function() {
      if (typeof callback === function)
        callback();
      webots.currentView.ondialogwindow(false);
      $(this).remove();
    }
  });
};

webots.confirm = function(title, message, callback) {
  webots.currentView.ondialogwindow(true);
  var parent = webots.currentView.view3D;
  var panel = document.createElement('div');
  panel.id = 'webotsConfirm';
  panel.innerHTML = message;
  parent.appendChild(panel);
  $('#webotsConfirm').dialog({
    title: title,
    resizeStart: DialogWindow.disablePointerEvents,
    resizeStop: DialogWindow.enablePointerEvents,
    dragStart: DialogWindow.disablePointerEvents,
    dragStop: DialogWindow.enablePointerEvents,
    appendTo: parent,
    open: DialogWindow.openDialog,
    modal: true,
    width: 400, // enough room to display the social network buttons in a line
    buttons: {
      Ok: function() { $(this).dialog('close'); callback(); },
      Cancel: function() { $(this).dialog('close'); }
    },
    close: function() { $(this).dialog('destroy').remove(); webots.currentView.ondialogwindow(false); }});
};
