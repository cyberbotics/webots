/* global ace, webots, DialogWindow, DefaultUrl */
'use strict';

function Editor(parent, mobile, view) {
  DialogWindow.call(this, parent, mobile);

  this.panel.id = 'webotsEditor';
  this.panel.className = 'webotsTabContainer';

  this.view = view;
  this.filenames = [];
  this.needToUploadFiles = [];
  this.sessions = [];

  var edit = document.createElement('div');
  edit.id = 'webotsEditorTab';
  edit.className = 'webotsTab';
  this.editor = ace.edit(edit);
  this.sessions[0] = this.editor.getSession();
  this.currentSession = 0;

  var that = this;
  this.tabs = document.createElement('div');
  this.tabs.id = 'webotsEditorTabs';
  this.tabs.className = 'webotsTabs';
  this.tabsHeader = document.createElement('ul');
  this.tabs.appendChild(this.tabsHeader);
  this.tabs.appendChild(edit);
  $(this.tabs).tabs({
    activate: function(event, ui) {
      that.currentSession = parseInt(ui.newTab.attr('id').substr(5)); // skip 'file-'
      that.editor.setSession(that.sessions[that.currentSession]);
    }
  });
  this.panel.appendChild(this.tabs);

  this.menu = document.createElement('div');
  this.menu.id = 'webotsEditorMenu';
  var saveShortcut;
  if (navigator.appVersion.indexOf('Mac') === -1)
    saveShortcut = 'Ctrl-S';
  else // macOS
    saveShortcut = 'Cmd-S';
  this.menu.innerHTML = '<input type="image" id="webotsEditorMenuImage" width="17px" src="' + DefaultUrl.getImageUrl('menu') + '">' +
                        '<div id="webotsEditorMenuContent">' +
                        '<div id="webotsEditorSaveAction" class="webotsEditorMenuContentItem" title="Save current file">Save<span style="float:right"><i><small>' + saveShortcut + '</small></i></span></div>' +
                        '<div id="webotsEditorSaveAllAction" class="webotsEditorMenuContentItem" title="Save all the files">Save All</div>' +
                        '<div id="webotsEditorResetAction" class="webotsEditorMenuContentItem" title="Reset current file to the original version">Reset</div>' +
                        '<div id="webotsEditorResetAllAction" class="webotsEditorMenuContentItem" title="Reset all the files to the original version">Reset All</div>' +
                        '</div>';
  this.panel.appendChild(this.menu);

  var clampedSize = DialogWindow.clampDialogSize({left: 0, top: 0, width: 800, height: 600});
  this.params.width = clampedSize.width;
  this.params.height = clampedSize.height;
  this.params.close = null;
  this.params.resize = function() { that._resize(); };
  this.params.open = function() { DialogWindow.resizeDialogOnOpen(that.panel); };
  this.params.title = 'Editor';

  $(this.panel).dialog(this.params).dialogExtend({maximizable: !mobile});

  this.editor.commands.addCommand({
    name: 'save',
    bindKey: {win: 'Ctrl-S', mac: 'Cmd-S'},
    exec: function(editor) { that.save(that.currentSession); }
  });
  $('#webotsEditorSaveAction').click(function() {
    that.save(that.currentSession);
    that._hideMenu();
  });
  $('#webotsEditorSaveAllAction').click(function() {
    for (var i = 0; i < that.filenames.length; i++)
      that.save(i);
    that._hideMenu();
  });
  $('#webotsEditorResetAction').click(function() {
    that._openResetConfirmDialog(false);
  });
  $('#webotsEditorResetAllAction').click(function() {
    that._openResetConfirmDialog(true);
  });
  $('#webotsEditorMenuImage').click(function() {
    if ($('#webotsEditorMenu').hasClass('pressed'))
      $('#webotsEditorMenu').removeClass('pressed');
    else
      $('#webotsEditorMenu').addClass('pressed');
  });
  $('#webotsEditorMenu').focusout(function() {
    // Let the time to handle the menu actions if needed.
    window.setTimeout(function() {
      if ($('.webotsEditorMenuContentItem:hover').length > 0)
        return;
      if ($('#webotsEditorMenu').hasClass('pressed'))
        $('#webotsEditorMenu').removeClass('pressed');
    }, 100);
  });
}

Editor.prototype = {
  constructor: Editor,

  hasUnsavedChanges: function() {
    if (this.unloggedFileModified)
      return true;
    for (var i = 0; i < this.filenames.length; i++) {
      if ($('#filename-' + i).html().endsWith('*'))
        return true;
    }
    return false;
  },

  // Upload file to the simulation server.
  upload: function(i) {
    this.view.stream.socket.send('set controller:' +
      this.dirname + '/' +
      this.filenames[i] + ':' +
      this.sessions[i].getLength() + '\n' +
      this.sessions[i].getValue());
    this.needToUploadFiles[i] = false;
  },

  // Save file to the web site.
  save: function(i) {
    if ($('#filename-' + i).html().endsWith('*')) { // file was modified
      $('#filename-' + i).html(this.filenames[i]);
      this.needToUploadFiles[i] = true;
      if (webots.User1Id !== '' && webots.User1Authentication !== '') // user logged in
        this._storeUserFile(i);
      else
        this.unloggedFileModified = true;

      if (this.view.time === 0)
        this.upload(i);
      else {
        if (typeof this.statusMessage === 'undefined') {
          this.statusMessage = document.createElement('div');
          this.statusMessage.id = 'webotsEditorStatusMessage';
          this.statusMessage.className = 'webotsEditorStatusMessage';
          this.statusMessage.innerHTML = '<font size="2">Reset the simulation to apply the changes.</font>';
        }
        this.panel.appendChild(this.statusMessage);
        setTimeout(function() { $('#webotsEditorStatusMessage').remove(); }, 1500);
      }
    }
  },

  addFile: function(filename, content) {
    var index = this.filenames.indexOf(filename);
    if (index >= 0) {
      this.needToUploadFiles[index] = false; // just received from the simulation server
      this.sessions[index].setValue(content);
      if ($('#filename-' + index).html().endsWith('*'))
        $('#filename-' + index).html(filename);
      if (webots.User1Authentication !== '' && webots.User1Id !== '')
        this.storeUserFile(index);
      return;
    }

    index = this.filenames.length;
    this.filenames.push(filename);
    this.needToUploadFiles[index] = false;
    if (index === 0) {
      this.sessions[index].setMode(this._aceMode(filename));
      this.sessions[index].setValue(content);
      $('#webotsEditorMenu').show();
      $('#webotsEditorTabs').show();
    } else
      this.sessions.push(ace.createEditSession(content, this._aceMode(filename)));
    var that = this;
    this.sessions[index].on('change', function(e) { that._textChange(index); });
    $('div#webotsEditorTabs ul').append('<li id="file-' + index + '"><a href="#webotsEditorTab" id="filename-' + index + '">' + filename + '</a></li>');
    $('div#webotsEditorTabs').tabs('refresh');
    if (index === 0)
      $('div#webotsEditorTabs').tabs('option', 'active', index);
  },

  closeAllTabs: function() {
    this.editor.setSession(ace.createEditSession('', ''));
    this.filenames = [];
    this.needToUploadFiles = [];
    this.sessions = [];
    this.sessions[0] = this.editor.getSession();
    this.currentSession = 0;
    $('div#webotsEditorTabs ul').empty();
    $('#webotsEditorMenu').hide();
    $('#webotsEditorTabs').hide();
  },

  // private functions
  _resize: function() {
    var padding = $('#webotsEditorTab').outerHeight() - $('#webotsEditorTab').height();
    $('#webotsEditorTab').height(this.tabs.clientHeight - this.tabsHeader.scrollHeight - padding);
    this.editor.resize();
  },

  _hideMenu: function() {
    if ($('#webotsEditorMenu').hasClass('pressed'))
      $('#webotsEditorMenu').removeClass('pressed');
  },

  _openResetConfirmDialog: function(allFiles) {
    var that = this;
    this.resetAllFiles = allFiles;
    var titleText, message;
    message = 'Permanently reset ';
    if (allFiles) {
      message += 'all the files';
      titleText = 'Reset files?';
    } else {
      message += 'this file';
      titleText = 'Reset file?';
    }
    message += ' to the original version?';
    message += '<br/><br/>Your modifications will be lost.';
    var confirmDialog = document.createElement('div');
    this.panel.appendChild(confirmDialog);
    $(confirmDialog).html(message);
    $(confirmDialog).dialog({
      title: titleText,
      modal: true,
      autoOpen: true,
      resizable: false,
      dialogClass: 'alert',
      open: function() { DialogWindow.openDialog(this); },
      appendTo: this.parent,
      buttons: {
        'Cancel': function() {
          $(this).dialog('close');
          $('#webotsEditorConfirmDialog').remove();
        },
        'Reset': function() {
          $(this).dialog('close');
          $('#webotsEditorConfirmDialog').remove();
          if (that.resetAllFiles) {
            for (var i = 0; i < this.filenames.length; i++)
              that.view.server.resetController(that.dirname + '/' + that.filenames[i]);
          } else
            that.view.server.resetController(that.dirname + '/' + that.filenames[that.currentSession]);
        }
      }
    });
    this._hideMenu();
  },

  _textChange: function(index) {
    if (!$('#filename-' + index).html().endsWith('*') && this.editor.curOp && this.editor.curOp.command.name) { // user change
      $('#filename-' + index).html(this.filenames[index] + '*');
    }
  },

  _aceMode: function(filename) {
    if (filename.toLowerCase() === 'makefile')
      return 'ace/mode/makefile';
    var extension = filename.split('.').pop().toLowerCase();
    if (extension === 'py')
      return 'ace/mode/python';
    if (extension === 'c' || extension === 'cpp' || extension === 'c++' || extension === 'cxx' || extension === 'cc' ||
        extension === 'h' || extension === 'hpp' || extension === 'h++' || extension === 'hxx' || extension === 'hh')
      return 'ace/mode/c_cpp';
    if (extension === 'java')
      return 'ace/mode/java';
    if (extension === 'm')
      return 'ace/mode/matlab';
    if (extension === 'json')
      return 'ace/mode/json';
    if (extension === 'xml')
      return 'ace/mode/xml';
    if (extension === 'yaml')
      return 'ace/mode/yaml';
    if (extension === 'ini')
      return 'ace/mode/ini';
    if (extension === 'html')
      return 'ace/mode/html';
    if (extension === 'js')
      return 'ace/mode/javascript';
    if (extension === 'css')
      return 'ace/mode/css';
    return 'ace/mode/text';
  },

  _storeUserFile: function(i) {
    var formData = new FormData();
    formData.append('dirname', this.view.server.project + '/controllers/' + this.dirname);
    formData.append('filename', this.filenames[i]);
    formData.append('content', this.sessions[i].getValue());
    $.ajax({
      url: '/ajax/upload-file.php',
      type: 'POST',
      data: formData,
      processData: false,
      contentType: false,
      success: function(data) {
        if (data !== 'OK')
          DialogWindow.alert('File saving error', data);
      }
    });
  }
};
