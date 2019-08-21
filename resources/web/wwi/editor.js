/* global ace, webots, DialogWindow, DefaultUrl, SystemInfo */
'use strict';

class Editor extends DialogWindow { // eslint-disable-line no-unused-vars
  constructor(parent, mobile, view) {
    super(parent, mobile);

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

    this.tabs = document.createElement('div');
    this.tabs.id = 'webotsEditorTabs';
    this.tabs.className = 'webotsTabs';
    this.tabsHeader = document.createElement('ul');
    this.tabs.appendChild(this.tabsHeader);
    this.tabs.appendChild(edit);
    $(this.tabs).tabs({
      activate: (event, ui) => {
        this.currentSession = parseInt(ui.newTab.attr('id').substr(5)); // skip 'file-'
        this.editor.setSession(this.sessions[this.currentSession]);
      }
    });
    this.panel.appendChild(this.tabs);

    this.menu = document.createElement('div');
    this.menu.id = 'webotsEditorMenu';
    var saveShortcut;
    if (SystemInfo.isMacOS())
      saveShortcut = 'Cmd-S';
    else
      saveShortcut = 'Ctrl-S';
    this.menu.innerHTML = '<input type="image" id="webotsEditorMenuImage" width="17px" src="' + DefaultUrl.wwiImagesUrl() + 'menu.png">' +
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
    this.params.resize = () => { this._resize(); };
    this.params.open = () => { DialogWindow.resizeDialogOnOpen(this.panel); };
    this.params.title = 'Editor';

    $(this.panel).dialog(this.params).dialogExtend({maximizable: !mobile});

    this.editor.commands.addCommand({
      name: 'save',
      bindKey: {win: 'Ctrl-S', mac: 'Cmd-S'},
      exec: (editor) => { this.save(this.currentSession); }
    });
    $('#webotsEditorSaveAction').click(() => {
      this.save(this.currentSession);
      this._hideMenu();
    });
    $('#webotsEditorSaveAllAction').click(() => {
      for (let i in this.filenames)
        this.save(i);
      this._hideMenu();
    });
    $('#webotsEditorResetAction').click(() => {
      this._openResetConfirmDialog(false);
    });
    $('#webotsEditorResetAllAction').click(() => {
      this._openResetConfirmDialog(true);
    });
    $('#webotsEditorMenuImage').click(() => {
      if ($('#webotsEditorMenu').hasClass('pressed'))
        $('#webotsEditorMenu').removeClass('pressed');
      else
        $('#webotsEditorMenu').addClass('pressed');
    });
    $('#webotsEditorMenu').focusout(() => {
      // Let the time to handle the menu actions if needed.
      window.setTimeout(() => {
        if ($('.webotsEditorMenuContentItem:hover').length > 0)
          return;
        if ($('#webotsEditorMenu').hasClass('pressed'))
          $('#webotsEditorMenu').removeClass('pressed');
      }, 100);
    });
  }

  hasUnsavedChanges() {
    if (this.unloggedFileModified)
      return true;
    for (let i in this.filenames) {
      if ($('#filename-' + i).html().endsWith('*'))
        return true;
    }
    return false;
  }

  // Upload file to the simulation server.
  upload(i) {
    this.view.stream.socket.send('set controller:' +
      this.dirname + '/' +
      this.filenames[i] + ':' +
      this.sessions[i].getLength() + '\n' +
      this.sessions[i].getValue());
    this.needToUploadFiles[i] = false;
  }

  // Save file to the web site.
  save(i) {
    if ($('#filename-' + i).html().endsWith('*')) { // file was modified
      $('#filename-' + i).html(this.filenames[i]);
      this.needToUploadFiles[i] = true;
      if ((typeof webots.User1Id !== 'undefined' || webots.User1Id !== '') && webots.User1Authentication) // user logged in
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
        setTimeout(() => { $('#webotsEditorStatusMessage').remove(); }, 1500);
      }
    }
  }

  addFile(filename, content) {
    var index = this.filenames.indexOf(filename);
    if (index >= 0) {
      this.needToUploadFiles[index] = false; // just received from the simulation server
      this.sessions[index].setValue(content);
      if ($('#filename-' + index).html().endsWith('*'))
        $('#filename-' + index).html(filename);
      if (webots.User1Authentication && (typeof webots.User1Id !== 'undefined' || webots.User1Id !== ''))
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
    this.sessions[index].on('change', (e) => { this._textChange(index); });
    $('div#webotsEditorTabs ul').append('<li id="file-' + index + '"><a href="#webotsEditorTab" id="filename-' + index + '">' + filename + '</a></li>');
    $('div#webotsEditorTabs').tabs('refresh');
    if (index === 0)
      $('div#webotsEditorTabs').tabs('option', 'active', index);
  }

  closeAllTabs() {
    this.editor.setSession(ace.createEditSession('', ''));
    this.filenames = [];
    this.needToUploadFiles = [];
    this.sessions = [];
    this.sessions[0] = this.editor.getSession();
    this.currentSession = 0;
    $('div#webotsEditorTabs ul').empty();
    $('#webotsEditorMenu').hide();
    $('#webotsEditorTabs').hide();
  }

  // private functions
  _resize() {
    var padding = $('#webotsEditorTab').outerHeight() - $('#webotsEditorTab').height();
    $('#webotsEditorTab').height(this.tabs.clientHeight - this.tabsHeader.scrollHeight - padding);
    this.editor.resize();
  }

  _hideMenu() {
    if ($('#webotsEditorMenu').hasClass('pressed'))
      $('#webotsEditorMenu').removeClass('pressed');
  }

  _openResetConfirmDialog(allFiles) {
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
      open: () => { DialogWindow.openDialog(confirmDialog); },
      appendTo: this.parent,
      buttons: {
        'Cancel': () => {
          $(confirmDialog).dialog('close');
          $('#webotsEditorConfirmDialog').remove();
        },
        'Reset': () => {
          $(confirmDialog).dialog('close');
          $('#webotsEditorConfirmDialog').remove();
          if (this.resetAllFiles) {
            this.filenames.forEach((filename) => {
              this.view.server.resetController(this.dirname + '/' + filename);
            });
          } else
            this.view.server.resetController(this.dirname + '/' + this.filenames[this.currentSession]);
        }
      }
    });
    this._hideMenu();
  }

  _textChange(index) {
    if (!$('#filename-' + index).html().endsWith('*') && this.editor.curOp && this.editor.curOp.command.name) { // user change
      $('#filename-' + index).html(this.filenames[index] + '*');
    }
  }

  _aceMode(filename) {
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
  }

  _storeUserFile(i) {
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
      success: (data) => {
        if (data !== 'OK')
          this.alert('File saving error', data);
      }
    });
  }
}
