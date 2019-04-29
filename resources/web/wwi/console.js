/* global DialogWindow */
'use strict';

function Console(parent, mobile) {
  DialogWindow.call(this, parent, mobile);

  this.panel.id = 'webotsConsole';
  this.panel.className = 'webotsConsole';

  var clampedSize = DialogWindow.clampDialogSize({left: 0, top: 0, width: 600, height: 400});
  this.params.width = clampedSize.width;
  this.params.height = clampedSize.height;
  this.params.close = function() { $('#consoleButton').removeClass('toolBarButtonActive'); };
  this.params.title = 'Console';

  $(this.panel).dialog(this.params).dialogExtend({maximizable: !mobile});
}

Console.prototype = {
  constructor: Console,

  scrollDown: function() {
    if (this.panel)
      this.panel.scrollTop = this.panel.scrollHeight;
  },

  clear: function() {
    if (this.panel) {
      while (this.panel.firstChild)
        this.panel.removeChild(this.panel.firstChild);
    } else
      console.clear();
  },

  stdout: function(message) {
    this._log(message, 0);
  },

  stderr: function(message) {
    this._log(message, 1);
  },

  info: function(message) {
    this._log(message, 2);
  },

  error: function(message) {
    this._log(message, 3);
  },

  // private functions
  _log: function(message, type) {
    var para = document.createElement('p');
    var style = 'margin:0;';
    var title = '';
    switch (type) {
      case 0:
        style += 'color:Blue;';
        title = 'Webots stdout';
        break;
      case 1:
        style += 'color:Red;';
        title = 'Webots stderr';
        break;
      case 2:
        style += 'color:Gray;';
        title = 'info';
        break;
      case 3:
        style += 'color:Salmon;';
        title = 'error';
        break;
    }
    if (this.panel) {
      para.style.cssText = style;
      para.title = title + ' (' + this._hourString() + ')';
      var t = document.createTextNode(message);
      para.appendChild(t);
      this.panel.appendChild(para);
      this.scrollDown();
    } else
      console.log('%c' + message, style);
  },

  _hourString: function() {
    var d = new Date();
    return d.getHours() + ':' +
         ((d.getMinutes() < 10) ? '0' : '') + d.getMinutes() + ':' +
         ((d.getSeconds() < 10) ? '0' : '') + d.getSeconds();
  }
};
