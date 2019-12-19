/* global DialogWindow, DefaultUrl */
'use strict';

class Console extends DialogWindow { // eslint-disable-line no-unused-vars
  constructor(parent, mobile) {
    super(parent, mobile);

    this.panel.id = 'webotsConsole';
    this.panel.className = 'webotsConsole';

    var clampedSize = DialogWindow.clampDialogSize({left: 0, top: 0, width: 600, height: 400});
    this.params.width = clampedSize.width;
    this.params.height = clampedSize.height;
    this.params.close = () => { $('#consoleButton').removeClass('toolBarButtonActive'); };
    this.params.title = 'Console';

    $(this.panel).dialog(this.params).dialogExtend({maximizable: !mobile});

    var buttons = document.createElement('div');
    buttons.className = 'webotsConsoleButtons';
    this.logs = document.createElement('div');
    this.logs.className = 'webotsConsoleLogs';
    var clearButtonIcon = document.createElement('img');
    clearButtonIcon.className = 'webotsConsoleButtonIcon';
    clearButtonIcon.setAttribute('src', DefaultUrl.wwiImagesUrl() + 'trash.png');
    this.clearButton = document.createElement('button');
    this.clearButton.className = 'webotsConsoleButton';
    this.clearButton.disabled = true;
    this.clearButton.title = 'Clear the console';
    this.clearButton.appendChild(clearButtonIcon);
    this.clearButton.addEventListener('click', () => {
      this.clear();
    });
    buttons.appendChild(this.clearButton);
    this.panel.appendChild(buttons);
    this.panel.appendChild(this.logs);
  }

  scrollDown() {
    if (this.panel)
      this.panel.scrollTop = this.panel.scrollHeight;
  }

  // Remove all the logs.
  clear() {
    if (this.logs) {
      while (this.logs.firstChild)
        this.logs.removeChild(this.logs.firstChild);
    } else
      console.clear();
    this.clearButton.disabled = true;
  }

  // Remove the oldest logs.
  purge() {
    const historySize = 100; // defines the maximum size of the log.
    if (this.logs) {
      while (this.logs.firstChild && this.logs.childElementCount > historySize)
        this.logs.removeChild(this.logs.firstChild);
    }
  }

  stdout(message) {
    this._log(message, 0);
  }

  stderr(message) {
    this._log(message, 1);
  }

  info(message) {
    this._log(message, 2);
  }

  error(message) {
    this._log(message, 3);
  }

  // private functions
  _log(message, type) {
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
    if (this.logs) {
      para.style.cssText = style;
      para.title = title + ' (' + this._hourString() + ')';
      var t = document.createTextNode(message);
      para.appendChild(t);
      this.logs.appendChild(para);
      this.purge();
      this.scrollDown();
      this.clearButton.disabled = false;
    } else
      console.log('%c' + message, style);
  }

  _hourString() {
    var d = new Date();
    return d.getHours() + ':' +
         ((d.getMinutes() < 10) ? '0' : '') + d.getMinutes() + ':' +
         ((d.getSeconds() < 10) ? '0' : '') + d.getSeconds();
  }
}
