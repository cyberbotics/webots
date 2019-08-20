/* global DialogWindow */
'use strict';

class Console extends DialogWindow { // eslint-disable-line no-unused-vars
  constructor(parent, mobile) {
    super(parent, mobile);

    this.panel.id = 'webotsConsole';
    this.panel.className = 'webotsConsole';

    var clampedSize = super.clampDialogSize({left: 0, top: 0, width: 600, height: 400});
    this.params.width = clampedSize.width;
    this.params.height = clampedSize.height;
    this.params.close = () => { $('#consoleButton').removeClass('toolBarButtonActive'); };
    this.params.title = 'Console';

    $(this.panel).dialog(this.params).dialogExtend({maximizable: !mobile});

    var buttonDiv = document.createElement('div');
    this.logDiv = document.createElement('div');
    var clearButton = document.createElement('button');
    var clearButtonText = document.createTextNode('Clear');
    clearButton.appendChild(clearButtonText);
    // clearButton.className = 'innerButton';
    clearButton.addEventListener('click', () => {
      this.clear();
    });
    buttonDiv.appendChild(clearButton);
    this.panel.appendChild(this.logDiv);
    this.panel.appendChild(buttonDiv);
  }

  scrollDown() {
    if (this.panel)
      this.panel.scrollTop = this.panel.scrollHeight;
  }

  clear() {
    if (this.logDiv) {
      while (this.logDiv.firstChild)
        this.logDiv.removeChild(this.logDiv.firstChild);
    } else
      console.clear();
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
    if (this.logDiv) {
      para.style.cssText = style;
      para.title = title + ' (' + this._hourString() + ')';
      var t = document.createTextNode(message);
      para.appendChild(t);
      this.logDiv.appendChild(para);
      this.scrollDown();
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
