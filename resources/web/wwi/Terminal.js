import FloatingWindow from './FloatingWindow.js';

export default class Terminal extends FloatingWindow {
  constructor(parentNode) {
    super(parentNode, 'terminal');
    this.floatingWindow.style.zIndex = '2';
    this.headerText.innerHTML = 'Terminal';
    this.floatingWindowContent.removeChild(this.frame);
    this.options = document.createElement('div');
    this.options.className = 'terminal-options';
    this.floatingWindowContent.appendChild(this.options);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-terminal';
    this.floatingWindowContent.appendChild(this.frame);
    this.ansiUp = new AnsiUp();
    this.textIDs = [];
    this.currentID = 0;

    parentNode.setWebotsMessageCallback(_ => this.createMessage(_));
    parentNode.setWebotsErrorMessageCallback(_ => this.createErrorMessage(_));

    this._createOptions();
  }

  createMessage(message) {
    let html = this.ansiUp.ansi_to_html(message);
    let newElement = '<p id=t' + this.currentID + ' style="color:white" class=terminal-message>' + html + '</p>';
    this._addMessage(newElement);
  }

  createErrorMessage(message) {
    let html = this.ansiUp.ansi_to_html(message);
    let newElement = '<p id=t' + this.currentID + ' style="color:red" class=terminal-message>' + html + '</p>';
    this._addMessage(newElement);
  }

  clear() {
    this.textIDs = [];
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-terminal';
    this.floatingWindowContent.appendChild(this.frame);
  }

  _addMessage(message) {
    this.textIDs.push('t' + this.currentID);
    this.currentID++;

    let terminal = this.frame;
    if (terminal) {
      terminal.innerHTML += message;
      if (this.textIDs.length > 1000) {
        let id = this.textIDs.shift();
        let nodeToRemove = document.getElementById(id);
        if (nodeToRemove)
          nodeToRemove.parentNode.removeChild(nodeToRemove);
      }
      terminal.scrollTop = terminal.scrollHeight;
    }
  }

  _createOptions() {
    const button = document.createElement('button');
    button.onclick = () => this.clear();
    button.className = 'icon-clear';
    this.options.appendChild(button);
  }
}
