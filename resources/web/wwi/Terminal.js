import FloatingWindow from './FloatingWindow.js';

export default class FloatingIDE extends FloatingWindow {
  constructor(parentNode) {
    super(parentNode, 'terminal');
    this.floatingWindow.style.zIndex = '2';
    this.headerText.innerHTML = 'Terminal';
    console.log(this.frame);
    this.floatingWindowContent.removeChild(this.frame);
    const container = document.createElement('div');
    this.floatingWindowContent.appendChild(container);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-terminal';
    container.appendChild(this.frame);
    this.ansiUp = new AnsiUp();
    this.textIDs = [];

    parentNode.setWebotsMessageCallback(_ => this.createMessage(_));
    parentNode.setWebotsErrorMessageCallback(_ => this.createErrorMessage(_));
  }

  createMessage(msg) {
    let html = this.ansiUp.ansi_to_html(msg);
    let newElement = '<p id=t' + this.currentID + ' >' + html + '</p>';
    this.textIDs.push('t' + this.currentID);
    this.currentID++;

    let terminal = this.frame;
    if (terminal) {
      terminal.innerHTML += newElement;
      if (this.textIDs.length > 1000) {
        let id = this.textIDs.shift();
        let nodeToRemove = document.getElementById(id);
        if (nodeToRemove)
          nodeToRemove.parentNode.removeChild(nodeToRemove);
      }
      terminal.parentNode.scrollTop = terminal.parentNode.scrollHeight;
    }
  }

  createErrorMessage(msg) {
    let newElement = '<p id=t' + this.currentID + ' style="color:red">' + msg + '</p>';
    this.textIDs.push('t' + this.currentID);
    this.currentID++;

    let terminal = document.getElementById('terminal');
    if (terminal) {
      terminal.innerHTML += newElement;
      if (this.textIDs.length > 1000) {
        let id = this.textIDs.shift();
        let nodeToRemove = document.getElementById(id);
        if (nodeToRemove)
          nodeToRemove.parentNode.removeChild(nodeToRemove);
      }
      terminal.parentNode.scrollTop = terminal.parentNode.scrollHeight;
    }
  }
}
