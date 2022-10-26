import FloatingWindow from './FloatingWindow.js';

export default class FloatingProtoParameterWindow extends FloatingWindow {
  constructor(parentNode) {
    super(parentNode, 'proto-parameter');
    this.floatingWindow.style.zIndex = '2';
    this.headerText.innerHTML = 'Proto parameter window';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);
  }
}
