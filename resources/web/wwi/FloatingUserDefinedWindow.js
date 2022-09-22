import FloatingWindow from './FloatingWindow.js';

export default class FloatingUserDefinedWindow extends FloatingWindow {
  constructor(parentNode) {
    super(parentNode, 'floating-user-defined');
    this.floatingWindow.style.zIndex = '2';
    this.headerText.innerHTML = 'User defined window';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);
  }

  setTitle(title) {
    this.headerText.textContent = title;
  }
