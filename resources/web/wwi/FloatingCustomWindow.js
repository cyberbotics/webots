import FloatingWindow from './FloatingWindow.js';

export default class FloatingCustomWindow extends FloatingWindow {
  constructor(parentNode) {
    super(parentNode, 'custom-window');
    this.floatingWindow.style.zIndex = '3';
    this.headerText.innerHTML = 'User defined window';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);
  }

  setTooltip(tooltip) {
    const element = document.getElementById('custom-window-button');
    if (element)
      element.title = tooltip;
  }

  setTitle(title) {
    this.headerText.textContent = title;
  }

  setContent(content) {
    this.floatingWindowContent.innerHTML = content;
  }
}
