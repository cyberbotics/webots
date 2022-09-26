import FloatingWindow from './FloatingWindow.js';

export default class FloatingCustomWindow extends FloatingWindow {
  constructor(parentNode) {
    super(parentNode, 'custom-window');
    this.floatingWindow.style.zIndex = '2';
    this.headerText.innerHTML = 'User defined window';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);
  }

  setTooltip(tooltip) {
    const element = document.getElementById('custom-window-tooltip');
    if (element)
      element.textContent = tooltip;
  }

  setTitle(title) {
    this.headerText.textContent = title;
  }

  setContent(content) {
    this.floatingWindowContent.innerHTML = content;
  }
}
