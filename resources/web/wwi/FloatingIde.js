import FloatingWindow from './FloatingWindow.js';

export default class FloatingIDE extends FloatingWindow {
  constructor(parentNode, name, url) {
    super(parentNode, name, url);

    this.floatingWindow.style.zIndex = '3';
    this.headerText.innerHTML = 'Source Code Editor';
    this.frame.id = this.name + '-floating-window';

    let elements = url.split('/').filter(element => element);
    let portString = elements[elements.length - 1];
    if (Number.isNaN(parseInt(portString))) {
      elements = url.split(':').filter(element => element);
      portString = elements[elements.length - 1];
    }
    const port = parseInt(portString) + 500;

    this.frame.src = url.replace(portString, port) + '/#/home/project/webots-project/';
  }
}
