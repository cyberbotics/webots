import FloatingWindow from './FloatingWindow.js';

export default class FloatingIDE extends FloatingWindow {
  constructor(parentNode, name, url) {
    super(parentNode, name, url);

    this.headerText.innerHTML = 'Source Code Editor';

    this.frame.id = this.name + '-robot-window';
    const elements = url.split('/').filter(element => element);
    const portString = elements[elements.length - 1];
    const port = parseFloat(portString) + 500;
    this.frame.src = url.replace(portString, port) + '/#/home/project/webots-project/';
  }
}
