import FloatingWindow from './FloatingWindow.js';

export default class FloatingIDE extends FloatingWindow {
  constructor(parentNode, name, url, windowName) {
    super(parentNode, name, url);
    this.window = windowName;

    this.headerText.innerHTML = 'IDE';

    this.frame.id = this.name + '-robot-window';
    const elements = url.split('/').filter(element => element);
    let portString = elements[elements.length - 1];
    let port = parseFloat(portString) + 500;
    this.frame.src = url.replace(portString, port) + '/#/home/project/webots-project/';
  }
}
