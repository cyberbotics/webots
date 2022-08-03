import FloatingWindow from './FloatingWindow.js';

export default class FloatingRobotWindow extends FloatingWindow {
  constructor(parentNode, name, url, windowName) {
    super(parentNode, name, url);
    this.window = windowName;
    this.headerText.innerHTML = 'Robot Window: ' + name;
    this.frame.id = this.name + '-robot-window';
    this.frame.src = (this.window === '<none>') ? '' : this.url + '/robot_windows/' + this.window + '/' + this.window +
      '.html?name=' + this.name;
  }
}
