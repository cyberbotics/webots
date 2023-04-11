import FloatingWindow from './FloatingWindow.js';

export default class FloatingRobotWindow extends FloatingWindow {
  constructor(parentNode, name, url, windowName, mainWindow) {
    super(parentNode, name, url);
    this.isMainWindow = mainWindow;
    this.window = windowName;
    this.headerText.innerHTML = (mainWindow ? 'World Info: ' : 'Robot Window: ') + name;
    this.floatingWindow.className += ' floating-robot-window';
    this.frame.id = this.name + '-robot-window';
    this.frame.src = (this.window === '<none>') ? '' : this.url + '/robot_windows/' + this.window + '/' + this.window +
      '.html?name=' + this.name;

    if (this.isMainWindow) {
      this.frame.allowtransparency = 'true';
      this.frame.style.backgroundColor = 'transparent';
    }
  }
}
