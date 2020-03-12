/* global SystemInfo */

'use strict';

class MultimediaClient { // eslint-disable-line no-unused-vars
  constructor(view, parentObject, contextMenu) {
    this.view = view;
    this.domElement = document.createElement('img');
    this.domElement.style.background = 'white';
    this.domElement.id = 'remoteScene';
    this.domElement.setAttribute('draggable', false);
    parentObject.appendChild(this.domElement);
    this.mouseDown = 0;
    this.onmousemove = (e) => { this._onMouseMove(e); };
    this.lastMousePosition = null;
    this.contextMenu = contextMenu;
    this.robotWindows = [];
    this.worldInfo = {title: null, infoWindow: null};
  }

  disconnect() {
    this.domElement.src = '';
  }

  finalize(onready) {
    this.domElement.addEventListener('mousedown', (e) => { this._onMouseDown(e); }, false);
    this.domElement.addEventListener('mouseup', (e) => { this._onMouseUp(e); }, false);
    this.domElement.addEventListener('wheel', (e) => { this._onWheel(e); }, false);
    this.domElement.addEventListener('contextmenu', (e) => { this._onContextMenu(e); }, false);
    if (typeof onready === 'function')
      onready();
  }

  setWorldInfo(title, infoWindowName) {
    this.worldInfo = {title: title, infoWindow: infoWindowName};
  }

  setRobotWindow(robotName, windowName) {
    this.robotWindows.push([robotName, windowName]);
  }

  setFollowed(solidId, mode) {
    var socket = this.view.stream.socket;
    if (!socket || socket.readyState !== 1)
      return;
    socket.send('follow: ' + mode + ',' + solidId);
  }

  showContextMenu(object) {
    this.contextMenu.show(object, this.lastMousePosition);
  }

  sendMouseEvent(type, event, wheel) {
    this.contextMenu.hide();
    var socket = this.view.stream.socket;
    if (!socket || socket.readyState !== 1)
      return;
    var modifier = (event.shiftKey ? 1 : 0) + (event.ctrlKey ? 2 : 0) + (event.altKey ? 4 : 0);
    socket.send('mouse ' + type + ' ' + event.button + ' ' + this.mouseDown + ' ' +
                event.offsetX + ' ' + event.offsetY + ' ' + modifier + ' ' + wheel);
    this.lastMousePosition = {x: event.offsetX, y: event.offsetY};
  }

  requestNewSize(width, height) {
    this.view.stream.socket.send('resize: ' + this.domElement.width + 'x' + this.domElement.height);
  }

  resize(width, height) {
    this.domElement.style.width = width + 'px';
    this.domElement.style.height = height + 'px';
  }

  processServerMessage(data) {
    if (data.startsWith('multimedia: ')) {
      let list = data.split(' ');
      let httpUrl = this.view.stream.wsServer.replace('ws', 'http');
      let url = httpUrl + list[1];
      this.view.toolBar.setMode(list[2]);
      this.domElement.src = url;
      this.resize(list[3], list[4]);
      console.log('Multimedia streamed on ' + url);
    } else if (data.startsWith('robot window: ')) {
      let robotInfo = data.substring(data.indexOf(':') + 1).trim();
      let separatorIndex = robotInfo.indexOf(':');
      let nameSize = parseFloat(robotInfo.substring(0, separatorIndex));
      let robotName = robotInfo.substring(separatorIndex + 1, nameSize + separatorIndex + 1);
      let windowName = robotInfo.substring(separatorIndex + nameSize + 2);
      this.setRobotWindow(robotName, windowName);
    } else if (data.startsWith('world info: ')) {
      // world info: <name size>:<info window name>:<world title>
      let info = data.substring(data.indexOf(':') + 1).trim();
      let separatorIndex = info.indexOf(':');
      let nameSize = parseFloat(info.substring(0, separatorIndex));
      let infoWindowName = info.substring(separatorIndex + 1, nameSize + separatorIndex + 1);
      let title = info.substring(separatorIndex + nameSize + 2);
      this.setWorldInfo(title, infoWindowName);
    } else if (data.startsWith('context menu: ')) {
      let info = data.substring(data.indexOf(':') + 1).trim();
      this.showContextMenu(JSON.parse(info));
    } else
      return false;
    return true;
  }

  _onMouseDown(event) {
    this.mouseDown = 0;
    switch (event.button) {
      case 0: // MOUSE.LEFT
        this.mouseDown |= 1;
        break;
      case 1: // MOUSE.MIDDLE
        this.mouseDown |= 4;
        break;
      case 2: // MOUSE.RIGHT
        this.mouseDown |= 2;
        break;
    }
    if (SystemInfo.isMacOS() && 'ctrlKey' in event && event['ctrlKey'] && this.mouseDown === 1)
      // On macOS, "Ctrl + left click" should be dealt as a right click.
      this.mouseDown = 2;

    event.target.addEventListener('mousemove', this.onmousemove, false);
    this.sendMouseEvent(-1, event, 0);
    event.preventDefault();
    return false;
  }

  _onMouseMove(event) {
    if (this.mouseDown === 0) {
      event.target.removeEventListener('mousemove', this.onmousemove, false);
      return false;
    }
    this.sendMouseEvent(0, event, 0);
    return false;
  }

  _onMouseUp(event) {
    event.target.removeEventListener('mousemove', this.onmousemove, false);
    this.sendMouseEvent(1, event, 0);
    event.preventDefault();
    return false;
  }

  _onWheel(event) {
    this.sendMouseEvent(2, event, Math.sign(event.deltaY));
    return false;
  }

  _onContextMenu(event) {
    event.preventDefault();
    return false;
  }
}
