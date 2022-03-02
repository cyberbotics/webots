import {getGETQueryValue} from './request_methods.js';

export default class RobotWindow {
  constructor(onready) {
    this.name = decodeURI(getGETQueryValue('name', 'undefined'));
    this.wsServer = window.location.href.substring(0, window.location.href.indexOf('/robot_windows/') + 1).replace('https://', 'wss://').replace('http://', 'ws://');
    this._onready = onready;
    this.socket = new WebSocket(this.wsServer);
    this.pendingMsgs = [];
    this.connect();
    console.log("RobotWindow loaded");
  };

  send(message) {
    if (this.socket.readyState !== 1)
      this.pendingMsgs.push(message);
    else
      this.socket.send('robot:' + this.name + ':' + message);
  };

  receive(message, robot) { // to be overridden
    console.log("Robot window received message from Robot '" + robot + "': " + message);
  };

  setTitle(title) {
    document.title = title;
  }

  connect() {
    this.socket.onopen = (event) => { this._onSocketOpen(event); };
    this.socket.onmessage = (event) => { this._onSocketMessage(event); };
    this.socket.onclose = (event) => { this._onSocketClose(event); };
    this.socket.onerror = (event) => { };
    this.send('init robot window');
  }

  _onSocketOpen(event) {
    while (this.pendingMsgs.length > 0)
      this.send(this.pendingMsgs.shift());
  }

  _onSocketMessage(event) {
    let data = event.data;
    const ignoreData = ['application/json:', 'stdout:', 'stderr:'].some(sw => data.startsWith(sw));
    if (data.startsWith('robot:')) {
      let message = data.match('"message":"(.*)","name"')[1];
      let robot = data.match(',"name":"(.*)"}')[1];
      message = message.replace(/\\/g, '');
      if (this.name === robot) // receive only the messages of our robot.
        this.receive(message, robot);
    } else if (ignoreData)
      return 0;
    else
      console.log('WebSocket error: Unknown message received: "' + data + '"');
  }

  _onSocketClose(event) {
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001)) { // https://tools.ietf.org/html/rfc6455#section-7.4.1
      if (this.socket)
        this.socket.close();
      window.close();
      this.close();
    }
  }
};
