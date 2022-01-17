import {getGETQueryValue} from './request_methods.js';
export default class RobotWindow {
  constructor(onready) {
    this.name = getGETQueryValue('name', 'undefined');
    this.wsServer = "ws://localhost:1234/";;
    this._onready = onready;
    this.socket = new WebSocket(this.wsServer);
    this.pendingMsgs = [];
    this.connect();
  };

  send(message) {
    if (this.socket.readyState !== 1) {
      this.pendingMsgs.push(message);
    } else {
      if (message === "init"){
        this.socket.send('robot_window:' + message);
      }
      else
        this.socket.send('robot:' + this.name + ':' + message);
    }
  };

  receive(message, robot) { // to be overridden
    console.log("Robot window received message from Robot '" + robot + "': " + message);
  };

  close() {
    window.close();
    if (this.socket)
      this.socket.close();
    this.close();
  }

  setTitle(title) {
    document.title = title;
  }

  connect() {
    this.socket.onopen = (event) => { this._onSocketOpen(event); };
    this.socket.onmessage = (event) => { this._onSocketMessage(event); };
    this.socket.onclose = (event) => { this._onSocketClose(event); };
    this.socket.onerror = (event) => { };
    this.send("init");
  }

  _onSocketOpen(event) {
    while (this.pendingMsgs.length > 0)
      this.send(this.pendingMsgs.shift());
  }

  _onSocketMessage(event) {
    let data = event.data;
    if (data.startsWith('robot:')) {
      var message = data.match("\"message\":\"(.*)\",\"name\"")[1];
      var robot = data.match(",\"name\":\"(.*)\"}")[1];
      message = message.replace(/\\/g, "");
      this.name = robot;
      this.receive(message, robot);
    }
    else if (data.startsWith('application/json:'))
      return 0;
    else if (data.startsWith('stdout:'))
      return 0;
    else if (data.startsWith('stderr:'))
      return 0;
    else //TODO1: remove this else
      console.log('WebSocket error: Unknown message received: "' + data + '"');
  }

  _onSocketClose(event) {
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001)) { // https://tools.ietf.org/html/rfc6455#section-7.4.1
      close();
    }
  }

};