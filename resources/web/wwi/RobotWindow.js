import {getGETQueryValue} from './request_methods.js';

export default class RobotWindow {
  constructor() {
    this.name = decodeURI(getGETQueryValue('name', 'undefined'));
    if (window.location.href.includes('/~WEBOTS_HOME/'))
      this.wsServer = window.location.href.substring(0, window.location.href.indexOf('/~WEBOTS_HOME/') + 1);
    else
      this.wsServer = window.location.href.substring(0, window.location.href.indexOf('/robot_windows/') + 1);
    this.wsServer = this.wsServer.replace('https://', 'wss://').replace('http://', 'ws://');
    this.socket = new WebSocket(this.wsServer);
    this.pendingMsgs = [];
    this.connect();
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
    this.socket.onopen = (event) => { this.#onSocketOpen(event); };
    this.socket.onmessage = (event) => { this.#onSocketMessage(event); };
    this.socket.onclose = (event) => { this.#onSocketClose(event); };
    this.socket.onerror = (event) => { };
    this.send('init robot window');
  }

  #onSocketOpen(event) {
    while (this.pendingMsgs.length > 0)
      this.send(this.pendingMsgs.shift());
  }

  #onSocketMessage(event) {
    const socketData = event.data.split(/\r?\n/);
    for (let i = 0, len = socketData.length; i < len; i++) {
      const data = socketData[i];
      const ignoreData = ['application/json:', 'stdout:', 'stderr:'].some(sw => data.startsWith(sw));
      if (data.startsWith('robot:')) {
        let message = data.match('"message":"(.*)","name"')[1];
        const robot = data.match(',"name":"(.*)"}')[1];
        message = message.replace(/\\/g, '');
        if (this.name === robot) // receive only the messages of our robot.
          this.receive(message, robot);
      } else if (!ignoreData)
        console.log('WebSocket error: Unknown message received: "' + data + '"');
    }
  }

  #onSocketClose(event) {
    // https://tools.ietf.org/html/rfc6455#section-7.4.1
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001)) {
      if (this.socket)
        this.socket.close();
      window.close();
    }
  }
};
