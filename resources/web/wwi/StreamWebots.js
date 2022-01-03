'use strict';

import {webots} from './webots.js';

export default class StreamWebots {
  constructor(wsServer, onready) {
    this.wsServer = wsServer + '/';
    this._onready = onready;
  }

  connect() {
    this.socket = new WebSocket(this.wsServer);

    this.socket.onopen = (event) => { this._onSocketOpen(event); };
    this.socket.onmessage = (event) => { this._onSocketMessage(event); };
    this.socket.onclose = (event) => { this._onSocketClose(event); };
    this.socket.onerror = (event) => {
    };
  }

  close() {
    if (typeof this.socket !== 'undefined') {
      this.socket.close();
      this.soclet = undefined;
    }
  }

  _onSocketOpen(event) { // TODO
    this.socket.send("init");
  }

  _onSocketClose(event) { // TODO
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001)) { // https://tools.ietf.org/html/rfc6455#section-7.4.1
      if (window.confirm(`Streaming server error
      Connection closed abnormally. (Error code:` + event.code + `)
      The simulation is going to be reset`))
        window.open(window.location.href, '_self');
    }
  }

  _onSocketMessage(event) {
    let data = event.data;
    if (data.startsWith('robot:') || data.startsWith('robot window:')){
      //
      data = data.substring(data.indexOf('{') + 1).slice(0, -1);
      let table = data.split(",").map(pair => pair.split(":"));
      //receive(table[0], table[1]);
    }
    else if (data.startsWith('application/json:')) {
      return 0;
    } else {
      console.log('WebSocket error: Unknown message received: "' + data + '"');
    }
  }
}
