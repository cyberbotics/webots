export default class RobotWindow {
  constructor(name, onready) {
    this.name = name;
    this.wsServer = "ws://localhost:1234/";;
    this._onready = onready;
    this.socket = new WebSocket(this.wsServer);
    this.connect();
  };
  waitForSocketConnection = function(socket, callback) {
    setTimeout(
        function () {
          if (socket.readyState == 1) {
                //console.log("Connection is made");
                if (callback != null){
                    callback(socket);
                }
            } else {
              console.log("wait...");
              this.robotWindow.waitForSocketConnection(socket, callback);
            }

        }, 5);
  }

  send = function (message, robot) {
    this.waitForSocketConnection(this.socket, function(socket){
      socket.send('robot:' + robot + ':' + message);
    });
  };

  receive = function (message, robot) { // to be overridden
    console.log("Robot window '" + this.name + "' received message from Robot '" + robot + "': " + message);
  };

  close = function () {
    window.close();
    if (this.socket)
      this.socket.close();
    this.close();
  }

  setTitle = function (title) { //TODO1
    console.log("title: ", title)
  }

  connect() {
    this.socket.onopen = (event) => { this._onSocketOpen(event); };
    this.socket.onmessage = (event) => { this._onSocketMessage(event); };
    this.socket.onclose = (event) => { this._onSocketClose(event); };
    this.socket.onerror = (event) => {};
  }

  _onSocketOpen(event) {
    console.log("Robot windows initialized");
  }

  _onSocketMessage(event) {
    let data = event.data;
    if (data.startsWith('robot:') || data.startsWith('robot window:')){

      var message = data.match("\"message\":\"(.*)\",\"name\"")[1];
      var robot = data.match(",\"name\":\"(.*)\"}")[1];
      message = message.replace(/\\/g, "");
      this.receive(message, robot);

    }
    else if (data.startsWith('application/json:'))
      return 0;
    else if (data.startsWith('stdout:'))
      return 0;
    else if (data.startsWith('stderr:'))
      return 0;
    else { //TODO1: remove this else
      console.log('WebSocket error: Unknown message received: "' + data + '"');
    }
  }

  _onSocketClose(event) {
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001)) { // https://tools.ietf.org/html/rfc6455#section-7.4.1
      close();
    }
  }

};