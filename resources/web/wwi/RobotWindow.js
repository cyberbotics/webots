export default class RobotWindow {
  constructor(name, onready) {
    this.name = name;
    this.wsServer = "ws://localhost:1234/";;
    this._onready = onready;
    this.socket = new WebSocket(this.wsServer);
    this.connect();
  };

  send = function (message, robot) {
    if (this.socket.readyState === WebSocket.OPEN) {
      this.socket.send('robot:' + robot + ':' + message); //TODO1: wait socket open
   }
  };

  receive = function (message, robot) { // to be overridden
    console.log("Robot window '" + this.name + "' received message from Robot '" + robot + "': " + message);
  };

  close = function () {
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
      data = data.substring(data.indexOf('{') + 1).slice(0, -1);
      let table = data.replaceAll("\"","").split(",").map(pair => pair.split(":"));
      this.receive(table[0][1], table[1][1]);
    }
    else if (data.startsWith('application/json:')) {
      return 0;
    } else {
      console.log('WebSocket error: Unknown message received: "' + data + '"');
    }
  }

  _onSocketClose(event) { // TODO1
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001)) { // https://tools.ietf.org/html/rfc6455#section-7.4.1
      if (window.confirm(`Streaming server error
      Connection closed abnormally. (Error code:` + event.code + `)
      The simulation is going to be reset`))
        window.open(window.location.href, '_self');
    }
    close();
  }

};