import StreamWebots from './StreamWebots.js';

export default class RobotWindow {
  constructor(name) {
    this.name = name;
    this.open();
  };

  open = function () {
    const finalize = () => {
      let rwInit = true;// true if robot windows initialization is done //TODO: replace by flag.
      if (rwInit)
        console.log("Robot windows initialized");
      return;
    };

    const url = "ws://localhost:1234";
    this.stream = new StreamWebots(url, finalize);
    this.stream.connect();
  };

  send = function (message, robot) {
    this.stream.socket.send('robot:' + robot + ':' + message); //TODO: wait socket open
  };

  receive = function (message, robot) { // to be overridden
    console.log("Robot window '" + this.name + "' received message from Robot '" + robot + "': " + message);
  };

  close = function () {
    if (this.stream)
      this.stream.close();
    this.close();
  }

  setTitle = function (title) { //TODO
    console.log("title: ", title)
  }
};