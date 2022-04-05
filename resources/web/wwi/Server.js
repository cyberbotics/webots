import Stream from './Stream.js';

export default class Server {
  constructor(url, view, onready) {
    this._url = url;
    this._view = view;
    this._onready = onready;
    // url has one of the following form:
    // "ws(s)://cyberbotics1.epfl.ch:80/simple/worlds/simple.wbt", or
    // "http://cyberbotics1.epfl.ch/session?url=https://github.com/cyberbotics/webots/blob/master/projects/languages/python/worlds/example.wbt"
    // "https://webots.cloud/ajax/server/session.php?url=https://github.com/cyberbotics/webots/blob/master/projects/languages/python/worlds/example.wbt"
    const n = this._url.indexOf('?url=https://github.com/', 8);
    // 8 is for skipping the "http(s)://domain" part of the URL which smallest form is 8 characters long: "https://a"
    if (n === -1) {
      const m = url.lastIndexOf('/');
      this._project = url.substring(this._url.indexOf('/', 6) + 1, m - 7); // e.g., "simple"
      this._worldFile = url.substring(m + 1); // e.g., "simple.wbt"
    } else
      this._repository = this._url.substring(n + 5);
  }

  connect() {
    let progressMessage = document.getElementById('webots-progress-message');
    if (progressMessage)
      progressMessage.innerHTML = 'Connecting to session server...';
    let self = this;
    fetch(self._url)
      .then(response => response.text())
      .then(function(data) {
        if (data.startsWith('Error:')) {
          self.onError();
          let errorMessage = data.substring(6).trim();
          errorMessage = errorMessage.charAt(0).toUpperCase() + errorMessage.substring(1);
          alert('Session server error: ' + errorMessage);
          return;
        }
        self.socket = new WebSocket(data + '/client');
        self.socket.onopen = (event) => {
          self.onOpen(event);
        };
        self.socket.onmessage = (event) => {
          self.onMessage(event);
        };
        self.socket.onclose = (event) => {
          console.log('Disconnected from the Webots server.');
        };
        self.socket.onerror = (event) => {
          console.error('Cannot connect to the simulation server');
        };
      })
      .catch(error => {
        this.onError();
        alert('Could not connect to session server');
        console.error(error);
      });
  }

  onError() {
    document.getElementById('webots-progress').style.display = 'none';
    this._view.onquit();
  }

  onOpen(event) {
    let message = `{"start":{"url":"` + this._repository + `"`;
    if (this._view.mode === 'mjpeg')
      message += ',"mode":"mjpeg"';
    message += '}}';
    this.socket.send(message);
    let progressMessage = document.getElementById('webots-progress-message');
    if (progressMessage)
      progressMessage.innerHTML = 'Starting simulation...';
  }

  onMessage(event) {
    const message = event.data;
    if (message.indexOf('webots:ws://') === 0 || message.indexOf('webots:wss://') === 0) {
      const url = message.substring(7);
      this._httpServerUrl = url.replace(/ws/, 'http');
      if (typeof this._view.x3dScene !== 'undefined')
        this._view.x3dScene.prefix = this._httpServerUrl + '/';
      this._view.stream = new Stream(url, this._view, this._onready);
      this._view.stream.connect();
    } else if (message.indexOf('controller:') === 0 || message.indexOf('reset controller:') === 0) {
      // Need to keep it to avoid having an error message.
    } else if (message.indexOf('queue:') === 0)
      console.log('The server is saturated. Queue to wait: ' + message.substring(6) + ' client(s).');
    else if (message === '.') { // received every 5 seconds when Webots is running
      // nothing to do
    } else if (message.indexOf('error:') === 0) {
      this.onError();
      alert('Session server ' + message);
    } else if (message.indexOf('docker:') === 0)
      console.log(message);
    else if (message.indexOf('ide: ') === 0)
      this._view.ide = true;
    else
      console.log('Received an unknown message from the Webots server socket: "' + message + '"');
  }
}
