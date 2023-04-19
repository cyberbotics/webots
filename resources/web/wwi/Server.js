import Stream from './Stream.js';
import ImageLoader from './ImageLoader.js';
import MeshLoader from './MeshLoader.js';
import WbCadShape from './nodes/WbCadShape.js';

export default class Server {
  #httpServerUrl;
  #project;
  #onready;
  #repository;
  #url;
  #view;
  #worldFile;
  constructor(url, view, onready) {
    this.#url = url;
    this.#view = view;
    this.#onready = onready;
    // url has one of the following form:
    // "ws(s)://cyberbotics1.epfl.ch:80/simple/worlds/simple.wbt", or
    // "http://cyberbotics1.epfl.ch/session?url=https://github.com/cyberbotics/webots/blob/master/projects/languages/python/worlds/example.wbt"
    // "https://webots.cloud/ajax/server/session.php?url=https://github.com/cyberbotics/webots/blob/master/projects/languages/python/worlds/example.wbt"
    const n = this.#url.indexOf('?url=https://github.com/', 8);
    // 8 is for skipping the "http(s)://domain" part of the URL which smallest form is 8 characters long: "https://a"
    if (n === -1) {
      const m = url.lastIndexOf('/');
      this.#project = url.substring(this.#url.indexOf('/', 6) + 1, m - 7); // e.g., "simple"
      this.#worldFile = url.substring(m + 1); // e.g., "simple.wbt"
    } else
      this.#repository = this.#url.substring(n + 5);
  }

  connect() {
    this.#view.progress.setProgressBar('block', 'Connecting to session server...', 0);
    let self = this;
    fetch(self.#url)
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
          clearTimeout(self.timeout);
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
    this.#view.progress.setProgressBar('none');
    this.#view.onquit();
  }

  onOpen(event) {
    let message = `{"start":{"url":"` + this.#repository + `"`;
    if (this.#view.mode === 'mjpeg')
      message += ',"mode":"mjpeg"';
    message += '}}';
    this.socket.send(message);
    this.#view.progress.setProgressBar('block', 'Starting simulation...', 5, 'Communication socket open...');
  }

  onMessage(event) {
    const message = event.data;
    if (message.indexOf('webots:ws://') === 0 || message.indexOf('webots:wss://') === 0) {
      const url = message.substring(7);
      this.#httpServerUrl = url.replace(/ws/, 'http');
      if (typeof this.#view.x3dScene !== 'undefined')
        this.#view.prefix = this.#httpServerUrl + '/';
      this.#view.stream = new Stream(url, this.#view, this.#onready);
      ImageLoader.stream = true;
      MeshLoader.stream = true;
      WbCadShape.stream = true;
      this.#view.stream.connect();
    } else if (message.indexOf('controller:') === 0 || message.indexOf('reset controller:') === 0) {
      // Need to keep it to avoid having an error message.
    } else if (message.indexOf('queue:') === 0)
      console.log('The server is saturated. Queue to wait: ' + message.substring(6) + ' client(s).');
    else if (message === '.') { // received every 5 seconds when Webots is running
      // nothing to do
    } else if (message.indexOf('error:') === 0) {
      this.onError();
      alert('Session server ' + message);
    } else if (message.indexOf('loading:') === 0) {
      let body = message.substring(9);
      let percent;
      if (document.getElementById('webots-progress-bar-percent'))
        percent = document.getElementById('webots-progress-bar-percent').value;
      else if (body.startsWith('Creating network'))
        percent = 20;
      else if (body.startsWith('Step '))
        percent = 20 + 65 * parseInt(body.charAt(5)) / (parseInt(body.charAt(7)) + 1);
      else if (body.endsWith('done'))
        percent = 85;
      else if (body.startsWith('webots'))
        percent = 90;
      this.#view.progress.setProgressBar('block', 'same', 5 + 0.6 * percent, body);
    } else if (message.indexOf('ide: ') === 0)
      this.#view.ide = true;
    else if (message.indexOf('shutdownTimeout: ') === 0) {
      const shutdownTimeout = parseFloat(message.substring(17)) - 300; // Warning is issued five minutes before closing
      if (shutdownTimeout > 0)
        this.timeout = setTimeout(() => {
          alert('Warning: the time limit is almost reached.\nThe simulation will be automatically closed in 5 minutes');
        }, shutdownTimeout * 1000);
    } else
      console.log('Received an unknown message from the Webots server socket: "' + message + '"');
  }
}
