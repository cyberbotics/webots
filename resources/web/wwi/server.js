/* global webots, Stream, TextureLoader */
'use strict';

class Server { // eslint-disable-line no-unused-vars
  constructor(url, view, onready) {
    this.url = url;
    this.view = view;
    this.onready = onready;
    // url has one of the following form:
    // "ws(s)://cyberbotics1.epfl.ch:80/simple/worlds/simple.wbt", or
    // "wss://cyberbotics1.epfl.ch/1999/session
    //  ?url=webots://github.com/cyberbotics/webots/branch/master/projects/languages/python/worlds/example.wbt"
    const n = this.url.indexOf('/session?url=', 6);
    // 6 is for skipping the "ws(s)://domain" part of the URL which smallest form is 6 characters long: "ws://a"
    if (n === -1) {
      const m = url.lastIndexOf('/');
      this.project = url.substring(this.url.indexOf('/', 6) + 1, m - 7); // e.g., "simple"
      this.worldFile = url.substring(m + 1); // e.g., "simple.wbt"
    } else
      this.repository = this.url.substring(n + 13);
    this.controllers = [];
  }

  connect() {
    let xhr = new XMLHttpRequest();
    const n = this.url.indexOf('/session?url=', 6);
    const url = 'http' + (n > 0 ? this.url.substring(2, n + 8) : this.url.substring(2, this.url.indexOf('/', 6)) + '/session');
    xhr.open('GET', url, true);
    $('#webotsProgressMessage').html('Connecting to session server...');
    xhr.onreadystatechange = (e) => {
      if (xhr.readyState !== 4)
        return;
      if (xhr.status !== 200)
        return;
      const data = xhr.responseText;
      if (data.startsWith('Error:')) {
        $('#webotsProgress').hide();
        let errorMessage = data.substring(6).trim();
        errorMessage = errorMessage.charAt(0).toUpperCase() + errorMessage.substring(1);
        webots.alert('Session server error', errorMessage);
        return;
      }
      this.socket = new WebSocket(data + '/client');
      this.socket.onopen = (event) => {
        this.onOpen(event);
      };
      this.socket.onmessage = (event) => {
        this.onMessage(event);
      };
      this.socket.onclose = (event) => {
        this.view.console.info('Disconnected to the Webots server.');
      };
      this.socket.onerror = (event) => {
        this.view.console.error('Cannot connect to the simulation server');
      };
    };
    xhr.send();
  }

  onOpen(event) {
    if (this.repository) {
      let message = `{"start":{"url":"${this.repository}"`;
      if (this.view.mode === 'mjpeg')
        message += ',"mode":"mjpeg"';
      message += '}}';
      this.socket.send(message);
    } else { // legacy format
      const host = location.protocol + '//' + location.host.replace(/^www./, ''); // remove 'www' prefix
      if (typeof webots.User1Id === 'undefined')
        webots.User1Id = '';
      if (typeof webots.User1Name === 'undefined')
        webots.User1Name = '';
      if (typeof webots.User1Authentication === 'undefined')
        webots.User1Authentication = '';
      if (typeof webots.User2Id === 'undefined')
        webots.User2Id = '';
      if (typeof webots.User2Name === 'undefined')
        webots.User2Name = '';
      if (typeof webots.CustomData === 'undefined')
        webots.CustomData = '';
      this.socket.send('{ "init" : [ "' + host + '", "' + this.project + '", "' + this.worldFile + '", "' +
        webots.User1Id + '", "' + webots.User1Name + '", "' + webots.User1Authentication + '", "' +
        webots.User2Id + '", "' + webots.User2Name + '", "' + webots.CustomData + '" ] }');
    }
    $('#webotsProgressMessage').html('Starting simulation...');
  }

  onMessage(event) {
    const message = event.data;
    if (message.indexOf('webots:ws://') === 0 || message.indexOf('webots:wss://') === 0) {
      const url = message.substring(7);
      this.httpServerUrl = url.replace(/ws/, 'http');
      TextureLoader.setTexturePathPrefix(this.httpServerUrl + '/'); // Serve the texture images. SSL prefix is supported.
      this.view.stream = new Stream(url, this.view, this.onready);
      this.view.stream.connect();
    } else if (message.indexOf('controller:') === 0) {
      const n = message.indexOf(':', 11);
      let controller = {};
      controller.name = message.substring(11, n);
      controller.port = message.substring(n + 1);
      this.view.console.info('Using controller ' + controller.name + ' on port ' + controller.port);
      this.controllers.push(controller);
    } else if (message.indexOf('queue:') === 0)
      this.view.console.error('The server is saturated. Queue to wait: ' + message.substring(6) + ' client(s).');
    else if (message === '.') { // received every 5 seconds when Webots is running
      // nothing to do
    } else if (message.indexOf('reset controller:') === 0)
      this.view.stream.socket.send('sync controller:' + message.substring(18).trim());
    else
      console.log('Received an unknown message from the Webots server socket: "' + message + '"');
  }

  resetController(filename) {
    this.socket.send('{ "reset controller" : "' + filename + '" }');
  }
}
