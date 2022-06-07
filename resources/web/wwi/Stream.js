'use strict';

import {webots} from './webots.js';

export default class Stream {
  constructor(wsServer, view, onready) {
    this.wsServer = wsServer + '/';
    this._view = view;
    this._onready = onready;
  }

  connect() {
    this.socket = new WebSocket(this.wsServer);
    const percent = document.getElementById('progress-bar-message').innerHTML === 'Initializing...' ? 0 : 60;
    this._view.progress.setProgressBar('block', 'Connecting to Webots instance...', percent, 'Opening socket...');
    this.socket.onopen = (event) => { this._onSocketOpen(event); };
    this.socket.onmessage = (event) => { this._onSocketMessage(event); };
    this.socket.onclose = (event) => { this._onSocketClose(event); };
    this.socket.onerror = (event) => {
      this._view.destroyWorld();
      this._view.onerror('WebSocket error: ' + event.data);
    };
  }

  close() {
    if (typeof this.socket !== 'undefined') {
      this.socket.close();
      this.soclet = undefined;
    }
  }

  _onSocketOpen(event) {
    let mode = this._view.mode;
    if (mode === 'mjpeg')
      mode += ': ' + this._view.view3D.offsetWidth + 'x' + (this._view.view3D.offsetHeight);
    else if (this._view.broadcast)
      mode += ';broadcast';

    this._view.progress.setProgressBar('block', 'same', 60 + 0.1 * 8, 'Sending mode: ' + mode);
    this.socket.send(mode);
  }

  _onSocketClose(event) {
    this._view.onerror('Disconnected from ' + this.wsServer + ' (' + event.code + ')');
    // https://tools.ietf.org/html/rfc6455#section-7.4.1
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001 && this._view.quitting === false)) {
      if (window.confirm(`Streaming server error
      Connection closed abnormally. (Error code:` + event.code + `)
      The simulation is going to be reset`))
        window.open(window.location.href, '_self');
    }
    this._view.destroyWorld();
    if (typeof this._view.onclose === 'function')
      this._view.onclose();
  }

  _onSocketMessage(event) {
    let data = event.data;
    if (data.startsWith('robot:') || data.startsWith('robot window:'))
      return 0; // We need to keep this condition, otherwise the robot window messages will be printed as errors.
    else if (data.startsWith('stdout:')) {
      this._view.onstdout(data.substring('stdout:'.length));
      return 0;
    } else if (data.startsWith('stderr:')) {
      this._view.onstderr(data.substring('stderr:'.length));
      return 0;
    } else if (data.startsWith('world:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      let currentWorld = data.substring(0, data.indexOf(':')).trim();
      data = data.substring(data.indexOf(':') + 1).trim();
      this._view.updateWorldList(currentWorld, data.split(';'));
    } else if (data.startsWith('pause:') || data === 'paused by client') {
      // Update timeout.
      if (data.startsWith('pause:')) {
        this._view.isAutomaticallyPaused = undefined;
        this._view.time = parseFloat(data.substring(data.indexOf(':') + 1).trim());
        if (this._view.currentState === 'real-time' && typeof this.onplay === 'function')
          this.onplay();
        else if ((this._view.currentState === 'run' || this._view.currentState === 'fast') && typeof this.onrun === 'function')
          this.onrun();
      }
    } else if (data === 'real-time' || data === 'run' || data === 'fast') {
      if (data === 'real-time' && this._view.currentState !== data && typeof this.onplay === 'function')
        this.onplay();
      else if ((data === 'run' || data === 'fast') && this._view.currentState !== data && typeof this.onrun === 'function')
        this.onrun();
      this._view.currentState = data;
      if (this._view.timeout >= 0)
        this.socket.send('timeout:' + this._view.timeout);
    } else if (data.startsWith('loading:')) {
      const info = data.replaceAll(':', ': ');
      data = data.substring(data.indexOf(':') + 1).trim();
      const message = data.substring(0, data.indexOf(':')).trim()
      let percent;
      if (message == 'Parsing nodes')
        percent = 0.16 * parseInt(data.substring(data.indexOf(':') + 1).trim());
      else if (message == 'Creating nodes')
        percent = 16 + 0.16 * parseInt(data.substring(data.indexOf(':') + 1).trim());
      else if (message == 'Downloading assets')
        percent = 32 + 0.16 * parseInt(data.substring(data.indexOf(':') + 1).trim());
        else if (message == 'Finalizing nodes')
        percent = 48 + 0.16 * parseInt(data.substring(data.indexOf(':') + 1).trim());
      this._view.progress.setProgressBar('block', 'Webots: ' + message, percent, info);
    } else if (data === 'scene load completed') {
      this._view.time = 0;
      if (document.getElementById('webots-clock'))
        document.getElementById('webots-clock').innerHTML = webots.parseMillisecondsIntoReadableTime(0);
      if (this._view.mode === 'mjpeg') {
        this._view.progress.setProgressBar('none');
        if (typeof this._onready === 'function')
          this._onready();
        this._view.multimediaClient.requestNewSize(); // To force the server to render once
      }
    } else if (data === 'reset finished') {
      this._view.resetSimulation();
      if (typeof this._view.x3dScene !== 'undefined' && typeof this._view.multimediaClient === 'undefined')
        this._view.x3dScene.resetViewpoint();
      if (typeof this._onready === 'function')
        this._onready();
    } else if (data.startsWith('time: ')) {
      this._view.time = parseFloat(data.substring(data.indexOf(':') + 1).trim());
      if (document.getElementById('webots-clock'))
        document.getElementById('webots-clock').innerHTML = webots.parseMillisecondsIntoReadableTime(this._view.time);
    } else if (data === 'delete world')
      this._view.destroyWorld();
    else {
      let messagedProcessed = false;
      if (typeof this._view.multimediaClient !== 'undefined')
        messagedProcessed = this._view.multimediaClient.processServerMessage(data);
      else if (typeof this._view.x3dScene !== 'undefined')
        messagedProcessed = this._view.x3dScene.processServerMessage(data, this._view);
      if (!messagedProcessed)
        console.log('WebSocket error: Unknown message received: "' + data + '"');
    }
  }
}
