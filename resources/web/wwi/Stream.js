'use strict';

import {webots} from './webots.js';
import ImageLoader from './ImageLoader.js';
import MeshLoader from './MeshLoader.js';
import WbCadShape from './nodes/WbCadShape.js';

export default class Stream {
  #onready;
  constructor(wsServer, view, onready) {
    this.wsServer = wsServer + '/';
    this.view = view;
    this.#onready = onready;
  }

  connect() {
    this.socket = new WebSocket(this.wsServer);
    const message = document.getElementById('progress-bar-message');
    const percent = message && message.innerHTML === 'Initializing...' ? 0 : 60;
    this.view.progress.setProgressBar('block', 'Connecting to Webots instance...', percent, 'Opening socket...');
    this.socket.onopen = (event) => { this.#onSocketOpen(event); };
    this.socket.onmessage = (event) => { this.#onSocketMessage(event); };
    this.socket.onclose = (event) => { this.#onSocketClose(event); };
    this.socket.onerror = (event) => {
      this.view.destroyWorld();
      this.view.onerror('WebSocket error: ' + event.data);
    };
  }

  close() {
    if (typeof this.socket !== 'undefined') {
      this.socket.close();
      this.soclet = undefined;
    }
    ImageLoader.stream = false;
    MeshLoader.stream = false;
    WbCadShape.stream = false;
  }

  #onSocketOpen(event) {
    let mode = this.view.mode;
    if (mode === 'mjpeg')
      mode += ': ' + this.view.view3D.offsetWidth + 'x' + (this.view.view3D.offsetHeight);
    else if (this.view.broadcast)
      mode += ';broadcast';

    this.view.progress.setProgressBar('block', 'same', 60 + 0.1 * 8, 'Sending mode: ' + mode);
    this.socket.send(mode);
  }

  #onSocketClose(event) {
    this.view.onerror('Disconnected from ' + this.wsServer + ' (' + event.code + ')');
    // https://tools.ietf.org/html/rfc6455#section-7.4.1
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001 && this.view.quitting === false)) {
      if (window.confirm(`Streaming server error
      Connection closed abnormally. (Error code:` + event.code + `)
      The simulation is going to be reset`))
        window.open(window.location.href, '_self');
    }
    this.view.destroyWorld();
    if (typeof this.view.onclose === 'function')
      this.view.onclose();
  }

  #onSocketMessage(event) {
    let data = event.data;

    if (data.startsWith('robot window:')) {
      const json = JSON.parse(data.substring(14));
      const robotWindow = json.window === '<generic>' ? 'generic' : json.window;

      if (json.remove) {
        const robots = webots.currentView.robots;
        for (let i = 0; i < robots.length; i++) {
          if (robots[i].name === json.robot) {
            robots.splice(i, 1);
            break;
          }
        }
      } else
        webots.currentView.robots.push({name: json.robot, window: robotWindow, main: json.main, visible: json.visible});
      if (document.getElementById('robot-window-button') !== null)
        document.getElementsByTagName('webots-view')[0].toolbar.loadRobotWindows();
    } else if (data.startsWith('robot:'))
      return 0; // We need to keep this condition, otherwise the robot window messages will be printed as errors.
    else if (data.startsWith('stdout:')) {
      this.view.onstdout(data.substring('stdout:'.length));
      return 0;
    } else if (data.startsWith('stderr:')) {
      this.view.onstderr(data.substring('stderr:'.length));
      return 0;
    } else if (data.startsWith('world:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      let currentWorld = data.substring(0, data.indexOf(':')).trim();
      data = data.substring(data.indexOf(':') + 1).trim();
      this.view.updateWorldList(currentWorld, data.split(';'));
    } else if (data.startsWith('pause:') || data === 'paused by client') {
      // Update timeout.
      if (data.startsWith('pause:')) {
        this.view.isAutomaticallyPaused = undefined;
        this.view.time = parseFloat(data.substring(data.indexOf(':') + 1).trim());
        if (this.view.currentState === 'real-time' && typeof this.onplay === 'function')
          this.onplay();
        else if ((this.view.currentState === 'run' || this.view.currentState === 'fast') && typeof this.onrun === 'function')
          this.onrun();
      }
    } else if (data === 'real-time' || data === 'run' || data === 'fast') {
      if (data === 'real-time' && this.view.currentState !== data && typeof this.onplay === 'function')
        this.onplay();
      else if ((data === 'run' || data === 'fast') && this.view.currentState !== data && typeof this.onrun === 'function')
        this.onrun();
      this.view.currentState = data;
      if (this.view.timeout >= 0)
        this.socket.send('timeout:' + this.view.timeout);
    } else if (data.startsWith('loading:')) {
      const info = data.replaceAll(':', ': ');
      data = data.substring(data.indexOf(':') + 1).trim();
      const message = data.substring(0, data.indexOf(':')).trim();
      let percent;
      if (message === 'Parsing nodes')
        percent = 0.16 * parseInt(data.substring(data.indexOf(':') + 1).trim());
      else if (message === 'Creating nodes')
        percent = 16 + 0.16 * parseInt(data.substring(data.indexOf(':') + 1).trim());
      else if (message === 'Downloading assets')
        percent = 32 + 0.16 * parseInt(data.substring(data.indexOf(':') + 1).trim());
      else if (message === 'Finalizing nodes')
        percent = 48 + 0.16 * parseInt(data.substring(data.indexOf(':') + 1).trim());
      this.view.progress.setProgressBar('block', 'Webots: ' + message + '...', percent, info + '%');
    } else if (data === 'scene load completed') {
      this.view.time = 0;
      if (document.getElementById('webots-clock'))
        document.getElementById('webots-clock').innerHTML = webots.parseMillisecondsIntoReadableTime(0);
      if (this.view.mode === 'mjpeg') {
        this.view.progress.setProgressBar('none');
        if (typeof this.#onready === 'function')
          this.#onready();
        this.view.multimediaClient.requestNewSize(); // To force the server to render once
      }
    } else if (data === 'reset finished') {
      this.view.resetSimulation();
      if (typeof this.view.x3dScene !== 'undefined' && typeof this.view.multimediaClient === 'undefined')
        this.view.x3dScene.resetViewpoint();
      if (typeof this.#onready === 'function')
        this.#onready();
    } else if (data.startsWith('time: ')) {
      this.view.time = parseFloat(data.substring(data.indexOf(':') + 1).trim());
      if (document.getElementById('webots-clock'))
        document.getElementById('webots-clock').innerHTML = webots.parseMillisecondsIntoReadableTime(this.view.time);
    } else if (data === 'delete world')
      this.view.destroyWorld();
    else {
      let messagedProcessed = false;
      if (typeof this.view.multimediaClient !== 'undefined')
        messagedProcessed = this.view.multimediaClient.processServerMessage(data);
      else if (typeof this.view.x3dScene !== 'undefined')
        messagedProcessed = this.view.x3dScene.processServerMessage(data, this.view);
      if (!messagedProcessed)
        console.log('WebSocket error: Unknown message received: "' + data + '"');
    }
  }
}
