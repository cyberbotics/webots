import {webots} from './webots.js';
import {exitFullscreen} from './fullscreen_handler.js';

export default class WebotsView extends HTMLElement {
  constructor() {
    super();
    this._hasActiveAnimation = false;
    this._initialCallbackDone = false;
  }

  connectedCallback() {
    if (this._initialCallbackDone)
      return;

    this._initialCallbackDone = true;
    this.animationCSS = document.createElement('link');
    this.animationCSS.href = 'https://cyberbotics.com/wwi/R2022a/css/animation.css';
    this.animationCSS.type = 'text/css';
    this.animationCSS.rel = 'stylesheet';
    this.animationCSS.disabled = true;
    document.head.appendChild(this.animationCSS);

    this.streamingCSS = document.createElement('link');
    this.streamingCSS.href = 'https://cyberbotics.com/wwi/R2022a/css/wwi.css';
    this.streamingCSS.type = 'text/css';
    this.streamingCSS.rel = 'stylesheet';
    this.streamingCSS.disabled = true;
    document.head.appendChild(this.streamingCSS);

    const script = document.createElement('script');
    script.textContent = `var Module = [];
        Module['locateFile'] = function(path, prefix) {

        // if it's a data file, use a custom dir
        if (path.endsWith(".data"))
          return "https://cyberbotics.com/wwi/R2022a/" + path;

        // otherwise, use the default, the prefix (JS file's dir) + the path
        return prefix + path;
      }`;
    document.head.appendChild(script);
    this._x3d = this.dataset.x3d;
    this._json = this.dataset.json;

    this._server = this.dataset.server;
    this._mode = this.dataset.mode;
    this._isBroadcast = this.dataset.isBroadcast;
    this._connectCallback = this.dataset.connectCallback;
    this._disconnectCallback = this.dataset.disconnectCallback;

    this._isMobileDevice = this.dataset.isMobileDevice;

    this._init();
  }

  _loadScript(scriptUrl) {
    return new Promise(function(resolve, reject) {
      const script = document.createElement('script');
      script.onload = resolve;
      script.src = scriptUrl;
      document.head.appendChild(script);
    });
  }

  _init() {
    const promises = [];
    Module.onRuntimeInitialized = () => {
      Promise.all(promises).then(() => {
        if (typeof this._x3d !== 'undefined' && this._x3d !== '')
          this.loadAnimation(!(this.dataset.autoplay && this.dataset.autoplay === 'false'));
        else if (typeof this._server !== 'undefined' && this._server !== '')
          this.connect(this._server, this._mode, this._isBroadcast, this._isMobileDevice, this._connectCallback, this._disconnectCallback);
      });
    };
    promises.push(this._loadScript('https://git.io/glm-js.min.js'));
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022a/enum.js'));
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022a/wrenjs.js'));
  }

  setIsMobileDevice(isMobileDevice) {
    this._isMobileDevice = isMobileDevice;
  }

  // Animation's functions
  setJson(fileName) {
    this._json = fileName;
  }
  setX3d(fileName) {
    this._x3d = fileName;
  }

  loadAnimation(play) {
    if (typeof this._x3d === 'undefined') {
      console.error('No x3d file defined');
      return;
    }

    this.animationCSS.disabled = false;
    this.streamingCSS.disabled = true;

    if (typeof this._view === 'undefined')
      this._view = new webots.View(this, this._isMobileDevice);
    this._view.open(this._x3d);
    if (play !== 'undefined' && play === false)
      this._view.setAnimation(this._json, 'pause', true);
    else
      this._view.setAnimation(this._json, 'play', true);
    this._hasActiveAnimation = true;
  }

  close() {
    this._view.animation.pause();
    this._view.animation.removePlayBar();
    this._view.removeLabels();
    this._view.destroyWorld();
    this._hasActiveAnimation = false;
  }

  hasActiveAnimation() {
    return this._hasActiveAnimation;
  }

  // Streaming viewer's functions

  setServer(server) {
    this._server = server;
  }

  // 'x3d' or 'mjpeg'
  setMode(mode) {
    this._mode = mode;
  }

  setIsBroadcast(isBroadcast) {
    this._isBroadcast = isBroadcast;
  }

  setConnectCallback(connectCallback) {
    this._connectCallback = connectCallback;
  }

  setDisconnectCallback(disconnectCallback) {
    this._disconnectCallback = disconnectCallback;
  }

  /*
   * url : url of the server
   * mode : x3d or mjpeg
   * broadcast: boolean
   * mobileDevice: boolean
   * callback: function
   * disconnectCallback: function. It needs to be passed there and not in disconnect because disconnect can be called from inside the web-component
   */
  connect(url, mode, broadcast, mobileDevice, callback, disconnectCallback) {
    // This `streaming viewer` setups a broadcast streaming where the simulation is shown but it is not possible to control it.
    // For any other use, please refer to the documentation:
    // https://www.cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website
    this.animationCSS.disabled = true;
    this.streamingCSS.disabled = false;

    if (typeof this._view === 'undefined')
      this._view = new webots.View(this, mobileDevice);
    this._view.broadcast = broadcast;
    this._view.setTimeout(-1); // disable timeout that stops the simulation after a given time

    this._disconnectCallback = disconnectCallback;

    this._view.open(url, mode);
    this._view.onquit = () => this.disconnect();
    this._view.onready = _ => {
      if (typeof callback === 'function')
        callback();
    };
  }

  disconnect() {
    let exitFullscreenButton = document.getElementById('exit_fullscreenButton');
    if (exitFullscreenButton && exitFullscreenButton.style.display !== 'none')
      exitFullscreen();

    this._view.close();
    this.innerHTML = null;
    if (this._view.mode === 'mjpeg')
      this._view.multimediaClient = undefined;

    if (typeof this._disconnectCallback === 'function')
      this._disconnectCallback();
  }

  hideToolbar() {
    let toolbar = document.getElementById('toolBar');
    if (toolbar) {
      if (toolbar.style.display !== 'none')
        toolbar.style.display = 'none';
    }
  }

  showToolbar() {
    let toolbar = document.getElementById('toolBar');
    if (toolbar) {
      if (toolbar.style.display !== 'block')
        toolbar.style.display = 'block';
    }
  }

  showQuit(enable) {
    webots.showQuit = enable;
  }

  showRevert(enable) {
    webots.showRevert = enable;
  }

  showRun(enable) {
    webots.showRun = enable;
  }

  sendMessage(message) {
    if (typeof this._view !== 'undefined' && this._view.stream.socket.readyState === 1)
      this._view.stream.socket.send(message);
  }

  _load(scriptUrl) {
    return new Promise(function(resolve, reject) {
      let script = document.createElement('script');
      script.onload = resolve;
      script.src = scriptUrl;
      document.head.appendChild(script);
    });
  }
}

window.customElements.define('webots-view', WebotsView);
