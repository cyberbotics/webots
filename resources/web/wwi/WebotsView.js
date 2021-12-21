import {webots} from './webots.js';
import {exitFullscreen} from './fullscreen_handler.js';

export default class WebotsView extends HTMLElement {
  constructor() {
    super();
    this._hasAnimation = false;
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
        this.initializationComplete = true;
        let scene = this.dataset.scene;
        let animation = this.dataset.animation;
        let isMobileDevice = this.dataset.isMobileDevice;
        let server = this.dataset.server;
        if ((typeof scene !== 'undefined' && scene !== '') && typeof animation !== 'undefined' && animation !== '')
          this.loadAnimation(scene, animation, isMobileDevice, !(this.dataset.autoplay && this.dataset.autoplay === 'false'));
        else if (typeof scene !== 'undefined' && scene !== '')
          this.loadScene(scene, isMobileDevice);
        else if (typeof server !== 'undefined' && server !== '')
          this.connect(server, this.dataset.mode, this.dataset.isBroadcast, isMobileDevice, this.dataset.connectCallback, this.dataset.disconnectCallback);
      });
    };
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022a/dependencies/glm-js.min.js'));
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022a/enum.js'));
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022a/wrenjs.js'));
  }

  _closeWhenDOMElementRemoved() {
    // https://stackoverflow.com/questions/52834774/dom-event-when-element-is-removed
    let observer = new MutationObserver(() => {
      if (!document.body.contains(this)) {
        this.close();
        observer.disconnect();
      }
    });
    observer.observe(document.body, {childList: true, subtree: true});
  }

  close() {
    if (this._hasAnimation)
      this._closeAnimation();
    else if (this._hasScene)
      this._closeScene();
    else if (typeof this._view !== 'undefined' && typeof this._view.stream !== 'undefined' && typeof this._view.stream.socket !== 'undefined')
      this._disconnect();
  }
  // Animation's functions
  loadAnimation(scene, animation, play, isMobileDevice) {
    if (typeof scene === 'undefined') {
      console.error('No x3d file defined');
      return;
    }

    if (!this.initializationComplete)
      setTimeout(() => this.loadAnimation(scene, animation, play, isMobileDevice), 500);
    else {
      // terminate the previous activity if any
      this.close();

      console.time('Loaded in: ');
      this.animationCSS.disabled = false;
      this.streamingCSS.disabled = true;

      if (typeof this._view === 'undefined')
        this._view = new webots.View(this, isMobileDevice);
      this._view.open(scene);
      if (play !== 'undefined' && play === false)
        this._view.setAnimation(animation, 'pause', true);
      else
        this._view.setAnimation(animation, 'play', true);
      this._hasAnimation = true;
      this._closeWhenDOMElementRemoved();
    }
  }

  _closeAnimation() {
    this._view.animation.pause();
    this._view.animation.removePlayBar();
    this._view.removeLabels();
    this._view.destroyWorld();
    this._view.animation = undefined;
    this._hasAnimation = false;
    this.innerHTML = null;
    this.animationCSS.disabled = true;
  }

  hasAnimation() {
    return this._hasAnimation;
  }

  // Streaming viewer's functions

  /*
   * url : url of the server
   * mode : x3d or mjpeg
   * broadcast: boolean
   * isMobileDevice: boolean
   * callback: function
   * disconnectCallback: function. It needs to be passed there and not in disconnect because disconnect can be called from inside the web-component
   */
  connect(server, mode, broadcast, isMobileDevice, callback, disconnectCallback) {
    // This `streaming viewer` setups a broadcast streaming where the simulation is shown but it is not possible to control it.
    // For any other use, please refer to the documentation:
    // https://www.cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website

    if (!this.initializationComplete)
      setTimeout(() => this.connect(server, mode, broadcast, isMobileDevice, callback, disconnectCallback), 500);
    else {
      // terminate the previous activity if any
      this.close();
      console.time('Loaded in: ');

      this.animationCSS.disabled = true;
      this.streamingCSS.disabled = false;

      if (typeof this._view === 'undefined')
        this._view = new webots.View(this, isMobileDevice);
      this._view.broadcast = broadcast;
      this._view.setTimeout(-1); // disable timeout that stops the simulation after a given time

      this._disconnectCallback = disconnectCallback;
      this._view.open(server, mode);
      this._view.onquit = () => this._disconnect();
      this._view.onready = _ => {
        if (typeof callback === 'function')
          callback();
      };
      this._closeWhenDOMElementRemoved();
    }
  }

  _disconnect() {
    let exitFullscreenButton = document.getElementById('exit_fullscreenButton');
    if (exitFullscreenButton && exitFullscreenButton.style.display !== 'none')
      exitFullscreen();

    this._view.close();
    this.innerHTML = null;
    if (this._view.mode === 'mjpeg')
      this._view.multimediaClient = undefined;

    if (typeof this._disconnectCallback === 'function')
      this._disconnectCallback();
    this.streamingCSS.disabled = true;
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

  displayQuit(enable) {
    webots.showQuit = enable;
  }

  displayRevert(enable) {
    webots.showRevert = enable;
  }

  displayRun(enable) {
    webots.showRun = enable;
  }

  sendMessage(message) {
    if (typeof this._view !== 'undefined' && this._view.stream.socket.readyState === 1)
      this._view.stream.socket.send(message);
  }

  // Scene functions
  loadScene(scene, isMobileDevice) {
    if (typeof scene === 'undefined') {
      console.error('No x3d file defined');
      return;
    }

    if (!this.initializationComplete)
      setTimeout(() => this.loadScene(scene, isMobileDevice), 500);
    else {
      // terminate the previous activity if any
      this.close();

      console.time('Loaded in: ');
      this.animationCSS.disabled = false;
      this.streamingCSS.disabled = true;

      if (typeof this._view === 'undefined')
        this._view = new webots.View(this, isMobileDevice);
      this._view.open(scene);
      this._hasScene = true;

      this._closeWhenDOMElementRemoved();
    }
  }

  _closeScene() {
    this._view.destroyWorld();
    this._hasScene = false;
    this.innerHTML = null;
    this.animationCSS.disabled = true;
  }
}

window.customElements.define('webots-view', WebotsView);
