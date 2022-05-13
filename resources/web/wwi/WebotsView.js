import {exitFullscreen} from './fullscreen_handler.js';
import Toolbar from './Toolbar.js';
import {webots} from './webots.js';
import {changeGtaoLevel} from './nodes/wb_preferences.js';
import WbWorld from './nodes/WbWorld.js';

/* The following member variables can be set by the application:

webotsView.showIde             // defines whether the IDE button should be displayed.
webotsView.showInfo            // defines whether the info button should be displayed.
webotsView.showPlay            // defines whether the play button should be displayed.
webotsView.showQuit            // defines whether the quit button should be displayed.
webotsView.showReload          // defines whether the reload button should be displayed.
webotsView.showReset           // defines whether the reset button should be displayed.
webotsView.showRobotWindow     // defines whether the robot window button should be displayed.
webotsView.showRun             // defines whether the run button should be displayed.
webotsView.showStep            // defines whether the step button should be displayed.
webotsView.showWorldSelection  // defines whether the world selection button should be displayed.
*/

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
    this.css = document.createElement('link');
    this.css.href = 'https://cyberbotics.com/wwi/R2022b/css/toolbar.css';
    this.css.type = 'text/css';
    this.css.rel = 'stylesheet';
    document.head.appendChild(this.css);

    const script = document.createElement('script');
    script.textContent = `var Module = [];
        Module['locateFile'] = function(path, prefix) {

        // if it's a data file, use a custom dir
        if (path.endsWith(".data"))
          return "https://cyberbotics.com/wwi/R2022b/" + path;

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
          this.loadAnimation(scene, animation, !(this.dataset.autoplay && this.dataset.autoplay === 'false'), isMobileDevice);
        else if (typeof scene !== 'undefined' && scene !== '')
          this.loadScene(scene, isMobileDevice);
        else if (typeof server !== 'undefined' && server !== '')
          this.connect(server, this.dataset.mode, this.dataset.isBroadcast, isMobileDevice, this.dataset.timeout);
      });
    };
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022b/dependencies/glm-js.min.js'));
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022b/dependencies/quaternion.min.js'));
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022b/enum.js'));
    promises.push(this._loadScript('https://cyberbotics.com/wwi/R2022b/wrenjs.js'));
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

  resize() {
    if (typeof this._view !== 'undefined')
      this._view.onresize();
  }

  setWebotsMessageCallback(callback) {
    if (typeof this._view !== 'undefined')
      this._view.messageCallback = callback;
  }

  setWebotsErrorMessageCallback(callback) {
    if (typeof this._view !== 'undefined')
      this._view.errorMessageCallback = callback;
  }

  hasView() {
    return typeof this._view !== 'undefined';
  }

  resetViewpoint() {
    if (typeof WbWorld.instance !== 'undefined' && typeof WbWorld.instance.viewpoint !== 'undefined') {
      WbWorld.instance.viewpoint.resetViewpoint();
      this._view.x3dScene.render();
    }
  }

  // The value is updated only on the web side, do not used with simulation.
  updateNode(nodeId, field, value, render) {
    if (typeof nodeId === 'undefined' || typeof field === 'undefined' || typeof value === 'undefined' || typeof this._view === 'undefined')
      return;

    let pose = {
      'id': nodeId,
      [field]: value
    };
    this._view.x3dScene.applyPose(pose);
    if (render)
      this._view.x3dScene.render();
  }

  getNode(id) {
    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.nodes !== 'undefined')
      return WbWorld.instance.nodes.get('n' + id);
  }

  setAmbientOcclusion(level) {
    level = Math.floor(level);
    if (level > 4)
      level = 4;
    else if (level < 1)
      level = 1;

    if (typeof this.toolbar !== 'undefined') {
      this.toolbar.changeGtao({srcElement: {id: this.toolbar.gtaoLevelToText(level)}});
      this.toolbar.settingsPane.style.visibility = 'hidden';
    } else
      changeGtaoLevel(level);
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

      if (typeof this._view === 'undefined')
        this._view = new webots.View(this, isMobileDevice);
      this._view.onready = () => {
        this.toolbar = new Toolbar(this._view, 'animation', this);
        if (typeof this.onready === 'function')
          this.onready();
      };
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
    if (typeof this.toolbar !== 'undefined') {
      this.toolbar.removeAnimationToolbar();
      this.toolbar = undefined;
    }
    this._view.removeLabels();
    this._view.destroyWorld();
    this._view.animation = undefined;
    this._hasAnimation = false;
    this.innerHTML = null;
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
   */
  connect(server, mode, broadcast, isMobileDevice, timeout) {
    // This `streaming viewer` setups a broadcast streaming where the simulation is shown but it is not possible to control it.
    // For any other use, please refer to the documentation:
    // https://www.cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website

    if (!this.initializationComplete)
      setTimeout(() => this.connect(server, mode, broadcast, isMobileDevice, timeout), 500);
    else {
      // terminate the previous activity if any
      this.close();
      console.time('Loaded in: ');

      if (typeof this._view === 'undefined')
        this._view = new webots.View(this, isMobileDevice);
      this._view.broadcast = broadcast;
      if (typeof timeout === 'undefined')
        timeout = -1; // disable timeout that stops the simulation after a given time
      this._view.setTimeout(timeout);

      this._view.onready = () => {
        if (typeof this.toolbar === 'undefined')
          this.toolbar = new Toolbar(this._view, 'streaming', this);
        if (typeof this.onready === 'function')
          this.onready();
      };
      this._view.open(server, mode);
      this._view.onquit = () => {
        if (typeof this.ondisconnect === 'function')
          this.ondisconnect();
      };
      this._closeWhenDOMElementRemoved();
    }
  }

  _disconnect() {
    let exitFullscreenButton = document.getElementById('exit_fullscreenButton');
    if (exitFullscreenButton && exitFullscreenButton.style.display !== 'none')
      exitFullscreen();

    if (typeof this.toolbar !== 'undefined') {
      this.toolbar.removeStreamingToolbar();
      this.toolbar = undefined;
    }
    this._view.close();
    this.innerHTML = null;
    if (this._view.mode === 'mjpeg')
      this._view.multimediaClient = undefined;

    if (typeof this.ondisconnect === 'function')
      this.ondisconnect();
  }

  hideToolbar() {
    if (typeof this.toolbar !== 'undefined')
      this.toolbar.hideToolbar(true);
  }

  showToolbar() {
    if (typeof this.toolbar !== 'undefined')
      this.toolbar.showToolbar(true);
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

      if (typeof this._view === 'undefined')
        this._view = new webots.View(this, isMobileDevice);

      this._view.onready = () => {
        this.toolbar = new Toolbar(this._view, 'scene', this);
        if (typeof this.onready === 'function')
          this.onready();
      };
      this._view.open(scene);
      this._hasScene = true;
      this._closeWhenDOMElementRemoved();
    }
  }

  hasScene() {
    return this._hasScene;
  }

  _closeScene() {
    if (typeof this.toolbar !== 'undefined') {
      this.toolbar.removeToolbar();
      this.toolbar = undefined;
    }
    this._view.destroyWorld();
    this._hasScene = false;
    this.innerHTML = null;
  }
}

window.customElements.define('webots-view', WebotsView);
