import {exitFullscreen} from './fullscreen_handler.js';
import Toolbar from './Toolbar.js';
import ProtoConverter from './ProtoConverter.js';
import {webots} from './webots.js';
import {changeGtaoLevel} from './nodes/wb_preferences.js';
import WbWorld from './nodes/WbWorld.js';

/* The following member variables can be set by the application:

webotsView.showIde               // defines whether the IDE button should be displayed.
webotsView.showInfo              // defines whether the info button should be displayed.
webotsView.showPlay              // defines whether the play button should be displayed.
webotsView.showQuit              // defines whether the quit button should be displayed.
webotsView.showReload            // defines whether the reload button should be displayed.
webotsView.showReset             // defines whether the reset button should be displayed.
webotsView.showRobotWindow       // defines whether the robot window button should be displayed.
webotsView.showRun               // defines whether the run button should be displayed.
webotsView.showStep              // defines whether the step button should be displayed.
webotsView.showTerminal          // defines whether the terminal button should be displayed.
webotsView.showCustomWindow      // defines whether the custom window button should be displayed.
webotsView.showWorldSelection    // defines whether the world selection button should be displayed.
*/

export default class WebotsView extends HTMLElement {
  #hasAnimation;
  #hasProto;
  #hasScene;
  #initialCallbackDone;
  #view;
  constructor() {
    super();
    this.#hasAnimation = false;
    this.#initialCallbackDone = false;
  }

  connectedCallback() {
    if (this.#initialCallbackDone)
      return;

    this.#initialCallbackDone = true;

    this.toolbarCss = document.createElement('link');
    this.toolbarCss.href = 'https://cyberbotics.com/wwi/R2023a/css/toolbar.css';
    this.toolbarCss.type = 'text/css';
    this.toolbarCss.rel = 'stylesheet';
    document.head.appendChild(this.toolbarCss);

    this.progressCss = document.createElement('link');
    this.progressCss.href = 'https://cyberbotics.com/wwi/R2023a/css/progress.css';
    this.progressCss.type = 'text/css';
    this.progressCss.rel = 'stylesheet';
    document.head.appendChild(this.progressCss);

    const script = document.createElement('script');
    script.textContent = `var Module = [];
        Module['locateFile'] = function(path, prefix) {

        // if it's a data file, use a custom dir
        if (path.endsWith('.data'))
          return 'https://cyberbotics.com/wwi/R2023a/' + path;

        // otherwise, use the default, the prefix (JS file's dir) + the path
        return prefix + path;
      }`;
    document.head.appendChild(script);
    this.#init();
  }

  #loadScript(scriptUrl) {
    return new Promise(function(resolve, reject) {
      const script = document.createElement('script');
      script.onload = resolve;
      script.src = scriptUrl;
      document.head.appendChild(script);
    });
  }

  #init() {
    const promises = [];
    Module.onRuntimeInitialized = () => {
      Promise.all(promises).then(() => {
        this.initializationComplete = true;
        let scene = this.dataset.scene;
        let animation = this.dataset.animation;
        let proto = this.dataset.proto;
        let thumbnail = this.dataset.thumbnail;
        let isMobileDevice = this.dataset.isMobileDevice;
        let server = this.dataset.server;
        if ((typeof scene !== 'undefined' && scene !== '') && typeof animation !== 'undefined' && animation !== '')
          this.loadAnimation(scene, animation, !(this.dataset.autoplay && this.dataset.autoplay === 'false'), isMobileDevice,
            thumbnail);
        else if (typeof scene !== 'undefined' && scene !== '')
          this.loadScene(scene, isMobileDevice, thumbnail);
        else if (typeof server !== 'undefined' && server !== '')
          this.connect(server, this.dataset.mode, this.dataset.isBroadcast, isMobileDevice, this.dataset.timeout, thumbnail);
        else if (typeof proto !== 'undefined' && proto !== '')
          this.loadProto(proto, isMobileDevice, thumbnail);
      });
    };

    promises.push(this.#loadScript('https://cyberbotics.com/wwi/R2023a/dependencies/ansi_up.js'));
    promises.push(this.#loadScript('https://cyberbotics.com/wwi/R2023a/dependencies/assimpjs.js'));
    promises.push(this.#loadScript('https://cyberbotics.com/wwi/R2023a/dependencies/glm-js.min.js'));
    promises.push(this.#loadScript('https://cyberbotics.com/wwi/R2023a/dependencies/quaternion.min.js'));
    promises.push(this.#loadScript('https://cyberbotics.com/wwi/R2023a/dependencies/libtess.min.js'));
    promises.push(this.#loadScript('https://cyberbotics.com/wwi/R2023a/enum.js'));
    promises.push(this.#loadScript('https://cyberbotics.com/wwi/R2023a/wrenjs.js'));
  }

  #closeWhenDOMElementRemoved() {
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
    if (this.#hasAnimation)
      this.#closeAnimation();
    else if (this.#hasScene || this.#hasProto)
      this.#closeScene();
    else if (typeof this.#view !== 'undefined' && typeof this.#view.stream !== 'undefined' &&
      typeof this.#view.stream.socket !== 'undefined')
      this.#disconnect();
  }

  resize() {
    this.#view?.onresize();
  }

  setWebotsMessageCallback(callback) {
    if (typeof this.#view !== 'undefined')
      this.#view.messageCallback = callback;
  }

  setWebotsErrorMessageCallback(callback) {
    if (typeof this.#view !== 'undefined')
      this.#view.errorMessageCallback = callback;
  }

  hasView() {
    return typeof this.#view !== 'undefined';
  }

  resetViewpoint() {
    if (typeof WbWorld.instance !== 'undefined' && typeof WbWorld.instance.viewpoint !== 'undefined') {
      WbWorld.instance.viewpoint.resetViewpoint();
      this.#view.x3dScene.render();
    }
  }

  // The value is updated only on the web side, do not used with simulation.
  updateNode(nodeId, field, value, render) {
    if (typeof nodeId === 'undefined' || typeof field === 'undefined' || typeof value === 'undefined' ||
      typeof this.#view === 'undefined')
      return;

    let pose = {
      'id': nodeId,
      [field]: value
    };
    this.#view.x3dScene.applyPose(pose);
    if (render)
      this.#view.x3dScene.render();
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
  loadAnimation(scene, animation, play, isMobileDevice, thumbnail) {
    if (typeof scene === 'undefined') {
      console.error('No x3d file defined');
      return;
    }

    if (!this.initializationComplete)
      setTimeout(() => this.loadAnimation(scene, animation, play, isMobileDevice, thumbnail), 500);
    else {
      // terminate the previous activity if any
      this.close();

      console.time('Loaded in: ');

      if (typeof this.#view === 'undefined')
        this.#view = new webots.View(this, isMobileDevice);
      this.#view.onready = () => {
        this.toolbar = new Toolbar(this.#view, 'animation', this);
        if (typeof this.onready === 'function')
          this.onready();
      };
      this.#view.open(scene, 'undefined', thumbnail);
      if (play !== 'undefined' && play === false)
        this.#view.setAnimation(animation, 'pause', true);
      else
        this.#view.setAnimation(animation, 'play', true);
      this.#hasAnimation = true;
      this.#closeWhenDOMElementRemoved();
    }
  }

  setCustomWindowTitle(title) {
    this?.toolbar.customWindow.setTitle(title);
  }

  setCustomWindowTooltip(tooltip) {
    this?.toolbar.customWindow.setTooltip(tooltip);
  }

  setCustomWindowContent(content) {
    this?.toolbar.customWindow.setContent(content);
  }

  #closeAnimation() {
    this.#view.animation.pause();
    if (typeof this.toolbar !== 'undefined') {
      this.toolbar.removeAnimationToolbar();
      this.toolbar = undefined;
    }
    this.#view.removeLabels();
    this.#view.destroyWorld();
    this.#view.animation = undefined;
    this.#hasAnimation = false;
    this.innerHTML = null;
  }

  hasAnimation() {
    return this.#hasAnimation;
  }

  setAnimationStepCallback(callbackFunction) {
    if (typeof this.#view !== 'undefined' && typeof this.#view.animation !== 'undefined') {
      this.#view.animation.stepCallback = callbackFunction;
      return true;
    }
  }

  // Streaming viewer's functions

  /*
   * url : url of the server
   * mode : x3d or mjpeg
   * broadcast: boolean
   * isMobileDevice: boolean
   */
  connect(server, mode, broadcast, isMobileDevice, timeout, thumbnail) {
    // This `streaming viewer` setups a broadcast streaming where the simulation is shown but it is not possible to control it.
    // For any other use, please refer to the documentation:
    // https://www.cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website

    if (!this.initializationComplete)
      setTimeout(() => this.connect(server, mode, broadcast, isMobileDevice, timeout, thumbnail), 500);
    else {
      // terminate the previous activity if any
      this.close();
      console.time('Loaded in: ');

      if (typeof this.#view === 'undefined')
        this.#view = new webots.View(this, isMobileDevice);
      this.#view.broadcast = broadcast;
      if (typeof timeout === 'undefined')
        timeout = -1; // disable timeout that stops the simulation after a given time
      this.#view.setTimeout(timeout);

      this.#view.onready = () => {
        if (typeof this.toolbar === 'undefined')
          this.toolbar = new Toolbar(this.#view, 'streaming', this);
        if (document.getElementById('robot-window-button') !== null)
          document.getElementsByTagName('webots-view')[0].toolbar.loadRobotWindows();
        if (typeof this.onready === 'function')
          this.onready();
      };
      this.#view.open(server, mode, thumbnail);
      this.#view.onquit = () => {
        if (typeof this.ondisconnect === 'function')
          this.ondisconnect();
      };
      this.#closeWhenDOMElementRemoved();
    }
  }

  #disconnect() {
    let exitFullscreenButton = document.getElementById('exit_fullscreenButton');
    if (exitFullscreenButton && exitFullscreenButton.style.display !== 'none')
      exitFullscreen();

    if (typeof this.toolbar !== 'undefined') {
      this.toolbar.removeStreamingToolbar();
      this.toolbar = undefined;
    }
    this.#view.close();
    this.innerHTML = null;
    if (this.#view.mode === 'mjpeg')
      this.#view.multimediaClient = undefined;

    if (typeof this.ondisconnect === 'function')
      this.ondisconnect();
  }

  hideToolbar() {
    this?.toolbar.hideToolbar(true);
  }

  showToolbar() {
    this?.toolbar.showToolbar(true);
  }

  sendMessage(message) {
    if (typeof this.#view !== 'undefined' && this.#view.stream.socket.readyState === 1)
      this.#view.stream.socket.send(message);
  }

  // Scene functions
  loadScene(scene, isMobileDevice, thumbnail) {
    if (typeof scene === 'undefined') {
      console.error('No x3d file defined');
      return;
    }
    if (!this.initializationComplete)
      setTimeout(() => this.loadScene(scene, isMobileDevice, thumbnail), 500);
    else {
      // terminate the previous activity if any
      this.close();

      console.time('Loaded in: ');

      if (typeof this.#view === 'undefined')
        this.#view = new webots.View(this, isMobileDevice);

      this.#view.onready = () => {
        this.toolbar = new Toolbar(this.#view, 'scene', this);
        if (typeof this.onready === 'function')
          this.onready();
      };
      this.#view.open(scene, 'undefined', thumbnail);
      this.#hasScene = true;
      this.#closeWhenDOMElementRemoved();
    }
  }

  hasScene() {
    return this.#hasScene;
  }

  #closeScene() {
    if (typeof this.toolbar !== 'undefined') {
      this.toolbar.removeToolbar();
      this.toolbar = undefined;
    }
    this.#view.destroyWorld();
    this.#hasScene = false;
    this.#hasProto = false;
    this.innerHTML = null;
  }

  loadProto(proto, isMobileDevice, thumbnail) {
    if (typeof proto === 'undefined') {
      console.error('No proto file defined');
      return;
    }

    if (!this.initializationComplete)
      setTimeout(() => this.loadProto(proto, isMobileDevice, thumbnail), 500);
    else {
      // terminate the previous activity if any
      this.close();

      console.time('Loaded in: ');
      if (typeof this.#view === 'undefined')
        this.#view = new webots.View(this, isMobileDevice);
      const protoConverter = new ProtoConverter(this.#view);
      this.#view.onready = () => {
        protoConverter.loadProto(proto);
        this.toolbar = new Toolbar(this.#view, 'scene', this);
        if (typeof this.onready === 'function')
          this.onready();
      }
      protoConverter.loadMinimalScene();
      this.#hasProto = true;
      this.#closeWhenDOMElementRemoved();
    }
  }
}

window.customElements.define('webots-view', WebotsView);
