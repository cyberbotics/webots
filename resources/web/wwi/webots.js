import Animation from './Animation.js';
import DefaultUrl from './DefaultUrl.js';
import MouseEvents from './MouseEvents.js';
import MultimediaClient from './MultimediaClient.js';
import Selector from './Selector.js';
import Server from './Server.js';
import Stream from './Stream.js';
import SystemInfo from './system_info.js';
import X3dScene from './X3dScene.js';

import WbVector3 from './nodes/utils/WbVector3.js';

/*
 * Injects a Webots 3D view inside a HTML tag.
 * @class
 * @classdesc
 *   The Webots view object displays a 3D view on a web page.
 *   This view represents a Webots simulation world that may be
 *   connected to a webots instance running on a remote server.
 * @example
 *   // Example: Initialize from a Webots streaming server
 *   const view = new webots.View(document.getElementById("myDiv"));
 *   view.open("ws://localhost:80/simple/worlds/simple.wbt");
 *   // or view.open("ws://localhost:80");
 *   // or view.open("file.x3d");
 *   view.onready = () => {
 *       // the initialization is done
 *   }
 *   view.onclose = () => {
 *       view = null;
 *   }
 */

let webots = window.webots || {};

webots.View = class View {
  constructor(view3D, mobile) {
    webots.currentView = this;
    this.onerror = (text) => {
      console.log('%c' + text, 'color:black');
    };
    this.onstdout = (text) => {
      console.log('%c' + text, 'color:blue');
      if (typeof this.messageCallback !== 'undefined')
        this.messageCallback(text);
    };
    this.onstderr = (text) => {
      console.log('%c' + text, 'color:red');
      if (typeof this.erorMessageCallback !== 'undefined')
        this.erorMessageCallback(text);
    };
    this.onquit = () => { // You can change this behavior by overriding this onquit() method
      window.history.back(); // go back to the previous page in the navigation history
    };
    this.onresize = () => {
      if (typeof this.x3dScene !== 'undefined')
        this.x3dScene.resize();
      else if (typeof this.multimediaClient !== 'undefined')
        this.multimediaClient.requestNewSize();

      const labels = document.getElementsByClassName('webots-label');
      for (let i = labels.length - 1; i >= 0; i--) {
        const element = labels.item(i);
        element.style.fontSize = this._getHeight(this._x3dDiv) * element.size / 2.25 + 'px'; // 2.25 is an empirical value to match with Webots appearance
        element.style.left = this._getWidth(this._x3dDiv) * element.x + 'px';
        element.style.top = this._getHeight(this._x3dDiv) * element.y + 'px';
      }
    };

    window.onresize = this.onresize;

    this.view3D = view3D;
    this.view3D.className = view3D.className + ' webotsView';

    if (typeof mobile === 'undefined')
      this._mobileDevice = SystemInfo.isMobileDevice();
    else
      this._mobileDevice = mobile;

    this.fullscreenEnabled = !SystemInfo.isIOS();
    if (!this.fullscreenEnabled) {
      // Add tag needed to run standalone web page in fullscreen on iOS.
      let meta = document.createElement('meta');
      meta.name = 'apple-mobile-web-app-capable';
      meta.content = 'yes';
      document.getElementsByTagName('head')[0].appendChild(meta);
    }

    this.timeout = 60 * 1000; // default to one minute
    this.deadline = this.timeout;
    this.currentState = false;
    this.quitting = false;
  }

  setTimeout(timeout) { // expressed in seconds
    if (timeout < 0) {
      this.timeout = timeout;
      this.deadline = 0;
      return;
    }

    this.timeout = timeout * 1000; // convert to milliseconds
    this.deadline = this.timeout;
    if (typeof this.time !== 'undefined')
      this.deadline += this.time;
  }

  setAnimation(url, gui, loop) {
    if (typeof gui === 'undefined')
      gui = 'play';
    if (typeof loop === 'undefined')
      loop = true;
    this.animation = new Animation(url, this.x3dScene, this, gui, loop);
  }

  open(url, mode) {
    this.url = url;
    if (typeof mode === 'undefined')
      mode = 'x3d';
    this.mode = mode;

    const initWorld = () => {
      if (typeof this.progress === 'undefined') {
        this.progress = document.createElement('div');
        this.progress.id = 'webotsProgress';
        this.progress.innerHTML = "<div><img src='" + DefaultUrl.wwiImagesUrl() + "load_animation.gif'>" +
        "</div><div id='webotsProgressMessage'>Initializing...</div>" +
        "</div><div id='webotsProgressPercent'></div>";
        this.view3D.appendChild(this.progress);
      }

      if (document.getElementById('webotsProgress'))
        document.getElementById('webotsProgress').style.display = 'block';

      if (this._isWebSocketProtocol) {
        if (this.url.endsWith('.wbt')) { // url expected form: "wss://localhost:1999/simple/worlds/simple.wbt" or
          // "http://localhost/1999/session?url=https://github.com/cyberbotics/webots/blob/master/projects/languages/python/worlds/example.wbt"
          // "https://webots.cloud/ajax/server/session.php?url=https://github.com/cyberbotics/webots/blob/master/projects/languages/python/worlds/example.wbt"
          this._server = new Server(this.url, this, finalizeWorld);
          this._server.connect();
        } else { // url expected form: "ws://cyberbotics1.epfl.ch:80"
          const httpServerUrl = 'http' + this.url.slice(2); // replace 'ws'/'wss' with 'http'/'https'
          this.stream = new Stream(this.url, this, finalizeWorld);
          if (typeof this.x3dScene !== 'undefined')
            this.x3dScene.prefix = httpServerUrl + '/';
          this.stream.connect();
        }
      } else // assuming it's an URL to a .x3d file
        this.x3dScene.loadWorldFile(this.url, finalizeWorld);
    };

    const finalizeWorld = () => {
      if (document.getElementById('webotsProgressMessage'))
        document.getElementById('webotsProgressMessage').innerHTML = 'Loading World...';
      if (typeof this.x3dScene !== 'undefined') {
        if (!this._isWebSocketProtocol) { // skip robot windows initialization
          if (typeof this.animation !== 'undefined')
            this.animation.init(loadFinalize);
          else
            loadFinalize();
          this.onresize();
          return;
        }
      }

      loadFinalize();
    };

    let loadFinalize = () => {
      if (typeof this.multimediaClient !== 'undefined')
        // finalize multimedia client
        this.multimediaClient.finalize();

      if (typeof this.onready === 'function')
        this.onready();
    };

    if (this.broadcast)
      this.setTimeout(-1);
    this._isWebSocketProtocol = this.url.startsWith('ws://') || this.url.startsWith('wss://') || this.url.endsWith('.wbt');

    const texturePathPrefix = url.includes('/') ? url.substring(0, url.lastIndexOf('/') + 1) : '';

    if (mode === 'mjpeg') {
      this.url = url;
      this.multimediaClient = new MultimediaClient(this, this.view3D);
    } else if (typeof this.x3dScene === 'undefined') {
      this._x3dDiv = document.getElementById('view3d');
      if (this._x3dDiv === null || typeof this._x3dDiv === 'undefined') {
        this._x3dDiv = document.createElement('div');
        this._x3dDiv.id = 'view3d';
        this.view3D.appendChild(this._x3dDiv);
      }

      this._x3dDiv.className = 'webots3DView';
      this.x3dScene = new X3dScene(this._x3dDiv);
      this.x3dScene.init(texturePathPrefix);
      let param = document.createElement('param');
      param.name = 'showProgress';
      param.value = false;
      this.x3dScene.domElement.appendChild(param);
    } else {
      if (typeof this._x3dDiv !== 'undefined') {
        this.view3D.appendChild(this._x3dDiv);
        this.x3dScene.prefix = texturePathPrefix;
      }
      if (typeof this.progress !== 'undefined')
        this.view3D.appendChild(this.progress);
    }

    if (typeof this.x3dScene !== 'undefined' && typeof this.mouseEvents === 'undefined') {
      let canvas = document.getElementById('canvas');
      this.mouseEvents = new MouseEvents(this.x3dScene, canvas, this._mobileDevice);
    }

    initWorld();
  }

  close() {
    if (this.multimediaClient)
      this.multimediaClient.disconnect();
    if (this._server && this._server.socket)
      this._server.socket.close();
    if (this.stream)
      this.stream.close();
  }

  // Functions for internal use.

  updateWorldList(currentWorld, worlds) {
    if (this.broadcast)
      // Do not show world list if in broadcast mode,
      // where multiple users can connect to the same Webots instance.
      return;

    const existingCurrentWorld = typeof this.currentWorld !== 'undefined';
    this.currentWorld = currentWorld;
    this.worlds = worlds;

    if (existingCurrentWorld) {
      const webotsView = document.getElementsByTagName('webots-view')[0];
      if (webotsView && typeof webotsView.toolbar !== 'undefined' && typeof webotsView.toolbar.worldSelectionDiv !== 'undefined') {
        webotsView.toolbar.deleteWorldSelect();
        webotsView.toolbar.createWorldSelect();
      }
    }
  }

  setLabel(properties) {
    let labelElement = document.getElementById('label' + properties.id);
    if (labelElement == null) {
      labelElement = document.createElement('div');
      labelElement.id = 'label' + properties.id;
      labelElement.className = 'webots-label';
      this._x3dDiv.appendChild(labelElement);
    }

    let font = properties.font.split('/');
    font = font[font.length - 1].replace('.ttf', '');

    labelElement.style.fontFamily = font;
    labelElement.style.color = 'rgba(' + properties.color + ')';
    labelElement.style.fontSize = this._getHeight(this._x3dDiv) * properties.size / 2.25 + 'px'; // 2.25 is an empirical value to match with Webots appearance
    labelElement.style.left = this._getWidth(this._x3dDiv) * properties.x + 'px';
    labelElement.style.top = this._getHeight(this._x3dDiv) * properties.y + 'px';
    labelElement.x = properties.x;
    labelElement.y = properties.y;
    labelElement.size = properties.size;

    if (properties.text.includes('█'))
      properties.text = properties.text.replaceAll('█', '<span style="background:' + labelElement.style.color + '"> </span>');

    labelElement.innerHTML = properties.text;
  }

  removeLabels() {
    const labels = document.getElementsByClassName('webots-label');
    for (let i = labels.length - 1; i >= 0; i--) {
      const element = labels.item(i);
      element.parentNode.removeChild(element);
    }
  }

  resetSimulation() {
    if (document.getElementById('webotsProgress'))
      document.getElementById('webotsProgress').style.display = 'none';
    this.removeLabels();
    if (document.getElementById('webotsClock'))
      document.getElementById('webotsClock').innerHTML = webots.parseMillisecondsIntoReadableTime(0);
    this.deadline = this.timeout;
    if (document.getElementById('webotsTimeout')) {
      if (this.deadline >= 0)
        document.getElementById('webotsTimeout').innerHTML = webots.parseMillisecondsIntoReadableTime(this.deadline);
      else
        document.getElementById('webotsTimeout').innerHTML = webots.parseMillisecondsIntoReadableTime(0);
    }
  }

  quitSimulation() {
    if (this.broadcast)
      return;
    this.close();
    if (document.getElementById('webotsProgressMessage'))
      document.getElementById('webotsProgressMessage').innerHTML = 'Bye bye...';
    if (document.getElementById('webotsProgress'))
      document.getElementById('webotsProgress').style.display = 'block';
    setTimeout(() => {
      if (document.getElementById('webotsProgress'))
        document.getElementById('webotsProgress').style.display = 'none';
    }, 1000);
    this.quitting = true;
    this.onquit();
  }

  destroyWorld() {
    if (typeof this.x3dScene !== 'undefined')
      this.x3dScene.destroyWorld();
    this.removeLabels();

    if (typeof this.mouseEvents !== 'undefined' && typeof this.mouseEvents.picker !== 'undefined') {
      this.mouseEvents.picker.selectedId = -1;
      this.mouseEvents.picker.coordinates = new WbVector3();
      Selector.reset();
    }
  }

  _getHeight(el) {
    const s = window.getComputedStyle(el, null);
    return el.clientHeight - parseInt(s.getPropertyValue('padding-top')) - parseInt(s.getPropertyValue('padding-bottom'));
  }

  _getWidth(el) {
    const s = window.getComputedStyle(el, null);
    return el.clientWidth - parseInt(s.getPropertyValue('padding-right')) - parseInt(s.getPropertyValue('padding-left'));
  }
};

webots.parseMillisecondsIntoReadableTime = (milliseconds) => {
  const hours = (milliseconds + 0.9) / (1000 * 60 * 60);
  const absoluteHours = Math.floor(hours);
  const h = absoluteHours > 9 ? absoluteHours : '0' + absoluteHours;
  const minutes = (hours - absoluteHours) * 60;
  const absoluteMinutes = Math.floor(minutes);
  const m = absoluteMinutes > 9 ? absoluteMinutes : '0' + absoluteMinutes;
  const seconds = (minutes - absoluteMinutes) * 60;
  const absoluteSeconds = Math.floor(seconds);
  const s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
  let ms = Math.floor((seconds - absoluteSeconds) * 1000);
  if (ms < 10)
    ms = '00' + ms;
  else if (ms < 100)
    ms = '0' + ms;
  return h + ':' + m + ':' + s + ':' + ms;
};

export {webots};
