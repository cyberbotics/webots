import {webots} from './webots.js';
import {exitFullscreen} from './fullscreen_handler.js';
const template = document.createElement('template');

template.innerHTML = `
<div id="playerDiv" ></div>
`;

export default class WebotsStreaming extends HTMLElement {
  constructor() {
    super();
    document.getElementsByTagName('webots-streaming')[0].appendChild(template.content.cloneNode(true));

    let link = document.createElement('link');
    link.href = 'https://cyberbotics.com/wwi/R2021b/css/wwi.css';
    link.type = 'text/css';
    link.rel = 'stylesheet';
    document.head.appendChild(link);

    let script = document.createElement('script');
    script.textContent = `var Module = [];
        Module['locateFile'] = function(path, prefix) {

        // if it's a data file, use a custom dir
        if (path.endsWith(".data"))
          return "https://cyberbotics.com/wwi/R2021b/" + path;

        // otherwise, use the default, the prefix (JS file's dir) + the path
        return prefix + path;
      }`;
    document.head.appendChild(script);

    this._init();
  }

  /*
   * ip : ip address of the server
   * mode : x3d or mjpeg
   * broadcast: boolean
   * mobileDevice: boolean
   */
  connect(ip, mode, broadcast, mobileDevice, callback, disconnectCallback) {
    // This `streaming viewer` setups a broadcast streaming where the simulation is shown but it is not possible to control it.
    // For any other use, please refer to the documentation:
    // https://www.cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website
    let playerDiv = document.getElementsByTagName('webots-streaming')[0];
    if (typeof this._view === 'undefined')
      this._view = new webots.View(playerDiv, mobileDevice);
    this._view.broadcast = broadcast;
    this._view.setTimeout(-1); // disable timeout that stops the simulation after a given time

    this._disconnectCallback = disconnectCallback;

    this._view.open(ip, mode);
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

    let playerDiv = document.getElementsByTagName('webots-streaming')[0];
    playerDiv.innerHTML = null;
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

  async _init() {
    let promises = [];
    promises.push(this._load('https://git.io/glm-js.min.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/enum.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/wrenjs.js'));

    await Promise.all(promises);
  }
}

window.customElements.define('webots-streaming', WebotsStreaming);
