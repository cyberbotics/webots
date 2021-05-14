import {webots} from './webots.js';

const template = document.createElement('template');

template.innerHTML = `
<div id="playerDiv" ></div>
`;

export default class WebotsStreaming extends HTMLElement {
  constructor() {
    super();
    document.getElementsByTagName('webots-streaming')[0].appendChild(template.content.cloneNode(true));

    let link = document.createElement('link');
    link.href = 'https://cyberbotics.com/wwi/wrenjs/css/wwi.css';
    link.type = 'text/css';
    link.rel = 'stylesheet';
    document.head.appendChild(link);

    let script = document.createElement('script');
    script.textContent = `var Module = [];
        Module['locateFile'] = function(path, prefix) {

        // if it's a data file, use a custom dir
        if (path.endsWith(".data"))
          return "https://cyberbotics.com/wwi/wrenjs/" + path;

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
    if (typeof this.view === 'undefined')
      this.view = new webots.View(playerDiv, mobileDevice);
    this.view.broadcast = broadcast;
    this.view.setTimeout(-1); // disable timeout that stops the simulation after a given time

    this.disconnectCallback = disconnectCallback;

    this.view.open(ip, mode);
    this.view.onquit = () => this.disconnect();
    this.view.onready = _ => {
      if (typeof callback === 'function')
        callback();
    };
  }

  disconnect() {
    this.view.close();

    let playerDiv = document.getElementsByTagName('webots-streaming')[0];
    playerDiv.innerHTML = null;
    if (this.view.mode === 'mjpeg')
      this.view.multimediaClient = undefined;

    if (typeof this.disconnectCallback === 'function')
      this.disconnectCallback();
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
    promises.push(this._load('https://cyberbotics.com/wwi/wrenjs/enum.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/wrenjs/wrenjs.js'));

    await Promise.all(promises);
  }
}

window.customElements.define('webots-streaming', WebotsStreaming);
