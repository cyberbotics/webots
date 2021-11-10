import {webots} from './webots.js';

const template = document.createElement('template');

template.innerHTML = `
<link type="text/css" href="https://cyberbotics.com/wwi/R2022a/css/animation.css" rel="stylesheet"/>
`;

export default class WebotsAnimation extends HTMLElement {
  constructor() {
    super();
    this._hasActiveAnimation = false;
    this.appendChild(template.content.cloneNode(true));

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
    const x3d = this.getAttribute('x3d');
    const json = this.getAttribute('json');

    if (x3d)
      this.setX3d(x3d);
    if (json)
      this.setJson(json);

    this._init();
  }

  _load(scriptUrl) {
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
          this.load(!(this.getAttribute('autoplay') && this.getAttribute('autoplay') === "false"));
      });
    };
    promises.push(this._load('https://git.io/glm-js.min.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2022a/enum.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2022a/wrenjs.js'));
  }

  setJson(fileName) {
    this._json = fileName;
  }
  setX3d(fileName) {
    this._x3d = fileName;
  }

  load(play, mobileDevice) {
    if (typeof this._x3d === 'undefined') {
	    console.error("No x3d file defined");
	    return;
    }
    if (typeof this._view === 'undefined')
      this._view = new webots.View(this, mobileDevice);
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

  active() {
    return this._hasActiveAnimation;
  }
}

window.customElements.define('webots-animation', WebotsAnimation);
