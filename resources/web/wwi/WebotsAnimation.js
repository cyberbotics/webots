import {webots} from './webots.js';

const template = document.createElement('template');

template.innerHTML = `
<link type="text/css" href="https://cyberbotics.com/wwi/R2021c/css/animation.css" rel="stylesheet"/>
`;

export default class WebotsAnimation extends HTMLElement {
  constructor() {
    super();
    this._hasActiveAnimation = false;
    this.appendChild(template.content.cloneNode(true));

    let script = document.createElement('script');
    script.textContent = `var Module = [];
        Module['locateFile'] = function(path, prefix) {

        // if it's a data file, use a custom dir
        if (path.endsWith(".data"))
          return "https://cyberbotics.com/wwi/R2021c/" + path;

        // otherwise, use the default, the prefix (JS file's dir) + the path
        return prefix + path;
      }`;
    document.head.appendChild(script);

    let name = document.getElementsByTagName('webots-animation')[0].title;
    if (!name)
      name = location.pathname.substring(location.pathname.lastIndexOf('/') + 1).replace('.html', '');
    this.setNames(name);

    this._init();
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
    promises.push(this._load('https://cyberbotics.com/wwi/R2021c/enum.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021c/wrenjs.js'));

    await Promise.all(promises);
  }

  setX3dName(name) {
    this._x3d = name + '.x3d';
  }

  setJsonName(name) {
    this._json = name + '.json';
  }

  setNames(name) {
    this.setX3dName(name);
    this.setJsonName(name);
  }

  play(mobileDevice) {
    if (typeof this._view === 'undefined')
      this._view = new webots.View(this, mobileDevice);
    this._view.open(this._x3d);
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
