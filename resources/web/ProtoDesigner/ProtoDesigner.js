import {webots} from '../wwi/webots.js';
import ProtoParser from '../wwi/ProtoParser.js';
import WbTokenizer from '../wwi/WbTokenizer.js';

class ProtoDesigner {
  constructor() {
    console.log('constructor ProtoDesigner');

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
    console.log('_init done');

    const url = '../wwi/Protos/ProtoBox.proto';
    this.loadProto(url);
  }

  loadProto(url) {
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async() => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
        await this.loadScene(xmlhttp.responseText);
    };
    xmlhttp.send();
  };

  loadScene(protoContent) {
    console.log('loading scene');

    // create x3d out of tokens
    const parser = new ProtoParser();
    const x3d = parser.encodeProto(protoContent);

    const view = new webots.View(document.getElementById('view3d'));
    view.open(x3d, 'x3d', '', true);
  };
};

let designer = new ProtoDesigner( // eslint-disable-line no-new
);

//designer.loadScene();
