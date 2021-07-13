'use strict';

import {webots} from '../wwi/webots.js';
import ProtoParser from '../wwi/ProtoParser.js';
import WbTokenizer from '../wwi/WbTokenizer.js';

import ProtoParametersView from './view/ProtoParametersView.js'; // TODO: replace by makefile?

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

    this._protoParametersElement = document.getElementById('proto-parameters');
    if (typeof this._protoParametersElement === 'undefined') {
      console.error('The Proto Designer cannot find the proto-parameters component.');
      return;
    }

    this._protoParameters = new ProtoParametersView(this._protoParametersElement);

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

    const indexBeginHeader = protoContent.search(/(?<=\n|\n\r)(PROTO)(?=\s\w+\s\[)/g);
    const indexBeginBody = protoContent.search(/(?<=\]\s*\n*\r*)({)/g);

    const protoHeader = protoContent.substring(indexBeginHeader, indexBeginBody - 1);
    const protoBody = protoContent.substring(indexBeginBody);

    // create x3d out of tokens
    const parser = new ProtoParser();
    const x3d = parser.encodeProto(protoBody);
    // const x3d = parser.encodeProtoManual(protoContent);

    const view = new webots.View(document.getElementById('view3d'));
    view.open(x3d, 'x3d', '', true);
  };
};

let designer = new ProtoDesigner( // eslint-disable-line no-new
);

//designer.loadScene();
