'use strict';

import {webots} from '../wwi/webots.js';
import WbWorld from '../wwi/nodes/WbWorld.js';
import WrenRenderer from '../wwi/WrenRenderer.js';

import Proto from './classes/Proto.js';
import ProtoParametersView from './view/ProtoParametersView.js'; // TODO: replace by makefile?

/*

console.log(WbWorld.instance.nodes);
const n = WbWorld.instance.nodes.get('n-6');
n.size.x = 3;
WbWorld.instance.nodes.get('n-6').updateSize();
renderer.render();

*/

class ProtoDesigner {
  constructor() {
    console.log('Constructor ProtoDesigner');

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

    this._init();
  };

  async _init() {
    let promises = [];
    promises.push(this._load('https://git.io/glm-js.min.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/enum.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/wrenjs.js'));

    await Promise.all(promises);

    WbWorld.init();
    this.renderer = new WrenRenderer();

    const url = '../wwi/Protos/ProtoTest.proto';
    // const url = '../wwi/Protos/ProtoBox.proto';
    // const url = '../wwi/Protos/ProtoSphere.proto';
    // const url = '../wwi/Protos/ProtoTemplate.proto';

    console.log('Loading PROTO: ' + url);
    this.loadProto(url);
  };

  _load(scriptUrl) {
    return new Promise(function(resolve, reject) {
      let script = document.createElement('script');
      script.onload = resolve;
      script.src = scriptUrl;
      document.head.appendChild(script);
    });
  };

  loadProto(url) {
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async() => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
        await this.generateX3d(xmlhttp.responseText);
    };
    xmlhttp.send();
  };

  generateX3d(protoText) {
    this.proto = new Proto(protoText);
  };

  async _initOld() {
    /*
    let promises = [];
    promises.push(this._load('https://git.io/glm-js.min.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/enum.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/wrenjs.js'));

    await Promise.all(promises);

    WbWorld.init();
    this.renderer = new WrenRenderer();
    this._protoParameters = new ProtoParametersView(this._protoParametersElement, this.renderer);
    console.log('_init done');

    // const url = '../wwi/Protos/ProtoTest.proto';
    // const url = '../wwi/Protos/ProtoBox.proto';
    // const url = '../wwi/Protos/ProtoSphere.proto';
    const url = '../wwi/Protos/ProtoTemplate.proto';

    console.log('Loading PROTO: ' + url);
    this.loadProto(url);
    */
  };

  loadSceneOld(protoContent) {
    /*
    console.log('loading scene');
    const parser = new ProtoParser();
    let rawProto;
    // check if template
    if(protoContent.search('# template language: javascript') !== -1) {
      console.log('PROTO is a template!');

      const indexBeginHeader = protoContent.search(/(?<=\n|\n\r)(PROTO)(?=\s\w+\s\[)/g);
      const indexBeginBody = protoContent.search(/(?<=\]\s*\n*\r*)({)/g);
      const protoHeader = protoContent.substring(indexBeginHeader, indexBeginBody);
      const protoModel = parser.extractParameters(protoHeader);

      const protoBody = protoContent.substring(indexBeginBody);

      // evaluate template
      const templateEngine = new WbProtoTemplateEngine();
      const fields = templateEngine.encodeFields(protoModel.parameters);
      const body = templateEngine.encodeBody(protoBody);
      console.log(fields);
      // rawProto = templateEngine.evaluateTemplate('../../javascript/jsTemplate.js', fields, body);

      const template = templateEngine.minimalTemplate();
      const result = templateEngine.fillTemplate(template, fields, body);

      return;

    } else {
      console.log('PROTO is NOT a template!');
      rawProto = protoContent;
    }

    const indexBeginHeader = rawProto.search(/(?<=\n|\n\r)(PROTO)(?=\s\w+\s\[)/g);
    const indexBeginBody = rawProto.search(/(?<=\]\s*\n*\r*)({)/g);
    const protoHeader = rawProto.substring(indexBeginHeader, indexBeginBody);
    console.log('Header: \n', protoHeader);
    const protoModel = parser.extractParameters(protoHeader);
    console.log('ProtoModel: \n', protoModel);
    this._protoParameters.showParameters(protoModel);

    const protoBody = rawProto.substring(indexBeginBody);

    // create x3d out of tokens
    const x3d = parser.encodeProtoBody(protoBody);
    // const x3d = parser.encodeProtoManual(rawProto);

    const view = new webots.View(document.getElementById('view3d'));
    view.open(x3d, 'x3d', '', true, this.renderer);
    */
  };
};

let designer = new ProtoDesigner( // eslint-disable-line no-new
);

//designer.loadScene();
