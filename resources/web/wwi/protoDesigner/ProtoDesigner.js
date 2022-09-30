'use strict';

import {webots} from '../wwi/webots.js';
import WbWorld from '../wwi/nodes/WbWorld.js';
import WrenRenderer from '../wwi/WrenRenderer.js';

import Proto from './classes/Proto.js';
import AssetLibrary from './classes/AssetLibrary.js';

import EditorView from './view/EditorView.js';
import HeaderView from './view/HeaderView.js';
import LibraryView from './view/LibraryView.js';

import {getAnId} from '../wwi/nodes/utils/id_provider.js';

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

    this.editorElement = document.getElementById('proto-parameters');
    if (typeof this.editorElement === 'undefined') {
      console.error('The Proto Designer cannot find the proto-parameters component.');
      return;
    }

    this.headerElement = document.getElementById('header-menu');
    if (typeof this.editorElement === 'undefined') {
      console.error('The Proto Designer cannot find the header-menu component.');
      return;
    }

    this._init();
  }

  async _init() {
    Module.onRuntimeInitialized = () => {
      Promise.all(promises).then(() => {
        WbWorld.init();

        this.renderer = new WrenRenderer();
        this.view = new webots.View(document.getElementById('view3d'));

        this.header = new HeaderView(this.headerElement, this);
        this.assetLibrary = new AssetLibrary();
        this.libraryView = new LibraryView(this.libraryElement, this.assetLibrary);
        this.editor = new EditorView(this.editorElement, this.view, this, this.libraryView);
        // this.assetLibrary.addObserver('loaded', () => { this.libraryView.loadAssets(); });

        //  this.library = new LibraryView(this.libraryElement);
        //  this.loadLibrary('./library/library.json');

        this.baseRobot = undefined; // robot base
        this.protoMap = new Map();
        // load default scene
        this.loadMinimalScene(); // setup background, viewpoint and worldinfo
      });
    };

    let promises = [];
    promises.push(this._load('https://git.io/glm-js.min.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/enum.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/wrenjs.js'));
  }

  _load(scriptUrl) {
    return new Promise(function(resolve, reject) {
      let script = document.createElement('script');
      script.onload = resolve;
      script.src = scriptUrl;
      document.head.appendChild(script);
    });
  }

  insertProto(rawProto, parentId, parameter) {
    console.log('Raw Proto:\n' + rawProto);
    const newProto = new Proto(rawProto); // note: only the header is parsed in the constructor
    newProto.parseBody();

    if (typeof parameter !== 'undefined')
      parameter.value = newProto;

    if (typeof this.baseRobot === 'undefined')
      this.baseRobot = newProto;

    this.protoMap.set(newProto.id, newProto);

    this.editor.refreshParameters();

    this.view.x3dScene._loadObject(newProto.x3d, parentId);
  }

  loadProto(url, parentId, parameter) {
    console.log('Requesting proto: ' + url);
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async() => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
        await this.insertProto(xmlhttp.responseText, parentId, parameter);
    };
    xmlhttp.send();
  }

  loadLibrary(url) {
    console.log('Loading library file: ' + url);
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('json');
    xmlhttp.onreadystatechange = async() => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
        await this.library.parseLibrary(xmlhttp.responseText);
    };
    xmlhttp.send();
  }

  async loadMinimalScene() {
    const xml = document.implementation.createDocument('', '', null);
    this.scene = xml.createElement('Scene');

    let worldinfo = xml.createElement('WorldInfo');
    worldinfo.setAttribute('id', getAnId());
    worldinfo.setAttribute('basicTimeStep', '32');
    worldinfo.setAttribute('coordinateSystem', 'NUE');

    let viewpoint = xml.createElement('Viewpoint');
    viewpoint.setAttribute('id', getAnId());
    viewpoint.setAttribute('position', '0 0 0.7686199');
    viewpoint.setAttribute('exposure', '1');
    viewpoint.setAttribute('bloomThreshold', '21');
    viewpoint.setAttribute('zNear', '0.05');
    viewpoint.setAttribute('zFar', '0');
    viewpoint.setAttribute('followSmoothness', '0.5');
    viewpoint.setAttribute('ambientOcclusionRadius', '2');

    let background = xml.createElement('Background');
    background.setAttribute('id', getAnId());
    background.setAttribute('skyColor', '0.15 0.45 1');

    this.scene.appendChild(worldinfo);
    this.scene.appendChild(viewpoint);
    this.scene.appendChild(background);
    xml.appendChild(this.scene);

    const x3d = new XMLSerializer().serializeToString(xml);
    this.view.open(x3d, 'x3d', '', true, this.renderer);
  }

  cleanup() {
    this.view.x3dScene._deleteObject(parseInt(this.baseRobot.x3dNodes[0].slice(1)));
    this.view.x3dScene.render();
    this.baseRobot = undefined;
    this.protoMap.clear();

    this.editor.refreshParameters();
  }

  exportProto() {
    let s = '';
    s += '#VRML_SIM R2021b utf8\n\n';
    s += 'PROTO MyTinkerbotsBase [\n';
    s += this.exportProtoHeader(this.baseRobot, 2);
    s += ']\n';
    s += '{\n';
    s += '  TinkerbotsBase {\n';
    s += this.exportProtoBody(this.baseRobot, 4);
    s += '  }\n';
    s += '}\n';

    console.log(s);
    return s;
  }

  exportParameters(proto, depth) {
    let p = '';
    const indent = ' '.repeat(depth);

    const parameters = proto.parameters;

    for (const parameter of parameters.values()) {
      if (parameter.value instanceof Proto)
        p += this.exportProtoBody(proto, depth);
      else {
        if (!parameter.isDefaultValue())
          p += indent + parameter.name + ' ' + parameter.vrmlify() + '\n';
      }
    }

    return p;
  }

  exportProtoHeader(proto, depth) {
    let h = '';
    const indent = ' '.repeat(depth);

    const protoParameters = proto.parameters;
    for (const parameter of protoParameters.values()) {
      if (!(parameter.value instanceof Proto))
        h += indent + parameter.exportVrmlHeader();
    }

    return h;
  }

  exportProtoBody(proto, depth) {
    let t = '';
    const indent = ' '.repeat(depth);

    const protoParameters = proto.parameters;
    for (const parameter of protoParameters.values()) {
      if (parameter.value instanceof Proto) {
        t += indent + parameter.name + ' ' + parameter.value.protoName + ' {\n';
        t += this.exportParameters(parameter.value, depth + 2);
        t += indent + '}\n';
      } else
        t += indent + parameter.name + ' IS ' + parameter.name + '\n';
    }

    return t;
  }
};

let designer = new ProtoDesigner( // eslint-disable-line no-new
);
