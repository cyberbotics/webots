'use strict';

import {webots} from '../wwi/webots.js';
import WbWorld from '../wwi/nodes/WbWorld.js';
import WrenRenderer from '../wwi/WrenRenderer.js';

import Proto from './classes/Proto.js';
import EditorView from './view/EditorView.js'; // TODO: replace by makefile?

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

    this._init();
  };

  async _init() {
    Module.onRuntimeInitialized = () => {
      Promise.all(promises).then(() => {
        WbWorld.init();

        this.renderer = new WrenRenderer();
        this.view = new webots.View(document.getElementById('view3d'));
        this.editor = new EditorView(this.editorElement, this.renderer, this.view, this);

        // const url = '../wwi/Protos/ProtoTestParameters.proto';
        // const url = '../wwi/Protos/ProtoBox.proto';
        // const url = '../wwi/Protos/ProtoTemplate.proto';
        // const url = '../wwi/Protos/ProtoTransform.proto';
        // const url = '../wwi/Protos/ProtoTestAll.proto';
        // const url = '../wwi/Protos/ProtoDefUse.proto';

        // base geometries
        // const url = '../wwi/Protos/ProtoTestBox.proto';
        // const url = '../wwi/Protos/ProtoTestCylinder.proto';
        // const url = '../wwi/Protos/ProtoTestSphere.proto';
        // const url = '../wwi/Protos/ProtoTestCapsule.proto';
        // const url = '../wwi/Protos/ProtoTestCone.proto';

        const url = '../wwi/Protos/ProtoTestSFNode.proto';

        console.log('Loading PROTO: ' + url);
        this.loadProto(url, this.loadScene.bind(this));
      });
    };

    let promises = [];
    promises.push(this._load('https://git.io/glm-js.min.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/enum.js'));
    promises.push(this._load('https://cyberbotics.com/wwi/R2021b/wrenjs.js'));
  };

  _load(scriptUrl) {
    return new Promise(function(resolve, reject) {
      let script = document.createElement('script');
      script.onload = resolve;
      script.src = scriptUrl;
      document.head.appendChild(script);
    });
  };

  addProto(url) {
    console.log('Adding new node');
    const parentId = -4;
    this.loadProto('../wwi/Protos/ProtoTestBox.proto', this.addProtoToScene.bind(this), parentId);
  }

  addProtoToScene(rawProto, parentId) {
    const newProto = new Proto(rawProto);
    console.log(newProto.x3d);

    //const x3d = '<nodes><Box id="n-5" size="1 1 1"></Box></nodes>';

    this.view.x3dScene._loadObject(newProto.x3d, parentId);
  }

  loadProto(url, onReady, parentId) {
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async() => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
        await onReady(xmlhttp.responseText, parentId);
    };
    xmlhttp.send();
  };

  loadScene(proto) {
    if (typeof this.proto === 'undefined') {
      this.proto = new Proto(proto);
      this.editor.showParameters(this.proto);
    }

    this.view.open(this.proto.x3d, 'x3d', '', true, this.renderer);
  }
};

let designer = new ProtoDesigner( // eslint-disable-line no-new
);

//designer.loadScene();
