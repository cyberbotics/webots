'use strict';

import {webots} from '../wwi/webots.js';
import WbWorld from '../wwi/nodes/WbWorld.js';
import WrenRenderer from '../wwi/WrenRenderer.js';

import Proto from './classes/Proto.js';
import EditorView from './view/EditorView.js'; // TODO: replace by makefile?

import {getAnId} from '../wwi/nodes/utils/utils.js';

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

        this.protos = new Map();

        // load default scene
        this.loadMinimalScene();

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

        if (typeof this.scene === 'undefined')
          throw new Error('Scene not ready yet');

        this.loadProto(url);
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

  addProtoToScene(rawProto, parentId) {
    console.log('Raw Proto:\n' + rawProto);
    const newProto = new Proto(rawProto);
    this.protos.set(newProto.name, newProto);

    if (typeof parentId === 'undefined')
      this.editor.updateParameters(newProto);

    console.log(newProto.x3d);
    this.view.x3dScene._loadObject(newProto.x3d, parentId);
  }

  loadProto(url, parentId) {
    console.log('Requesting proto: ' + url);
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', url, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async() => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
        await this.addProtoToScene(xmlhttp.responseText, parentId);
    };
    xmlhttp.send();
  };

  async loadMinimalScene() {
    const xml = document.implementation.createDocument('', '', null);
    this.scene = xml.createElement('Scene');

    let worldinfo = xml.createElement('WorldInfo');
    worldinfo.setAttribute('id', getAnId());
    worldinfo.setAttribute('basicTimeStep', '32');
    worldinfo.setAttribute('coordinateSystem', 'NUE');

    let viewpoint = xml.createElement('Viewpoint');
    viewpoint.setAttribute('id', getAnId());
    viewpoint.setAttribute('orientation', '-0.84816706 -0.5241698 -0.07654181 0.34098753');
    viewpoint.setAttribute('position', '-1.2506319 2.288824 7.564137');
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
};

let designer = new ProtoDesigner( // eslint-disable-line no-new
);
