'use strict';

import {webots} from '../wwi/webots.js';
import WbWorld from '../wwi/nodes/WbWorld.js';
import WrenRenderer from '../wwi/WrenRenderer.js';

import Proto from './classes/Proto.js';
import EditorView from './view/EditorView.js'; // TODO: replace by makefile?

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
        this.editor = new EditorView(this.editorElement, this.renderer, this.view);

        // const url = '../wwi/Protos/ProtoTestParameters.proto';
        // const url = '../wwi/Protos/ProtoBox.proto';
        // const url = '../wwi/Protos/ProtoSphere.proto';
        // const url = '../wwi/Protos/ProtoTemplate.proto';
        // const url = '../wwi/Protos/ProtoTransform.proto';
        //const url = '../wwi/Protos/ProtoTestAll.proto';
        //const url = '../wwi/Protos/ProtoDefUse.proto';

        // base geometries
        const url = '../wwi/Protos/ProtoTestCylinder.proto';

        console.log('Loading PROTO: ' + url);
        this.loadProto(url);
      })
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

  loadScene(proto) {
    if (typeof this.proto === 'undefined') {
      this.proto = new Proto(proto);
      this.editor.showParameters(this.proto);
    }
    let b = `
    <Scene>
    <WorldInfo id='n1' docUrl='https://cyberbotics.com/doc/reference/worldinfo' basicTimeStep='32' coordinateSystem='NUE'></WorldInfo>
    <Viewpoint id='n2' docUrl='https://cyberbotics.com/doc/reference/viewpoint' orientation='-0.6474242 -0.7566491 -0.09123625 0.36827004' position='-0.26666832 0.28241983 0.85613644' exposure='1' bloomThreshold='21' zNear='0.05' zFar='0' followSmoothness='0.5' ambientOcclusionRadius='2'></Viewpoint>
    <Background id='n3' docUrl='https://cyberbotics.com/doc/reference/background' skyColor='0.15 0.45 1' ></Background>
    <Transform id='n4'>
    <Shape id='n6' castShadows='true'>
    <Appearance id='n8'><Material diffuseColor="0.8 0 0" specularColor="1 1 1" shininess="1"/></Appearance>
    <PBRAppearance id='n8' baseColor='0.8 0 0'></PBRAppearance>
    <Box id='n7' size='0.1 0.1 0.1'></Box>
    </Shape>
    <Transform id='n5' translation='0 0.15 0'>
    <Shape id='n9' castShadows='true'>
    <Appearance id='n10'><Material diffuseColor="0.45098 0.823529 0.0862745" specularColor="1 1 1" shininess="1"/></Appearance>
    <PBRAppearance id='n10' baseColor='0.45098 0.823529 0.0862745'></PBRAppearance>
    <Box id='n11' USE='n7'></Box>
    </Shape>
    </Transform>
    </Transform>
    </Scene>
    `

    let a = `
    <Scene>
    <WorldInfo id="n-1" docUrl="https://cyberbotics.com/doc/reference/worldinfo" basicTimeStep="32" coordinateSystem="NUE"/>
    <Viewpoint id="n-2" docUrl="https://cyberbotics.com/doc/reference/viewpoint" orientation="-0.84816706 -0.5241698 -0.07654181 0.34098753" position="-1.2506319 2.288824 7.564137" exposure="1" bloomThreshold="21" zNear="0.05" zFar="0" followSmoothness="0.5" ambientOcclusionRadius="2"/><Background id="n-3" docUrl="https://cyberbotics.com/doc/reference/background" skyColor="0.15 0.45 1"/>
    <Transform id="n-4">
    <Shape id="n-5" castShadows='true'>
    <Appearance id='n8'><Material diffuseColor="0.8 0 0" specularColor="1 1 1" shininess="1"/></Appearance>
    <PBRAppearance id="n-7" baseColor="1 0 0"/>
    <Box id="n-6" size="0.5 0.5 0.5"/>
    </Shape>
    <Transform id="n-8" translation="0 1 0">
    <Shape id="n-9" castShadows='true'>
    <Appearance id='n10'><Material diffuseColor="0.45098 0.823529 0.0862745" specularColor="1 1 1" shininess="1"/></Appearance>
    <PBRAppearance id="n-10" baseColor="0 1 0"/>
    <Box USE="n-6"/>
    </Shape></Transform></Transform></Scene>`;

    this.view.open(this.proto.x3d, 'x3d', '', true, this.renderer);
  }

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
