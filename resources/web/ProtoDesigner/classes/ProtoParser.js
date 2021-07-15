import WbSFBool from '../../wwi/nodes/utils/WbSFBool.js';
import WbSFDouble from '../../wwi/nodes/utils/WbSFDouble.js';
import WbSFInt32 from '../../wwi/nodes/utils/WbSFInt32.js';
import WbSFString from '../../wwi/nodes/utils/WbSFString.js';
import WbSFColor from '../../wwi/nodes/utils/WbSFColor.js';
import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';
import {getAnId} from '../../wwi/nodes/utils/utils.js';

import Tokenizer from './Tokenizer.js';
import {FieldModel, VRML_TYPE} from './FieldModel.js';

/*
  Generates an x3d from VRML
*/
export default class ProtoParser {
  constructor(prefix = '') {
    this._prefix = '../wwi/images/post_processing/';

    this.protoModel = {protoName: undefined, parameters: []};

    // define default scene
    this._xml = document.implementation.createDocument('', '', null);
    this._scene = this._xml.createElement('Scene');

    let worldinfo = this._xml.createElement('WorldInfo');
    worldinfo.setAttribute('id', getAnId());
    worldinfo.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/worldinfo');
    worldinfo.setAttribute('basicTimeStep', '32');
    worldinfo.setAttribute('coordinateSystem', 'NUE');

    let viewpoint = this._xml.createElement('Viewpoint');
    viewpoint.setAttribute('id', getAnId());
    viewpoint.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/viewpoint');
    viewpoint.setAttribute('orientation', '-0.84816706 -0.5241698 -0.07654181 0.34098753');
    viewpoint.setAttribute('position', '-1.2506319 2.288824 7.564137');
    viewpoint.setAttribute('exposure', '1');
    viewpoint.setAttribute('bloomThreshold', '21');
    viewpoint.setAttribute('zNear', '0.05');
    viewpoint.setAttribute('zFar', '0');
    viewpoint.setAttribute('followSmoothness', '0.5');
    viewpoint.setAttribute('ambientOcclusionRadius', '2');

    let background = this._xml.createElement('Background');
    background.setAttribute('id', getAnId());
    background.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/background');
    background.setAttribute('skyColor', '0.15 0.45 1');

    this._scene.appendChild(worldinfo);
    this._scene.appendChild(viewpoint);
    this._scene.appendChild(background);
    this._xml.appendChild(this._scene);
  };

  encodeProtoBody(rawProto) {
    // tokenize proto
    this._tokenizer = new WbTokenizer(rawProto);
    this._tokenizer.tokenize();
    console.log(this._tokenizer.tokens());

    this._tokenizer.skipToken('{'); // skip proto body bracket
    while (this._tokenizer.hasMoreTokens()) {
      const token = this._tokenizer.nextToken();
      if (token.isNode())
        this.encodeNode(token.word(), this._scene, 'Scene');
    }

    console.log(new XMLSerializer().serializeToString(this._xml));
    console.log(this._xml);
    return new XMLSerializer().serializeToString(this._xml); // 'test.x3d';
  };

  encodeNode(nodeName, parentElement, parentName) {
    console.log('Encoding node: ' + nodeName);
    let nodeElement = this._xml.createElement(nodeName);
    console.log('> xml.createElement(' + nodeName + ')');

    //nodeElement.setAttribute('id', getAnId());
    //console.log('> ' + nodeName + '.setAttribute(id, ...)');

    this._tokenizer.skipToken('{'); // skip opening bracket
    let ctr = 1; // bracket counter
    while (ctr !== 0) {
      const word = this._tokenizer.nextWord();
      if (word === '{') {
        ctr++;
        continue;
      }
      if (word === '}') {
        ctr--;
        continue;
      }

      // add minimal attribute, this is needed regardless of what follows
      nodeElement.setAttribute('id', getAnId());

      if (this._tokenizer.peekWord() === 'IS')
        this.parseIS(nodeName, word, nodeElement);
      else if (this._tokenizer.peekWord() === 'DEF')
        this.parseDEF();
      else if (this._tokenizer.peekWord() === 'USE')
        this.parseUSE();
      else // otherwise, assume it's a field
        this.encodeField(nodeName, word, nodeElement);
    };

    parentElement.appendChild(nodeElement);
    console.log('> ' + parentName + '.appendChild(' + nodeName + ')');
  };

  encodeField(nodeName, fieldName, nodeElement) {
    const fieldType = WbFieldModel[nodeName][fieldName];
    console.log('Encoding field ' + fieldName + ' of type: ' + fieldType);

    if (typeof nodeElement === 'undefined')
      throw new Error('\'nodeElement\' is not defined but should be.');

    console.log('> ' + nodeName + '.setAttribute(id, ...)');
    // TODO: add  box.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/box');

    let value = '';
    if (fieldType === VRML_TYPE.SF_BOOL)
      value += this._tokenizer.nextWord() === 'TRUE' ? 'true' : 'false';
    else if (fieldType === VRML_TYPE.SF_FLOAT)
      value += this._tokenizer.nextWord();
    else if (fieldType === VRML_TYPE.SF_INT32)
      value += this._tokenizer.nextWord();
    else if (fieldType === VRML_TYPE.SF_VECT2F) {
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord();
    } else if (fieldType === VRML_TYPE.SF_VECT3F) {
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord();
    } else if (fieldType === VRML_TYPE.SF_COLOR) {
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord();
    } else if (fieldType === VRML_TYPE.SF_ROTATION) {
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord();
    } else if (fieldType === VRML_TYPE.SF_NODE)
      this.encodeNode(this._tokenizer.nextWord(), nodeElement, nodeName);
    else
      throw new Error('unknown fieldName \'' + fieldName + '\' of node ' + nodeName + '(fieldType: ' + fieldType + ')');

    if (fieldType !== VRML_TYPE.SF_NODE) {
      // add field attributes
      console.log('> ' + nodeName + '.setAttribute(' + fieldName + ', ' + value + ')');
      nodeElement.setAttribute(fieldName, value);
    }
  };

  extractParameters(protoHeader) {
    this._headerTokenizer = new WbTokenizer(protoHeader);
    this._headerTokenizer.tokenize();
    const tokens = this._headerTokenizer.tokens();
    console.log('Header: \n', tokens);

    this._headerTokenizer.skipToken('PROTO');
    this.protoModel.protoName = this._headerTokenizer.nextWord();
    let id = 0;

    while (!this._headerTokenizer.peekToken().isEof()) {
      const lastToken = this._headerTokenizer.nextToken();
      const token = this._headerTokenizer.peekToken();

      if (token.isIdentifier() && lastToken.isKeyword()) {
        let parameter = {};
        parameter.id = id++;
        parameter.nodeRef = undefined; // parent node reference
        parameter.name = token.word();
        console.log('parameter name: ' + parameter.name);
        parameter.type = lastToken.fieldTypeFromVrml();
        console.log('parameter type: ' + parameter.type);
        // consume current token (which is the parameter name)
        this._headerTokenizer.nextToken();

        //parameter.value = this.parseParameterValue(parameter.type);
        this.protoModel.parameters.push(parameter);
      }
    }

    return this.protoModel;
  };

  parseIS(nodeName, fieldName, nodeElement) {
    this._tokenizer.skipToken('IS'); // consume IS token
    const alias = this._tokenizer.nextToken(); // actual proto parameter

    // ensure it is a proto parameter
    const ix = this.findProtoParameter(alias.word());
    if (ix === -1)
      throw new Error('Cannot parse IS keyword because parameter \'' + alias.word() + '\' is not in the proto header.');

    if (this.protoModel.parameters[ix].type === VRML_TYPE.SF_NODE)
      throw new Error('TODO: parseIS for SF_NODES not yet implemented');

    nodeElement.setAttribute(fieldName, this.protoModel.parameters[ix].value.asX3d());

    // make the header parameter point to this field's parent
    this.protoModel.parameters[ix].nodeRef = nodeElement.getAttribute('id');
  };

  parseDEF() {
    console.error('TODO: parseDEF not yet implemented');
  };

  parseUSE() {
    console.error('TODO: parseUSE not yet implemented');
  };

  findProtoParameter(parameterName) {
    const parameters = this.protoModel.parameters;
    for (let i = 0; i < parameters.length; ++i) {
      if (parameters[i].name === parameterName)
        return i;
    }
    return -1;
  };

  encodeProtoManual(rawProto) {
    // create xml
    let head = xml.createElement('head');
    let meta = xml.createElement('meta');
    meta.setAttribute('name', 'generator');
    meta.setAttribute('content', 'Webots');

    let scene = xml.createElement('Scene');
    let worldinfo = xml.createElement('WorldInfo');
    worldinfo.setAttribute('id', 'n1');
    worldinfo.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/worldinfo');
    worldinfo.setAttribute('basicTimeStep', '32');
    worldinfo.setAttribute('coordinateSystem', 'NUE');

    let viewpoint = xml.createElement('Viewpoint');
    viewpoint.setAttribute('id', 'n2');
    viewpoint.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/viewpoint');
    viewpoint.setAttribute('orientation', '-0.84816706 -0.5241698 -0.07654181 0.34098753');
    viewpoint.setAttribute('position', '-1.2506319 2.288824 7.564137');
    viewpoint.setAttribute('exposure', '1');
    viewpoint.setAttribute('bloomThreshold', '21');
    viewpoint.setAttribute('zNear', '0.05');
    viewpoint.setAttribute('zFar', '0');
    viewpoint.setAttribute('followSmoothness', '0.5');
    viewpoint.setAttribute('ambientOcclusionRadius', '2');

    let background = xml.createElement('Background');
    background.setAttribute('id', 'n3');
    background.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/background');
    background.setAttribute('skyColor', '0.15 0.45 1');

    let shape = xml.createElement('Shape');
    shape.setAttribute('id', 'n5');

    let box = xml.createElement('Box');
    box.setAttribute('id', 'n6');
    box.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/box');
    box.setAttribute('size', '1 1 1');

    // define structure
    head.appendChild(meta);

    shape.appendChild(box);
    scene.appendChild(worldinfo);
    scene.appendChild(viewpoint);
    scene.appendChild(background);
    scene.appendChild(shape);

    // xml.appendChild(head);
    xml.appendChild(scene);

    console.log(new XMLSerializer().serializeToString(xml));
    console.log(xml);
    return new XMLSerializer().serializeToString(xml); // 'test.x3d';
  };
}
