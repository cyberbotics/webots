import {M_PI_4} from './nodes/utils/constants.js';
import WbAbstractAppearance from './nodes/WbAbstractAppearance.js';
import WbAppearance from './nodes/WbAppearance.js';
import WbBackground from './nodes/WbBackground.js';
import WbBillboard from './nodes/WbBillboard.js';
import WbBox from './nodes/WbBox.js';
import WbCapsule from './nodes/WbCapsule.js';
import WbCone from './nodes/WbCone.js';
import WbCylinder from './nodes/WbCylinder.js';
import WbDirectionalLight from './nodes/WbDirectionalLight.js';
import WbElevationGrid from './nodes/WbElevationGrid.js';
import WbFog from './nodes/WbFog.js';
import WbGeometry from './nodes/WbGeometry.js';
import WbGroup from './nodes/WbGroup.js';
import WbImage from './nodes/WbImage.js';
import WbImageTexture from './nodes/WbImageTexture.js';
import WbIndexedFaceSet from './nodes/WbIndexedFaceSet.js';
import WbIndexedLineSet from './nodes/WbIndexedLineSet.js';
import WbLight from './nodes/WbLight.js';
import WbMaterial from './nodes/WbMaterial.js';
import WbPBRAppearance from './nodes/WbPBRAppearance.js';
import WbPlane from './nodes/WbPlane.js';
import WbPointLight from './nodes/WbPointLight.js';
import WbPointSet from './nodes/WbPointSet.js';
import WbScene from './nodes/WbScene.js';
import WbShape from './nodes/WbShape.js';
import WbSphere from './nodes/WbSphere.js';
import WbSpotLight from './nodes/WbSpotLight.js';
import WbTextureTransform from './nodes/WbTextureTransform.js';
import WbTransform from './nodes/WbTransform.js';
import WbSFBool from './nodes/utils/WbSFBool.js';
import WbSFDouble from './nodes/utils/WbSFDouble.js';
import WbSFInt32 from './nodes/utils/WbSFInt32.js';
import WbSFColor from './nodes/utils/WbSFColor.js';
import WbVector2 from './nodes/utils/WbVector2.js';
import WbVector3 from './nodes/utils/WbVector3.js';
import WbVector4 from './nodes/utils/WbVector4.js';
import WbViewpoint from './nodes/WbViewpoint.js';
import WbWorld from './nodes/WbWorld.js';
import {getAnId} from './nodes/utils/utils.js';

import DefaultUrl from './DefaultUrl.js';
import loadHdr from './hdr_loader.js';

import WbTokenizer from './WbTokenizer.js';
import {WbFieldModel, FIELD_TYPES} from './WbFieldModel.js';

/*
  This module takes an x3d world, parse it and populate the scene.
*/
export default class ProtoParser {
  constructor(prefix = '') {
    this._prefix = '../wwi/images/post_processing/';
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
      // otherwise, assume it's a field
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

    // add minimal attributes
    nodeElement.setAttribute('id', getAnId());
    console.log('> ' + nodeName + '.setAttribute(id, ...)');
    // TODO: add  box.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/box');

    let value = '';
    if (fieldType === FIELD_TYPES.SF_BOOL)
      value += this._tokenizer.nextWord() === 'TRUE' ? 'true' : 'false';
    else if (fieldType === FIELD_TYPES.SF_FLOAT)
      value += this._tokenizer.nextWord();
    else if (fieldType === FIELD_TYPES.SF_INT32)
      value += this._tokenizer.nextWord();
    else if (fieldType === FIELD_TYPES.SF_VECT2F) {
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord();
    } else if (fieldType === FIELD_TYPES.SF_VECT3F) {
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord();
    } else if (fieldType === FIELD_TYPES.SF_COLOR) {
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord();
    } else if (fieldType === FIELD_TYPES.SF_ROTATION) {
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord() + ' ';
      value += this._tokenizer.nextWord();
    } else if (fieldType === FIELD_TYPES.SF_NODE)
      this.encodeNode(this._tokenizer.nextWord(), nodeElement, nodeName);
    else
      throw new Error('unknown fieldName \'' + fieldName + '\' of node ' + nodeName + '(fieldType: ' + fieldType + ')');

    if (fieldType !== FIELD_TYPES.SF_NODE) {
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

    let protoModel = {protoName: undefined, parameters: []};

    this._headerTokenizer.skipToken('PROTO');
    protoModel.protoName = this._headerTokenizer.nextWord();
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

        parameter.value = this.parseParameterValue(parameter.type);
        protoModel.parameters.push(parameter);
      }
    }

    return protoModel;
  };

  parseParameterValue(parameterType) {
    if (parameterType === FIELD_TYPES.SF_BOOL)
      return new WbSFBool(this._headerTokenizer.nextToken().toBool());
    else if (parameterType === FIELD_TYPES.SF_FLOAT)
      return new WbSFDouble(this._headerTokenizer.nextToken().toFloat());
    else if (parameterType === FIELD_TYPES.SF_INT32)
      return new WbSFInt32(this._headerTokenizer.nextToken().toInt());
    else if (parameterType === FIELD_TYPES.SF_STRING)
      return this._headerTokenizer.nextWord();
    else if (parameterType === FIELD_TYPES.SF_VECT2F) {
      const x = this._headerTokenizer.nextToken().toFloat();
      const y = this._headerTokenizer.nextToken().toFloat();
      return new WbVector2(x, y);
    } else if (parameterType === FIELD_TYPES.SF_VECT3F) {
      const x = this._headerTokenizer.nextToken().toFloat();
      const y = this._headerTokenizer.nextToken().toFloat();
      const z = this._headerTokenizer.nextToken().toFloat();
      return new WbVector3(x, y, z);
    } else if (parameterType === FIELD_TYPES.SF_COLOR) {
      const r = this._headerTokenizer.nextToken().toFloat();
      const g = this._headerTokenizer.nextToken().toFloat();
      const b = this._headerTokenizer.nextToken().toFloat();
      return new WbSFColor(r, g, b);
    } else if (parameterType === FIELD_TYPES.SF_ROTATION) {
      const x = this._headerTokenizer.nextToken().toFloat();
      const y = this._headerTokenizer.nextToken().toFloat();
      const z = this._headerTokenizer.nextToken().toFloat();
      const w = this._headerTokenizer.nextToken().toFloat();
      return new WbVector4(x, y, z, w);
    } else if (parameterType === FIELD_TYPES.SF_NODE)
      console.error('TODO: implement SFNode in parseParameterValue.');
    else
      throw new Error('Unknown ParameterType \'' + parameterType + '\' in parseParameterValue.');
  }

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
