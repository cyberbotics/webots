import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';
import {getAnId} from '../../wwi/nodes/utils/utils.js';

import Tokenizer from './Tokenizer.js';
import {FieldModel, VRML} from './FieldModel.js';

/*
  Generates an x3d from VRML
*/
export default class ProtoParser {
  constructor(bodyTokenizer, parameters, prefix = '../../wwi/images/post_processing/') {
    this.prefix = prefix;
    this.bodyTokenizer = bodyTokenizer;
    this.parameters = parameters;

    this.defList = new Map();

    // define default scene
    this.xml = document.implementation.createDocument('', '', null);
    this.scene = this.xml.createElement('Scene');

    // TODO: move elsewhere, unnecessary to build background and viewpoint for each proto

    let worldinfo = this.xml.createElement('WorldInfo');
    worldinfo.setAttribute('id', getAnId());
    worldinfo.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/worldinfo');
    worldinfo.setAttribute('basicTimeStep', '32');
    worldinfo.setAttribute('coordinateSystem', 'NUE');

    let viewpoint = this.xml.createElement('Viewpoint');
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

    let background = this.xml.createElement('Background');
    background.setAttribute('id', getAnId());
    background.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/background');
    background.setAttribute('skyColor', '0.15 0.45 1');

    this.scene.appendChild(worldinfo);
    this.scene.appendChild(viewpoint);
    this.scene.appendChild(background);
    this.xml.appendChild(this.scene);
  };

  generateX3d() {
    if (typeof this.bodyTokenizer === 'undefined')
      throw new Error('Cannot generate x3d because body tokenizer was not provided to ProtoParser.');

    if (this.bodyTokenizer.peekWord() === '{')
      this.bodyTokenizer.skipToken('{'); // skip proto body bracket

    console.log('x3d encoding process:');

    while (this.bodyTokenizer.hasMoreTokens()) {
      const token = this.bodyTokenizer.nextToken();
      if (token.isNode())
        this.encodeNodeAsX3d(token.word(), this.scene, 'Scene');
    }

    this.xml = new XMLSerializer().serializeToString(this.xml); // store the raw x3d data only
    console.log('Generated x3d:\n', this.xml);
    return this.xml;
  };

  encodeNodeAsX3d(nodeName, parentElement, parentName, alias) {
    let nodeElement = this.xml.createElement(nodeName);
    console.log('> ' + nodeName + 'Element = xml.createElement(' + nodeName + ')' + ' [parentName=' + parentName + ']');

    this.bodyTokenizer.skipToken('{'); // skip opening bracket following node token

    let ctr = 1; // bracket counter
    while (ctr !== 0) {
      const word = this.bodyTokenizer.nextWord();
      if (word === '{' || word === '}') {
        ctr = word === '{' ? ++ctr : --ctr;
        continue;
      }

      // each node needs an id, at the very least, no matter what follows
      if (nodeElement.getAttribute('id') === null) { // set the id only once per node
        nodeElement.setAttribute('id', getAnId());
        console.log('> ' + nodeName + 'Element.setAttribute(\'id\', \'' + nodeElement.getAttribute('id') + '\')');
        if (typeof alias !== 'undefined') {
          if (this.defList.has(alias))
            throw new Error('DEF nodes must be unique.');

          console.log('>> saving node \'' + nodeName + '\' (id = ' + nodeElement.getAttribute('id') + ') as DEF ' + alias + '.');
          this.defList.set(alias, {id: nodeElement.getAttribute('id'), typeName: nodeName});
        }
      }

      if (this.bodyTokenizer.peekWord() === 'IS')
        this.parseIS(nodeName, word, nodeElement);
      else if (this.bodyTokenizer.peekWord() === 'DEF')
        this.parseDEF(nodeName, word, nodeElement, parentElement, parentName);
      else if (this.bodyTokenizer.peekWord() === 'USE')
        this.parseUSE(nodeElement, nodeName);
      else if (this.bodyTokenizer.peekWord() === '[')
        this.parseMF(nodeElement, nodeName); // the parent of MF fields is the node currently being created
      else // otherwise, assume it's a normal field
        this.encodeFieldAsX3d(nodeName, word, nodeElement);
    };

    parentElement.appendChild(nodeElement);
    console.log('> ' + parentName + 'Element.appendChild(' + nodeName + 'Element)');
  };

  encodeFieldAsX3d(nodeName, fieldName, nodeElement, alias) {
    const fieldType = FieldModel[nodeName]['supported'][fieldName];
    if (typeof fieldType === 'undefined') {
      const fieldType = FieldModel[nodeName]['unsupported'][fieldName]; // check if it's one of the unsupported ones instead
      if (typeof fieldType !== 'undefined') {
        this.bodyTokenizer.consumeTokensByType(fieldType);
        return;
      } else
        throw new Error('Cannot encode field \'' + fieldName + '\' as x3d because it is not part of the FieldModel of node \'' + nodeName + '\'.');
    }

    console.log('>> field \'' + fieldName + '\' is of type \'' + fieldType + '\'');

    if (typeof nodeElement === 'undefined')
      throw new Error('Cannot assign field to node because \'nodeElement\' is not defined.');

    // TODO: add  box.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/box');

    let value = '';
    if (fieldType === VRML.SFBool)
      value += this.bodyTokenizer.nextWord() === 'TRUE' ? 'true' : 'false';
    else if (fieldType === VRML.SFFloat)
      value += this.bodyTokenizer.nextWord();
    else if (fieldType === VRML.SFInt32)
      value += this.bodyTokenizer.nextWord();
    else if (fieldType === VRML.SFVec2f) {
      value += this.bodyTokenizer.nextWord() + ' ';
      value += this.bodyTokenizer.nextWord();
    } else if (fieldType === VRML.SFVec3f) {
      value += this.bodyTokenizer.nextWord() + ' ';
      value += this.bodyTokenizer.nextWord() + ' ';
      value += this.bodyTokenizer.nextWord();
    } else if (fieldType === VRML.SFColor) {
      value += this.bodyTokenizer.nextWord() + ' ';
      value += this.bodyTokenizer.nextWord() + ' ';
      value += this.bodyTokenizer.nextWord();
    } else if (fieldType === VRML.SFRotation) {
      value += this.bodyTokenizer.nextWord() + ' ';
      value += this.bodyTokenizer.nextWord() + ' ';
      value += this.bodyTokenizer.nextWord() + ' ';
      value += this.bodyTokenizer.nextWord();
    } else if (fieldType === VRML.SFNode)
      this.encodeNodeAsX3d(this.bodyTokenizer.nextWord(), nodeElement, nodeName, alias);
    else
      throw new Error('Could not encode field \'' + fieldName + '\' (type: ' + fieldType + ') as x3d. Type not handled.');

    if (fieldType !== VRML.SFNode) {
      nodeElement.setAttribute(fieldName, value);
      console.log('> ' + nodeName + 'Element.setAttribute(\'' + fieldName + '\', \'' + value + '\')');
    }
  };

  parseIS(nodeName, fieldName, nodeElement) {
    const refName = this.bodyTokenizer.recallWord(); // get word before the IS token
    this.bodyTokenizer.skipToken('IS'); // consume IS token
    const alias = this.bodyTokenizer.nextWord(); // actual proto parameter

    // ensure it is a proto parameter
    const parameter = this.getParameterByName(alias);

    if (typeof parameter === 'undefined')
      throw new Error('Cannot parse IS keyword because \'' + alias + '\' is not a known parameter.');

    if (parameter.type === VRML.SFNode && typeof parameter.value !== 'undefined')
      throw new Error('TODO: parseIS for SFNode not yet implemented');

    const value = parameter.x3dify();
    if (parameter.type === VRML.SFNode && typeof value !== 'undefined')
      console.error('Case of SFNodes defined in the header not handled yet.');

    nodeElement.setAttribute(fieldName, value);
    console.log('> ' + nodeName + 'Element.setAttribute(\'' + fieldName + '\', \'' + value + '\')');

    // make the header parameter point to this field's parent
    if (!parameter.nodeRefs.includes(nodeElement.getAttribute('id'))) {
      parameter.nodeRefs.push(nodeElement.getAttribute('id'));
      parameter.refNames.push(refName);
      console.log('>> added nodeRef ' + nodeElement.getAttribute('id') + ' to parameter \'' + refName + '\' (' + alias + ').');
    }
  };

  parseDEF(nodeName, fieldName, nodeElement, parentElement, parentName) {
    this.bodyTokenizer.skipToken('DEF');
    const alias = this.bodyTokenizer.nextWord(); // consume the def alias

    // parse the field as if it were a normal one
    this.encodeFieldAsX3d(nodeName, fieldName, nodeElement, alias); // the parameters are passed through untouched

    parentElement.appendChild(nodeElement);
    console.log('> ' + parentName + 'Element.appendChild(' + nodeName + 'Element)');
  };

  parseUSE(parentElement, parentName) {
    this.bodyTokenizer.skipToken('USE');
    const alias = this.bodyTokenizer.nextWord(); // consume the reference name

    if (!this.defList.has(alias))
      throw new Error('USE node defined before DEF node.');

    // create a minimal x3d clone of the DEF node
    const aliasInfo = this.defList.get(alias);
    let useNode = this.xml.createElement(aliasInfo.typeName);
    useNode.setAttribute('USE', aliasInfo.id); // id of the DEF node

    useNode.setAttribute('id', getAnId()); // id of this node
    console.log('> ' + aliasInfo.typeName + 'Element.setAttribute(\'id\', \'' + useNode.getAttribute('id') + '\')');

    parentElement.appendChild(useNode);
    console.log('> ' + parentName + 'Element.appendChild(' + aliasInfo.typeName + 'Element)');
  };

  parseMF(parentElement, parentName) {
    const field = this.bodyTokenizer.recallWord(); // get field that triggered the parseMF

    const fieldType = FieldModel[parentName]['supported'][field];
    if (typeof fieldType === 'undefined') {
      const fieldType = FieldModel[parentName]['unsupported'][field]; // check if it's one of the unsupported ones instead
      if (typeof fieldType !== 'undefined') {
        this.bodyTokenizer.consumeTokensByType(fieldType);
        return;
      } else
        throw new Error('Cannot encode field \'' + field + '\' as x3d because it is not part of the FieldModel of node \'' + parentName + '\'.');
    }

    console.log('>> field \'' + field + '\' is of type \'' + fieldType + '\'');
    this.bodyTokenizer.skipToken('['); // consume '[' token. Must be done after asserting if the field is supported

    while (this.bodyTokenizer.peekWord() !== ']') { // for nested MF nodes, each consecutive parseMF will consume a pair of '[' and ']'
      switch (fieldType) {
        case VRML.MFNode:
          const childNodeName = this.bodyTokenizer.nextWord();
          this.encodeNodeAsX3d(childNodeName, parentElement, parentName);
          break;
        default:
          throw new Error('Cannot parse MF because field \'' + field + '\' (fieldType: ' + fieldType + ') is not supported.');
      }
    }

    this.bodyTokenizer.skipToken(']'); // consume closing ']' token
  };

  getParameterByName(parameterName) {
    for (const value of this.parameters.values()) {
      if (value.name === parameterName)
        return value;
    }
  };

  encodeProtoManual(rawProto) {
    /*
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
    */
  };
}
