import {getAnId} from '../../nodes/utils/id_provider.js';

import {FieldModel} from './FieldModel.js';
import {VRML} from './utility/utility.js';

import Proto from './Proto.js';

/*
  Generates an x3d from VRML
*/
export default class ProtoParser {
  constructor(bodyTokenizer, parameters, proto, prefix = '../../wwi/images/post_processing/') {
    this.prefix = prefix;
    this.bodyTokenizer = bodyTokenizer;
    this.parameters = parameters; // TODO: if proto ref is passed, can skip passing parameters
    this.proto = proto;

    this.x3dNodes = []; // keep track of the x3d nodes
    this.nestedProtos = []; // keep track of internal nested protos
    this.defList = new Map();

    this.x3dFragments = new Map();

    this.xml = document.implementation.createDocument('', '', null);
    this.nodes = this.xml.createElement('nodes');
    this.xml.appendChild(this.nodes);
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
        this.encodeNodeAsX3d(token.word(), this.nodes, 'nodes');
    }

    this.xml = new XMLSerializer().serializeToString(this.xml); // store the raw x3d data only

    // if nested protos were present, innest their x3d fragments
    // TODO: find a way that allows to append the XML directly instead of after the fact
    if (this.x3dFragments.size !== 0) {
      for (const [key, fragment] of this.x3dFragments.entries()) {
        console.log('Adding fragment (key = ' + key + '): ' + fragment);
        this.xml = this.xml.replace('<' + key + '/>', fragment);
      }
    }

    console.log('Generated x3d:\n', this.xml);
    return this.xml;
  };

  encodeNodeAsX3d(nodeName, parentElement, parentName, alias) {
    // check if it's a nested Proto
    let nodeElement = this.xml.createElement(nodeName);

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
        this.x3dNodes.push(nodeElement.getAttribute('id'));
        if (typeof alias !== 'undefined') {
          if (this.defList.has(alias))
            throw new Error('DEF nodes must be unique.');

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

    if (parentElement)
      parentElement.appendChild(nodeElement);
  };

  encodeFieldAsX3d(nodeName, fieldName, nodeElement, alias) {
    // determine if the field is a VRML node of if it should be consumed
    const fieldType = FieldModel[nodeName]['supported'][fieldName];
    if (typeof fieldType === 'undefined') {
      const fieldType = FieldModel[nodeName]['unsupported'][fieldName]; // check if it's one of the unsupported ones instead
      if (typeof fieldType !== 'undefined') {
        this.bodyTokenizer.consumeTokensByType(fieldType);
        return;
      } else
        throw new Error('Cannot encode field \'' + fieldName + '\' as x3d because it is not part of the FieldModel of node \'' + nodeName + '\'.');
    }

    if (typeof nodeElement === 'undefined')
      throw new Error('Cannot assign field to node because \'nodeElement\' is not defined.');

    // TODO: add  box.setAttribute('docUrl', 'https://cyberbotics.com/doc/reference/box');
    /*
    let value = '';
    if (fieldType === VRML.SFBool)
      value += this.bodyTokenizer.nextWord() === 'TRUE' ? 'true' : 'false';
    else if (fieldType === VRML.SFString)
      value += this.bodyTokenizer.nextWord();
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
    } else if (fieldType === VRML.SFNode) {
      let imageTextureType;
      if (this.bodyTokenizer.peekWord() === 'ImageTexture')
        imageTextureType = this.bodyTokenizer.recallWord(); // remember the type: baseColorMap, roughnessMap, etc

      this.encodeNodeAsX3d(this.bodyTokenizer.nextWord(), nodeElement, nodeName, alias);
      // exceptions to the rule. TODO: find a better solution (on webots side)
      if (typeof imageTextureType !== 'undefined') {
        const imageTextureElement = nodeElement.lastChild;
        if (imageTextureType === 'baseColorMap')
          imageTextureElement.setAttribute('role', 'baseColor');
        else if (imageTextureType === 'roughnessMap')
          imageTextureElement.setAttribute('role', 'roughness');
        else if (imageTextureType === 'metalnessMap')
          imageTextureElement.setAttribute('role', 'metalness');
        else if (imageTextureType === 'normalMap')
          imageTextureElement.setAttribute('role', 'normal');
        else if (imageTextureType === 'occlusionMap')
          imageTextureElement.setAttribute('role', 'occlusion');
        else if (imageTextureType === 'emissiveColorMap')
          imageTextureElement.setAttribute('role', 'emissiveColor');
        else
          throw new Error('Encountered ImageTexture exception but type \'' + imageTextureType + '\' not handled.')
      }
    } else if (fieldType === VRML.MFString) {
      while (this.bodyTokenizer.peekWord() !== ']')
        value += this.bodyTokenizer.nextWord() + ' ';
      value = value.slice(0, -1);
    } else if (fieldType === VRML.MFVec2f) {
      let ctr = 1;
      while (this.bodyTokenizer.peekWord() !== ']') {
        value += this.bodyTokenizer.nextWord();
        value += (!(ctr % 2) ? ', ' : ' ');
        ctr = ctr > 1 ? 1 : ++ctr;
      }
      value = value.slice(0, -2);
    } else if (fieldType === VRML.MFVec3f) {
      let ctr = 1;
      while (this.bodyTokenizer.peekWord() !== ']') {
        value += this.bodyTokenizer.nextWord();
        value += (!(ctr % 3) ? ', ' : ' ');
        ctr = ctr > 2 ? 1 : ++ctr;
      }
      value = value.slice(0, -2);
    } else if (fieldType === VRML.MFInt32) {
      while (this.bodyTokenizer.peekWord() !== ']')
        value += this.bodyTokenizer.nextWord() + ' ';
      value = value.slice(0, -1);
      console.log(value);
    } else
      throw new Error('Could not encode field \'' + fieldName + '\' (type: ' + fieldType + ') as x3d. Type not handled.');

    */

    if (fieldType === VRML.SFNode) {
      let imageTextureType;
      if (this.bodyTokenizer.peekWord() === 'ImageTexture')
        imageTextureType = this.bodyTokenizer.recallWord(); // remember the type: baseColorMap, roughnessMap, etc

      this.encodeNodeAsX3d(this.bodyTokenizer.nextWord(), nodeElement, nodeName, alias);
      // exceptions to the rule. TODO: find a better solution (on webots side)
      if (typeof imageTextureType !== 'undefined') {
        const imageTextureElement = nodeElement.lastChild;
        if (imageTextureType === 'baseColorMap')
          imageTextureElement.setAttribute('role', 'baseColor');
        else if (imageTextureType === 'roughnessMap')
          imageTextureElement.setAttribute('role', 'roughness');
        else if (imageTextureType === 'metalnessMap')
          imageTextureElement.setAttribute('role', 'metalness');
        else if (imageTextureType === 'normalMap')
          imageTextureElement.setAttribute('role', 'normal');
        else if (imageTextureType === 'occlusionMap')
          imageTextureElement.setAttribute('role', 'occlusion');
        else if (imageTextureType === 'emissiveColorMap')
          imageTextureElement.setAttribute('role', 'emissiveColor');
        else
          throw new Error('Encountered ImageTexture exception but type \'' + imageTextureType + '\' not handled.');
      }
    } else {
      const stringifiedValue = this.stringifyTokenizedValuesByType(fieldType);
      nodeElement.setAttribute(fieldName, stringifiedValue);
    }
  };

  encodeNestedProtoAsX3d(rawProto, protoName, parentElement) {
    console.log('Raw test of nested proto ' + protoName + ':\n', rawProto);

    const nested = new Proto(rawProto); // only parse the header
    // add link to nested proto into main proto
    this.nestedProtos.push(nested);
    // overwrite default nested parameters by consuming parent proto tokens
    this.bodyTokenizer.skipToken('{'); // skip opening bracket
    while (this.bodyTokenizer.peekWord() !== '}') {
      const field = this.bodyTokenizer.nextWord();

      // check if the value is provided directly or by IS reference
      if (this.bodyTokenizer.peekWord() === 'IS') {
        this.bodyTokenizer.skipToken('IS');
        // search parameter in proto header and get current value
        const alias = this.bodyTokenizer.nextWord();
        const parameter = this.proto.getParameterByName(alias); // header parameter, value after the IS
        if (typeof parameter !== 'undefined') {
          nested.setParameterValue(field, parameter.value); // field here refers to the word before the IS token
          // get parameter reference on the nested side
          const targetParameter = nested.getParameterByName(field); // value before the IS (in the parent proto)
          const link = {'origin': parameter, 'target': targetParameter};
          if (!this.proto.aliasLinks.includes(link))
            this.proto.aliasLinks.push(link);
        } else
          throw new Error('Cannot overwrite the value of parameter ' + alias + ' because it is not in the list of parameters.');
      } else { // field value is provided directly (i.e read it from the tokenizer)
        const value = this.stringifyTokenizedValuesByType(fieldType);
        nested.setParameterValueFromString(field, value);
      }
    }
    // as the header is now updated, the body can be parsed
    nested.parseBody();

    const fragmentId = 'fragment' + protoName + Date.now();
    const fragmentElement = this.xml.createElement(fragmentId);
    parentElement.appendChild(fragmentElement);

    const fragment = nested.x3d.replace('<nodes>', '').replace('</nodes>', '');
    this.x3dFragments.set(fragmentId, fragment);
    this.x3dNodes = this.x3dNodes.concat(nested.x3dNodes);
    this.bodyTokenizer.skipToken('}'); // closing bracket
  }

  parseIS(nodeName, fieldName, nodeElement) {
    const refName = this.bodyTokenizer.recallWord(); // get word before the IS token
    this.bodyTokenizer.skipToken('IS'); // consume IS token
    const alias = this.bodyTokenizer.nextWord(); // actual proto parameter

    // ensure it is a proto parameter
    const parameter = this.proto.getParameterByName(alias);

    if (typeof parameter === 'undefined')
      throw new Error('Cannot parse IS keyword because \'' + alias + '\' is not a known parameter.');

    parameter.role = refName;
    const value = parameter.x3dify();
    if (typeof value !== 'undefined') {
      if (parameter.type === VRML.SFNode)
        nodeElement.appendChild(value);
      else
        nodeElement.setAttribute(fieldName, value);
    }

    // make the header parameter point to this field's parent (i.e the node)
    const link = {'origin': parameter, 'target': {'nodeRef': nodeElement.getAttribute('id'), 'fieldName': refName}};
    if (!this.proto.aliasLinks.includes(link))
      this.proto.aliasLinks.push(link);
    /*
    if (!parameter.nodeRefs.includes(nodeElement.getAttribute('id'))) {
      parameter.nodeRefs.push(nodeElement.getAttribute('id'));
      parameter.refNames.push(refName);
    }
    */
  };

  parseDEF(nodeName, fieldName, nodeElement, parentElement, parentName) {
    this.bodyTokenizer.skipToken('DEF');
    const alias = this.bodyTokenizer.nextWord(); // consume the def alias

    // parse the field as if it were a normal one
    this.encodeFieldAsX3d(nodeName, fieldName, nodeElement, alias); // the parameters are passed through untouched

    parentElement.appendChild(nodeElement);
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

    parentElement.appendChild(useNode);
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

    this.bodyTokenizer.skipToken('['); // consume '[' token. Must be done after asserting if the field is supported

    while (this.bodyTokenizer.peekWord() !== ']') { // for nested MF nodes, each consecutive parseMF will consume a pair of '[' and ']'
      switch (fieldType) {
        case VRML.MFNode:
          const childNodeName = this.bodyTokenizer.nextWord();
          this.encodeNodeAsX3d(childNodeName, parentElement, parentName);
          break;
        case VRML.MFString:
        case VRML.MFVec2f:
        case VRML.MFVec3f:
        case VRML.MFFloat:
        case VRML.MFColor:
        case VRML.MFInt32:
          this.encodeFieldAsX3d(parentName, field, parentElement);
          break;
        default:
          throw new Error('Cannot parse MF because field \'' + field + '\' (fieldType: ' + fieldType + ') is not supported.');
      }
    }

    this.bodyTokenizer.skipToken(']'); // consume closing ']' token
  };

  encodeMFFieldAsX3d(parentElement, parentName) {
    console.log(parentName);
  }

  stringifyTokenizedValuesByType(type) {
    let value = '';

    switch (type) {
      case VRML.SFBool:
        value += this.bodyTokenizer.nextWord() === 'TRUE' ? 'true' : 'false';
        break;
      case VRML.SFString:
      case VRML.SFFloat:
      case VRML.SFInt32:
        value += this.bodyTokenizer.nextWord();
        break;
      case VRML.SFVec2f:
        value += this.bodyTokenizer.nextWord() + ' ';
        value += this.bodyTokenizer.nextWord();
        break;
      case VRML.SFVec3f:
      case VRML.SFColor:
        value += this.bodyTokenizer.nextWord() + ' ';
        value += this.bodyTokenizer.nextWord() + ' ';
        value += this.bodyTokenizer.nextWord();
        break;
      case VRML.SFRotation:
        value += this.bodyTokenizer.nextWord() + ' ';
        value += this.bodyTokenizer.nextWord() + ' ';
        value += this.bodyTokenizer.nextWord() + ' ';
        value += this.bodyTokenizer.nextWord();
        break;
      case VRML.MFString:
        if (this.bodyTokenizer.peekWord() !== '[')
          value = this.bodyTokenizer.nextWord(); // field is MFString, but only 1 element is given
        else {
          this.bodyTokenizer.skipToken('[');
          while (this.bodyTokenizer.peekWord() !== ']')
            value += this.bodyTokenizer.nextWord() + ' ';
          value = value.slice(0, -1);
          this.bodyTokenizer.skipToken(']');
        }
        break;
      case VRML.MFFloat:
      case VRML.MFInt32:
        while (this.bodyTokenizer.peekWord() !== ']')
          value += this.bodyTokenizer.nextWord() + ' ';
        value = value.slice(0, -1);
        break;
      case VRML.MFVec2f: {
        let ctr = 1;
        while (this.bodyTokenizer.peekWord() !== ']') {
          value += this.bodyTokenizer.nextWord();
          value += (!(ctr % 2) ? ', ' : ' ');
          ctr = ctr > 1 ? 1 : ++ctr;
        }
        value = value.slice(0, -2);
        break;
      }
      case VRML.MFColor:
      case VRML.MFVec3f: {
        let ctr = 1;
        while (this.bodyTokenizer.peekWord() !== ']') {
          value += this.bodyTokenizer.nextWord();
          value += (!(ctr % 3) ? ', ' : ' ');
          ctr = ctr > 2 ? 1 : ++ctr;
        }
        value = value.slice(0, -2);
        break;
      }
      default:
        throw new Error('Field type \'' + type + '\' is either unsupported or should not be stringified.');
    }

    return value;
  };
}
