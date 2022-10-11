'use strict';

import {VRML, generateParameterId, generateProtoId} from './utility/utility.js';

import WbVector2 from '../../nodes/utils/WbVector2.js';
import WbVector3 from '../../nodes/utils/WbVector3.js';
import WbVector4 from '../../nodes/utils/WbVector4.js';

import TemplateEngine  from './TemplateEngine.js';
import ProtoParser from './ProtoParser.js';
import Parameter from './Parameter.js';
import Tokenizer from './Tokenizer.js';
import Token from './Token.js';
import { ProtoModel } from './ProtoModel.js';
import { assert } from './utility/templating/modules/webots/wbutility.js';

export default class Proto {
  constructor(protoText, url) {
    this.id = generateProtoId();
    this.nestedList = []; // list of internal protos
    this.linkedList = []; // list of protos inserted through the Proto header
    this.url = url;
    this.x3dnodes = [];
    this.externProtos = new Map();

    this.aliasLinks = []; // list of IS references, encoded as mappings between parameters and {node, fieldname} pairs

    this.isTemplate = protoText.search('template language: javascript') !== -1;
    if (this.isTemplate) {
      console.log('PROTO is a template!');
      this.templateEngine = new TemplateEngine();
    }

    // get EXTERNPROTO
    const lines = protoText.split('\n');
    for (let i = 0; i < lines.length; i++) {
      let line = lines[i];
      if (line.indexOf('EXTERNPROTO') !== -1) {
        // get only the text after 'USER_LOG' for the single line
        line = line.split('EXTERNPROTO')[1].trim();
        let address = line.replaceAll('"', '');
        let protoName = address.split('/').pop().replace('.proto', '');
        if (address.startsWith('webots://'))
          address = 'https://raw.githubusercontent.com/cyberbotics/webots/R2022b/' + address.substring(9);

        this.externProtos.set(protoName, address);
      }
    }
    console.log('EXTERNPROTO', this.externProtos);

    // change all relative paths to remote ones
    const re = /\"(?:[^\"]*)\.(jpe?g|png|hdr|obj|stl|dae|wav|mp3)\"/g;
    let result;
    while((result = re.exec(protoText)) !== null) {
      console.log(result)
      protoText = protoText.replace(result[0], '\"' + combinePaths(result[0].slice(1, -1), this.url) + '\"');
    }
    // raw proto body text must be kept in case the template needs to be regenerated
    const indexBeginBody = protoText.search(/(?<=\]\s*\n*\r*)({)/g);
    this.rawBody = protoText.substring(indexBeginBody);
    if (!this.isTemplate)
      this.protoBody = this.rawBody; // body already VRML compliant

    // head only needs to be parsed once and persists through regenerations
    const indexBeginHead = protoText.search(/(?<=\n|\n\r)(PROTO)(?=\s\w+\s\[)/g); // proto header
    const rawHead = protoText.substring(indexBeginHead, indexBeginBody);

    // parse header and map each entry
    this.parameters = new Map();
    this.parseHead(rawHead);
  };

  parseHead(rawHead) {
    const headTokenizer = new Tokenizer(rawHead);
    headTokenizer.tokenize();

    const tokens = headTokenizer.tokens();
    console.log('Header Tokens: \n', tokens);

    // build parameter list
    headTokenizer.skipToken('PROTO');
    this.protoName = headTokenizer.nextWord();

    while (!headTokenizer.peekToken().isEof()) {
      const token = headTokenizer.nextToken();
      let nextToken = headTokenizer.peekToken();

      if (token.isKeyword() && nextToken.isPunctuation()) {
        if (nextToken.word() === '{'){
          // field restrictions are not supported yet, consume the tokens
          headTokenizer.consumeTokensByType(VRML.SFNode);
          nextToken = headTokenizer.peekToken(); // update upcoming token reference after consumption
        }
      }

      if (token.isKeyword() && nextToken.isIdentifier()) {
        // note: header parameter name might be just an alias (ex: size IS myCustomSize), only the alias known at this point
        const name = nextToken.word(); // actual name used in the header (i.e value after an IS)
        const type = token.fieldTypeFromVrml();
        console.log('VRML PARAM ' + name, type)
        const isRegenerator = this.isTemplate ? this.isTemplateRegenerator(name) : false;

        headTokenizer.nextToken(); // consume current token (i.e the parameter name)

        const defaultValue = this.encodeParameterAsJavaScript(type, headTokenizer);
        let value = defaultValue.valueOf();

        //if (defaultValue instanceof WbVector2 || defaultValue instanceof WbVector3 || defaultValue instanceof WbVector4)
        //  value = defaultValue.clone();
        //else if (typeof defaultValue === 'undefined')
        //  value = undefined;
        //else {
        //  value = defaultValue.valueOf();
        //}

        const parameterId = generateParameterId();
        const parameter = new Parameter(this, parameterId, name, type, isRegenerator, defaultValue, value)
        this.parameters.set(parameterId, parameter);
      }
    }
  };

  encodeParameterAsJavaScript(type, tokenizer) {
    switch (type) {
      case VRML.SFBool:
        return tokenizer.nextToken().toBool();
      case VRML.SFFloat:
        return tokenizer.nextToken().toFloat();
      case VRML.SFInt32:
        return tokenizer.nextToken().toInt();
      case VRML.SFString:
        return `'tokenizer.nextWord()'`;
      case VRML.SFVec2f:
        const x = tokenizer.nextToken().toFloat();
        const y = tokenizer.nextToken().toFloat();
        return `{x: ${x}, y: ${y}}`
      case VRML.SFVec3f: {
        const x = tokenizer.nextToken().toFloat();
        const y = tokenizer.nextToken().toFloat();
        const z = tokenizer.nextToken().toFloat();
        return `{x: ${x}, y: ${y}, z: ${z}}`
      }
      case VRML.SFColor: {
        const r = tokenizer.nextToken().toFloat();
        const g = tokenizer.nextToken().toFloat();
        const b = tokenizer.nextToken().toFloat();
        return `{r: ${r}, g: ${g}, b: ${b}}`
      }
      case VRML.SFRotation: {
        const x = tokenizer.nextToken().toFloat();
        const y = tokenizer.nextToken().toFloat();
        const z = tokenizer.nextToken().toFloat();
        const a = tokenizer.nextToken().toFloat();
        return `{x: ${x}, y: ${y}, z: ${z}, a: ${a}}`
      }
      case VRML.SFNode:
        console.log('Decoding SFNode parameter')
        if (tokenizer.peekWord() === 'NULL') {
          tokenizer.skipToken('NULL');
          return undefined;
        } else {
          const nodeName = tokenizer.nextWord();
          if (typeof ProtoModel[nodeName] !== 'undefined') {
            const nodeModel = ProtoModel[nodeName];
            tokenizer.skipToken('{');
            let fields = '{'
            while(tokenizer.peekWord() !== '}') {
              const parameterName = tokenizer.nextWord();
              assert(nodeModel['parameters'].has(parameterName));
              const parameter = nodeModel['parameters'][parameterName];
              const type = nodeModel['parameters']['type'];
              const value = encodeParameterAsJavaScript(type, tokenizer)
              fields += '{'
            }
            tokenizer.skipToken('}');
          }
          console.log(nodeText)
          return ;
        }
      case VRML.MFString: {
        console.log('Decoding MFString parameter')
        const vector = [];
        tokenizer.skipToken('[');
        while (tokenizer.peekWord() !== ']')
          vector.push(this.encodeParameterAsJavaScript(VRML.SFString, tokenizer));
        tokenizer.skipToken(']');
        return vector;
      }
      case VRML.MFInt32: {
        console.log('Decoding MFInt32 parameter')
        const vector = [];
        tokenizer.skipToken('[');
        while (tokenizer.peekWord() !== ']')
          vector.push(this.encodeParameterAsJavaScript(VRML.SFInt32, tokenizer));
        tokenizer.skipToken(']');
      }
      case VRML.MFFloat: {
        console.log('Decoding MFFloat parameter')
        const vector = [];
        tokenizer.skipToken('[');
        while (tokenizer.peekWord() !== ']')
          vector.push(this.encodeParameterAsJavaScript(VRML.SFFloat, tokenizer));
        assert(tokenizer.peekWord() === ']');
        tokenizer.skipToken(']');
        return vector;
      }
      case VRML.MFNode:
        console.log('Decoding MFNode parameter')
        const vector = [];
        if (tokenizer.peekWord() === '[') {
          while (tokenizer.peekWord() !== ']')
          vector.push(this.encodeParameterAsJavaScript(VRML.SFNode, tokenizer));
          tokenizer.skipToken(']');
        } else
          vector.push(this.encodeParameterAsJavaScript(VRML.SFNode, tokenizer));

        return vector;
      default:
        throw new Error('Unknown type \'' + type + '\' in parseParameterValue.');
    }
  }

  encodeFieldsForTemplateEngine() {
    this.encodedFields = ''; // ex: 'size: {value: {x: 2, y: 1, z: 1}, defaultValue: {x: 2, y: 1, z: 1}}'

    for (const [key, parameter] of this.parameters.entries())
      this.encodedFields += parameter.name + ': ' + parameter.jsify() + ', ';

    this.encodedFields = this.encodedFields.slice(0, -2); // remove last comma and space

    console.log('Encoded Fields:\n' + this.encodedFields);
  }

  parseBody() {
    this.clearReferences();
    // note: if not a template, the body is already pure VRML
    if (this.isTemplate)
      this.regenerateBodyVrml(); // overwrites this.protoBody with a purely VRML compliant body

    // tokenize body
    this.bodyTokenizer = new Tokenizer(this.protoBody);
    this.bodyTokenizer.tokenize();

    // generate x3d from VRML
    this.generateX3d();
  };

  generateX3d() {
    const parser = new ProtoParser(this.bodyTokenizer, this.parameters, this);
    this.x3d = parser.generateX3d();
    this.x3dNodes = parser.x3dNodes;
    console.log('New x3d nodes: ', this.x3dNodes)
    this.nestedList = this.nestedList.concat(parser.nestedProtos);
    console.log('Nested protos: ', this.nestedList);
  };

  isTemplateRegenerator(parameterName) {
    return this.rawBody.search('fields.' + parameterName + '.') !== -1;
  };

  regenerateBodyVrml() {
    this.encodeFieldsForTemplateEngine(); // make current proto parameters in a format compliant to templating rules

    if(typeof this.templateEngine === 'undefined')
      throw new Error('Regeneration was called but the template engine is not defined (i.e this.isTemplate is false)');

    this.protoBody = this.templateEngine.generateVrml(this.encodedFields, this.rawBody);
    // console.log('Regenerated Proto Body:\n' + this.protoBody);
  };

  setParameterValue(parameterName, value) {
    // ensure parameter exists
    for (const [key, parameter] of this.parameters.entries()) {
      if(parameter.name === parameterName) {
        parameter.value = value;
        console.log('Overwriting value of parameter ' + parameterName + ' with ', value);
        return;
      }
    }

    throw new Error('Cannot set parameter ' + parameterName + ' (value = ', value, ') because it is not a parameter of proto ' + this.protoName);
  }

  setParameterValueFromString(parameterName, value) {
    // ensure parameter exists
    for (const [key, parameter] of this.parameters.entries()) {
      if(parameter.name === parameterName) {
        parameter.setValueFromString(value);
        console.log('Overwriting value of parameter ' + parameterName + ' with ' + value);
        return;
      }
    }

    throw new Error('Cannot set parameter ' + parameterName + ' (value =' + value + ') because it is not a parameter of proto ' + this.protoName);
  }

  getParameterByName(parameterName) {
    for (const value of this.parameters.values()) {
      if (value.name === parameterName)
        return value;
    }
  };

  getTriggeredFields(triggerParameter) {
    const links = triggerParameter.protoRef.aliasLinks;
    let triggered = [];

    for (let i = 0; i < links.length; ++i) {
      if (links[i].origin === triggerParameter) {
        if (links[i].origin instanceof Parameter && links[i].target instanceof Parameter) // nested IS reference
          triggered = triggered.concat(this.getTriggeredFields(links[i].target)); // recursively follow parameter chain
        else if (links[i].origin instanceof Parameter && typeof links[i].target === 'object') { // found chain end
          triggered.push(links[i].target);
        } else
          throw new Error('Cannot retrieve fields triggered by paramter change because chain is malformed.');
      }
    }

    return triggered;
  }

  clearReferences() {
    this.nestedList = [];
    this.linkedList = [];
    this.x3dNodes = [];
    for (const parameter of this.parameters.values()) {
      parameter.nodeRefs = [];
      parameter.refNames = [];
    }
  };
};


function combinePaths(url, parentUrl) {
  if (url.startsWith('http://' || url.startsWith('https://')))
    return url;  // url is already resolved

  const newUrl = new URL(url, parentUrl.slice(0, parentUrl.lastIndexOf('/') + 1)).href
  console.log('FROM >' + url + '< AND >' + parentUrl + "< === " + newUrl);
  return newUrl;
}

export { Proto, combinePaths };
