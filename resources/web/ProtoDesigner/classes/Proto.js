'use strict';

import {VRML, generateParameterId, generateProtoId} from './utility/utility.js';

import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';

import TemplateEngine  from './TemplateEngine.js';
import ProtoParser from './ProtoParser.js';
import Parameter from './Parameter.js';
import Tokenizer from './Tokenizer.js';

export default class Proto {
  constructor(protoText) {
    this.id = generateProtoId();
    this.isTemplate = protoText.search('template language: javascript') !== -1;
    if(this.isTemplate) {
      console.log('PROTO is a template!');
      this.templateEngine = new TemplateEngine();
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

    console.log('Parameters:');
    for (const [key, value] of this.parameters.entries())
      console.log(value);

    if (this.isTemplate)
      this.regenerate(); // generate VRML compliant proto body
    else {
      this.parseBody();
      this.generateX3d();
    }
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
        // note: header parameter name might be just an alias (ex: size IS myCustomSize), only alias known at this point
        const name = nextToken.word();; // actual name used in the header (i.e value after an IS)
        const type = token.fieldTypeFromVrml();
        const isRegenerator = this.isTemplate ? this.isTemplateRegenerator(name) : false;

        headTokenizer.nextToken(); // consume current token (i.e the parameter name)

        const defaultValue = this.parseParameterValue(type, headTokenizer);
        let value;
        if (defaultValue instanceof WbVector2 || defaultValue instanceof WbVector3 || defaultValue instanceof WbVector4)
          value = defaultValue.clone();
        else if (typeof defaultValue === 'undefined')
          value = undefined;
        else
          value = defaultValue.valueOf();

        const parameter = new Parameter(name, type, isRegenerator, defaultValue, value)
        this.parameters.set(generateParameterId(), parameter);
      }
    }
  };

  parseParameterValue(type, tokenizer) {
    switch (type) {
      case VRML.SFBool:
        return tokenizer.nextToken().toBool();
      case VRML.SFFloat:
        return tokenizer.nextToken().toFloat();
      case VRML.SFInt32:
        return tokenizer.nextToken().toInt();
      case VRML.SFString:
        return tokenizer.nextWord();
      case VRML.SFVec2f:
        const x = tokenizer.nextToken().toFloat();
        const y = tokenizer.nextToken().toFloat();
        return new WbVector2(x, y);
      case VRML.SFVec3f:
      case VRML.SFColor: {
        const x = tokenizer.nextToken().toFloat();
        const y = tokenizer.nextToken().toFloat();
        const z = tokenizer.nextToken().toFloat();
        return new WbVector3(x, y, z);
      }
      case VRML.SFRotation: {
        const x = tokenizer.nextToken().toFloat();
        const y = tokenizer.nextToken().toFloat();
        const z = tokenizer.nextToken().toFloat();
        const w = tokenizer.nextToken().toFloat();
        return new WbVector4(x, y, z, w);
      }
      case VRML.SFNode:
        if(tokenizer.peekWord() === 'NULL') {
          tokenizer.skipToken('NULL');
          return;
        } else
          console.error('TODO: implement handling of pre-provided SFNode in proto header.');
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
    this.bodyTokenizer = new Tokenizer(this.protoBody);
    this.bodyTokenizer.tokenize();
    console.log('Body Tokens:\n', this.bodyTokenizer.tokens());
  };

  generateX3d() {
    // x3d of the body of the proto
    const parser = new ProtoParser(this.bodyTokenizer, this.parameters);
    this.x3d = parser.generateX3d();
  }

  isTemplateRegenerator(parameterName) {
    return this.rawBody.search('fields.' + parameterName + '.') !== -1;
  };

  regenerate() {
    console.log('rawBody:\n' + this.rawBody);

    this.clearReferences(); // remove node references

    this.encodeFieldsForTemplateEngine(); // make current proto parameters in a format compliant to templating rules

    if(typeof this.templateEngine === 'undefined')
      throw new Error('Regeneration was called but the template engine is not defined (i.e this.isTemplate is false)');

    this.protoBody = this.templateEngine.generateVrml(this.encodedFields, this.rawBody);
    console.log('Regenerated Proto Body:\n' + this.protoBody);

    this.parseBody();
    this.generateX3d();
  };

  clearReferences() {
    for (const parameter of this.parameters.values()) {
      parameter.nodeRefs = [];
      parameter.refNames = [];
    }
  };
};
