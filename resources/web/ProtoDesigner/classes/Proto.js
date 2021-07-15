'use strict';

import WbSFBool from '../../wwi/nodes/utils/WbSFBool.js';
import WbSFDouble from '../../wwi/nodes/utils/WbSFDouble.js';
import WbSFInt32 from '../../wwi/nodes/utils/WbSFInt32.js';
import WbSFString from '../../wwi/nodes/utils/WbSFString.js';
import WbSFColor from '../../wwi/nodes/utils/WbSFColor.js';
import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';

import ProtoTemplateEngine  from './ProtoTemplateEngine.js';
import ProtoParser from './ProtoParser.js';
import Parameter from './Parameter.js';
import Tokenizer from './Tokenizer.js';
import {VRML_TYPE} from './FieldModel.js';

export default class Proto {
  constructor(protoText) {
    this.isTemplate = protoText.search('template language: javascript') !== -1;
    if(this.isTemplate) {
      console.log('PROTO is a template!');
      this.templateEngine = new ProtoTemplateEngine();
    }

    // raw proto body text must be kept in case the template needs to be regenerated
    const indexBeginBody = protoText.search(/(?<=\]\s*\n*\r*)({)/g);
    this.rawBody = protoText.substring(indexBeginBody);
    
    // head only needs to be parsed once and persists through regenerations
    const indexBeginHead = protoText.search(/(?<=\n|\n\r)(PROTO)(?=\s\w+\s\[)/g); // proto header
    const rawHead = protoText.substring(indexBeginHead, indexBeginBody);

    // parse header and map each entry
    this.id = 0; // id of the parameter
    this.parameters = new Map(); 
    this.parseHead(rawHead);
    
    console.log('Parameters:');
    for (const [key, value] of this.parameters.entries())
      console.log(value);

    parseBody();
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
      const nextToken = headTokenizer.peekToken();
  
      // TODO: skip curly brackets inbetween 'field' keyword and parameter name (i.e restricted nodes)

      if (nextToken.isIdentifier() && token.isKeyword()) {
        const name = nextToken.word();
        const type = token.fieldTypeFromVrml();
        const isRegenerator = this.isTemplate ? isTemplateRegenerator(name) : false;
        
        headTokenizer.nextToken(); // consume current token (i.e the parameter name)
        
        const defaultValue = this.parseParameterValue(type, headTokenizer);
        const value = defaultValue.clone();
        
        const parameter = new Parameter(name, type, isRegenerator, defaultValue, value)
        this.parameters.set(this.uniqueId(), parameter);
      }
    }
  };

  parseParameterValue(type, tokenizer) {
    if (type === VRML_TYPE.SF_BOOL)
      return new WbSFBool(tokenizer.nextToken().toBool());
    else if (type === VRML_TYPE.SF_FLOAT)
      return new WbSFDouble(tokenizer.nextToken().toFloat());
    else if (type === VRML_TYPE.SF_INT32)
      return new WbSFInt32(tokenizer.nextToken().toInt());
    else if (type === VRML_TYPE.SF_STRING)
      return new WbSFString(tokenizer.nextWord());
    else if (type === VRML_TYPE.SF_VECT2F) {
      const x = tokenizer.nextToken().toFloat();
      const y = tokenizer.nextToken().toFloat();
      return new WbVector2(x, y);
    } else if (type === VRML_TYPE.SF_VECT3F) {
      const x = tokenizer.nextToken().toFloat();
      const y = tokenizer.nextToken().toFloat();
      const z = tokenizer.nextToken().toFloat();
      return new WbVector3(x, y, z);
    } else if (type === VRML_TYPE.SF_COLOR) {
      const r = tokenizer.nextToken().toFloat();
      const g = tokenizer.nextToken().toFloat();
      const b = tokenizer.nextToken().toFloat();
      return new WbSFColor(r, g, b);
    } else if (type === VRML_TYPE.SF_ROTATION) {
      const x = tokenizer.nextToken().toFloat();
      const y = tokenizer.nextToken().toFloat();
      const z = tokenizer.nextToken().toFloat();
      const w = tokenizer.nextToken().toFloat();
      return new WbVector4(x, y, z, w);
    } else if (type === VRML_TYPE.SF_NODE)
      console.error('TODO: implement SFNode in parseParameterValue.');
    else
      throw new Error('Unknown type \'' + type + '\' in parseParameterValue.');
  }

  encodeParametersForTemplateEngine() {
    this.encodedParameters = '';
    
    for (const [key, value] of this.parameters.entries())
      // size: {value: {x: 2, y: 1, z: 1}, defaultValue: {x: 2, y: 1, z: 1}}, color: {value: {r: 0, g: 1, b: 1}, defaultValue: {r: 0, g: 1, b: 1}}';
      this.encodedParameters += value.name + ': {value: ' + value.value.jsify() + ', defaultValue: ' + value.defaultValue.jsify() + '}, ';

    this.encodedParameters = this.encodedParameters.slice(0, -2); // remove last comma and space
    
    console.log('Encoded Parameters:\n' + this.encodedParameters);
  }

  parseBody() {
    
  };

  isTemplateRegenerator(parameterName) {
    return this.rawBody.search('fields.' + parameterName + '.') !== -1;
  };

  regenerate() {
    this.bodyTokenizer = new Tokenizer(this.rawBody);
    
    // template engine stuff
    
    // x3d of the body of the proto
    this.x3d = 1;
  };
  
  uniqueId() {
    return this.id++;
  };
};
