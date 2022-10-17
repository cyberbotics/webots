'use strict';

import {generateParameterId, generateProtoId} from './utility/utility.js';

import TemplateEngine  from './TemplateEngine.js';
import Tokenizer from './Tokenizer.js';
import { FieldModel } from './FieldModel.js'; // TODO: merge in BaseNode?
import { typeFactory, SFNode, MFNode } from './Vrml.js';
import NodeFactory from './NodeFactory.js';
import { VRML } from './constants.js';

export default class ProtoNode {
  constructor(protoText, protoUrl) {
    // IMPORTANT! When adding new member variables of type Map, modify the .clone method so that it creates a copy of it
    this.id = generateProtoId();
    this.url = protoUrl;
    this.name = this.url.slice(this.url.lastIndexOf('/') + 1).replace('.proto', '')
    console.log('CREATING PROTO ' + this.name)
    this.parameters = new Map();
    this.externProto = new Map();
    this.def = new Map();

    this.xml = document.implementation.createDocument('', '', null)
    // to generate: <Shape castShadows="true"><PBRAppearance baseColor="1 0 0"/></Shape>
    /*
    const xml = document.implementation.createDocument('', '', null)
    const shapeNode = xml.createElement('Shape');
    shapeNode.setAttribute('castShadows', 'true')
    const pbrNode = xml.createElement('PBRAppearance')
    pbrNode.setAttribute('baseColor', '1 0 0');
    shapeNode.appendChild(pbrNode)
    xml.appendChild(shapeNode);
    console.log(new XMLSerializer().serializeToString(xml))
    */

    //clone tester
    /*
    const a = new BaseNode('Box');
    const b = a.clone();
    a.test = [3, 2, 1];
    console.log(a, b)
    throw new Error('stop')
    */

    // the value of a PROTO is its base-type
    this.value = undefined;

    this.isTemplate = protoText.search('template language: javascript') !== -1;
    if (this.isTemplate) {
      console.log('PROTO is a template!');
      this.templateEngine = new TemplateEngine();
    }

    // change all relative paths to remote ones
    const re = /\"(?:[^\"]*)\.(jpe?g|png|hdr|obj|stl|dae|wav|mp3)\"/g;
    let result;
    while((result = re.exec(protoText)) !== null) {
      // console.log(result)
      protoText = protoText.replace(result[0], '\"' + combinePaths(result[0].slice(1, -1), this.url) + '\"');
    }

    // raw proto body text must be kept in case the template needs to be regenerated
    const indexBeginBody = protoText.search(/(?<=\]\s*\n*\r*)({)/g);
    this.rawBody = protoText.substring(indexBeginBody);
    if (!this.isTemplate)
      this.protoBody = this.rawBody; // body already VRML compliant

    // head only needs to be parsed once and persists through regenerations
    // TODO: rename interface/parameters
    const indexBeginHead = protoText.search(/(?<=\n|\n\r)(PROTO)(?=\s\w+\s\[)/g); // proto header
    this.rawHead = protoText.substring(indexBeginHead, indexBeginBody);

    // defines tags and EXTERNPROTO, persists through regenerations
    this.rawHeader = protoText.substring(0, indexBeginHead);

    // get EXTERNPROTO
    this.promises = [];
    const lines = this.rawHeader.split('\n');
    for (let i = 0; i < lines.length; i++) {
      let line = lines[i];
      if (line.indexOf('EXTERNPROTO') !== -1) {
        // get only the text after 'EXTERNPROTO' for the single line
        line = line.split('EXTERNPROTO')[1].trim();
        let address = line.replaceAll('"', '');
        let protoName = address.split('/').pop().replace('.proto', '');
        if (address.startsWith('webots://'))
          address = 'https://raw.githubusercontent.com/cyberbotics/webots/R2022b/' + address.substring(9);
        else
          address = combinePaths(address, this.url)

        this.externProto.set(protoName, address);
        this.promises.push(this.getExternProto(address));
      }
    }
  };

  getExternProto(protoUrl) {
    return new Promise((resolve, reject) => {
      const xmlhttp = new XMLHttpRequest();
      xmlhttp.open('GET', protoUrl, true);
      xmlhttp.overrideMimeType('plain/text');
      xmlhttp.onreadystatechange = async() => {
        if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
          resolve(xmlhttp.responseText);
      };
      xmlhttp.send();
    }).then(text => {
      console.log('downloaded ' + protoUrl + ', generating prototype');
      const nodeFactory = new NodeFactory();
      return nodeFactory.createPrototype(text, protoUrl);
    });
  }

  async fetch() {
    return Promise.all(this.promises).then(async () => {
      // parse header and map each parameter entry
      console.log(this.name + ': all EXTERNPROTO promises have been resolved')
      await this.parseHead();
    });
  }


  clone() {
    let copy = Object.assign(Object.create(Object.getPrototypeOf(this)), this);
    copy.id = generateProtoId();


    copy.parameters = new Map();
    for (const [parameterName, parameterValue] of this.parameters) {
      if (typeof parameterValue !== 'undefined') {
        // console.log('cloning ' + parameterName)
        copy.parameters.set(parameterName, parameterValue.clone());
      }
    }

    copy.def = new Map(this.def);
    copy.externProto = new Map(this.externProto);
    return copy;
  }

  async parseHead() {
    console.log('PARSE HEAD OF ' + this.name)
    const headTokenizer = new Tokenizer(this.rawHead, this);
    headTokenizer.tokenize();

    const tokens = headTokenizer.tokens();
    //console.log('Header Tokens: \n', tokens);

    // build parameter list
    headTokenizer.skipToken('PROTO');
    this.protoName = headTokenizer.nextWord();

    while (!headTokenizer.peekToken().isEof()) {
      const token = headTokenizer.nextToken();
      let nextToken = headTokenizer.peekToken();

      if (token.isKeyword() && nextToken.isPunctuation()) {
        if (nextToken.word() === '{'){
          // TODO: field restrictions are not supported yet, consume the tokens
          headTokenizer.consumeTokensByType(VRML.SFNode);
          nextToken = headTokenizer.peekToken(); // update upcoming token reference after consumption
        }
      }

      if (token.isKeyword() && nextToken.isIdentifier()) {
        const parameterName = nextToken.word(); // actual name used in the header (i.e value after an IS)
        const parameterType = token.fieldTypeFromVrml();
        const isRegenerator = this.isTemplate ? this.isTemplateRegenerator(parameterName) : false;
        headTokenizer.nextToken(); // consume the parameter name token

        console.log('INTERFACE PARAMETER ' + parameterName + ', TYPE: ' + parameterType + ', VALUE:');
        const parameter = typeFactory(parameterType, headTokenizer);
        // parameter.setValueFromTokenizer(headTokenizer);
        this.parameters.set(parameterName, parameter);
      }
    }
  };

  parseBody() {
    console.log('PARSE BODY OF ' + this.name)
    this.clearReferences();
    // note: if not a template, the body is already pure VRML
    if (this.isTemplate)
      this.regenerateBodyVrml(); // overwrites this.protoBody with a purely VRML compliant body

    // tokenize body
    const bodyTokenizer = new Tokenizer(this.protoBody, this);
    bodyTokenizer.tokenize();

    // skip bracket opening the PROTO body
    bodyTokenizer.skipToken('{'); // TODO: move elsewhere or remove from tokenizer

    const baseType = bodyTokenizer.peekWord();
    let protoUrl;
    if (typeof FieldModel[baseType] === 'undefined')
      protoUrl = this.externProto.get(baseType); // it's a derived PROTO

    const nodeFactory = new NodeFactory();
    this.value = nodeFactory.createNode(bodyTokenizer);
  };


  configureNodeFromTokenizer(tokenizer) {
    console.log('configure proto ' + this.node + ' from tokenizer')
    tokenizer.skipToken('{');

    while (tokenizer.peekWord() !== '}') {
      const fieldName = tokenizer.nextWord();
      for (const [parameterName, parameter] of this.parameters) {
        if (fieldName === parameterName) {
          console.log('configuring ' + fieldName + ' of ' + this.name + ', node id: ', this.id);

          if (tokenizer.peekWord() === 'IS') {
            tokenizer.skipToken('IS');
            const alias = tokenizer.nextWord();
            console.log('alias:', alias)
            if (!tokenizer.proto.parameters.has(alias))
              throw new Error('Alias "' + alias + '" not found in PROTO ' + this.name);

            parameter.value = tokenizer.proto.parameters.get(alias).value;
          } else if (tokenizer.peekWord() === 'USE') {
            throw new Error('TODO: handle USE')
          } else {
            if (parameter instanceof SFNode) {
              let defName;
              if (tokenizer.peekWord() === 'DEF') {
                tokenizer.skipToken('DEF');
                defName = tokenizer.nextWord();
              }

              const nodeFactory = new NodeFactory();
              const node = nodeFactory.createNode(tokenizer, this.context());
              parameter.value = node;
              if (typeof defName !== 'undefined')
                tokenizer.proto.def.set(defName, node);
            } else
              parameter.setValueFromTokenizer(tokenizer);

            console.log('> value set to ', parameter.value)
          }

        }
      }
    }

    tokenizer.skipToken('}');
  }

  toX3d() {
    const nodeElement = this.xml.createElement(this.value.name);
    nodeElement.setAttribute('id', this.id);
    console.log('ENCODE ' + this.value.name)
    for(const [parameterName, parameter] of this.value.parameters) {
      console.log('  ENCODE ' +  parameterName + ' ? ', typeof parameter.value !== 'undefined');
      if (typeof parameter.value === 'undefined')
        continue;

      parameter.toX3d(parameterName, nodeElement);
    }

    console.log(nodeElement)
    this.xml.appendChild(nodeElement);
    console.log('RESULT:', new XMLSerializer().serializeToString(this.xml));

    return nodeElement;
  }

  toX3dString() {
    return new XMLSerializer().serializeToString(this.toX3d());
  }

  // TODO: can be moved to parameter?
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

  clearReferences() {
    //for (const parameter of this.parameters.values()) {
    //  parameter.nodeRefs = [];
    //  parameter.refNames = [];
    //}
  };
};


function combinePaths(url, parentUrl) {
  if (url.startsWith('http://') || url.startsWith('https://'))
    return url;  // url is already resolved

  let newUrl;
  if (parentUrl.startsWith('http://' || url.startsWith('https://')))
    newUrl = new URL(url, parentUrl.slice(0, parentUrl.lastIndexOf('/') + 1)).href;
  else
    newUrl = parentUrl.slice(0, parentUrl.lastIndexOf('/') + 1) + url;

  // console.log('FROM >' + url + '< AND >' + parentUrl + "< === " + newUrl);
  return newUrl;
}

export { ProtoNode, combinePaths };
