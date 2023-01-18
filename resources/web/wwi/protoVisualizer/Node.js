'use strict';

import {getAnId} from '../nodes/utils/id_provider.js';
import TemplateEngine from './TemplateEngine.js';
import Tokenizer from './Tokenizer.js';
import {VRML} from './vrml_type.js';
import {vrmlFactory, jsifyFromTokenizer, SFString} from './Vrml.js';
import {FieldModel} from './FieldModel.js';
import {Parameter} from './Parameter.js';
import WbWorld from '../nodes/WbWorld.js';
import Field from './Field.js';

export default class Node {
  static cProtoModels = new Map();

  constructor(url, protoText, isRoot = false) {
    this.url = url;
    this.isProto = this.url.toLowerCase().endsWith('.proto');
    this.name = this.isProto ? this.url.slice(this.url.lastIndexOf('/') + 1).replace('.proto', '') : url;
    console.log('create new node:', this.name, 'from', this.url);

    this.fields = new Map();
    this.parameters = new Map();
    this.externProto = new Map();
    this.def = new Map();

    // raw PROTO body text must be kept in case the template needs to be regenerated
    //const indexBeginBody = protoText.search(/(?<=\]\s*\n*\r*)({)/g);
    //this.rawBody = protoText.substring(indexBeginBody);
    //if (!this.isTemplate)
    //  this.protoBody = this.rawBody; // body already in VRML format

    // interface (i.e. parameters) only needs to be parsed once and persists through regenerations
    //const indexBeginInterface = protoText.search(/(^\s*PROTO\s+[a-zA-Z0-9\-_+]+\s*\[\s*$)/gm);
    //this.rawInterface = protoText.substring(indexBeginInterface, indexBeginBody);

    // header defines tags and EXTERNPROTO, persists through regenerations
    //this.rawHeader = protoText.substring(0, indexBeginInterface);

    // create parameters based on prototype
    if (this.isProto) {
      if (!Node.cProtoModels.has(this.url))
        throw new Error('A PROTO model should be available for', this.name, 'by now.');

      const protoModel = Node.cProtoModels.get(this.url);
      console.log(protoModel)
      for (const [parameterName, model] of Object.entries(protoModel['parameters'])) {
        console.log(parameterName, model)
        const parameterType = model['type'];
        const defaultValue = vrmlFactory(parameterType, model['defaultValue']);
        //defaultValue.setValueFromModel(model['defaultValue'])
        const value = vrmlFactory(parameterType, model['defaultValue']);
        //value.setValueFromModel(model['defaultValue'])

        const parameter = new Parameter(this, parameterName, parameterType, [], defaultValue, value, false);
        console.log(parameterName, parameter);
        this.parameters.set(parameterName, parameter);
      }
    }

    if (this.isProto) {
      // create fields based on prototype
      const match = Node.cProtoModels.get(this.url)['rawBody'].match(/\{\s*([a-zA-Z0-9\_\-\+]+)\s*\{/);
      this.baseType = match[1];
      console.log(this.name, 'derives from', this.baseType);

      if (typeof FieldModel[this.baseType] !== 'undefined') {
        this.isDerived = false;
        console.log('> derives from a base node')
        const f = FieldModel[this.baseType];

        // set field values based on field model
        for (const fieldName of Object.keys(f)) {
          const type = FieldModel[this.baseType][fieldName]['type'];
          const value = vrmlFactory(type, FieldModel[this.baseType][fieldName]['defaultValue']);
          //value.setValueFromJavaScript();
          const defaultValue = vrmlFactory(type, FieldModel[this.baseType][fieldName]['defaultValue']);
          //defaultValue.setValueFromJavaScript(FieldModel[this.baseType][fieldName]['defaultValue']);
          const field = new Field(fieldName, type, value, defaultValue);
          this.fields.set(fieldName, field);
        }
      } else {
        console.log('> derives from a PROTO')
        this.isDerived = true;
      }
    } else {
      this.baseType = this.name;

      // create from FieldModel
      const fields = FieldModel[this.name];
      for (const fieldName of Object.keys(fields)) {
        const type = FieldModel[this.baseType][fieldName]['type'];
        const value = vrmlFactory(type, FieldModel[this.baseType][fieldName]['defaultValue']);
        //value.setValueFromJavaScript(FieldModel[this.baseType][fieldName]['defaultValue']);
        const defaultValue = vrmlFactory(type, FieldModel[this.baseType][fieldName]['defaultValue']);
        //defaultValue.setValueFromJavaScript(FieldModel[this.baseType][fieldName]['defaultValue']);
        const field = new Field(fieldName, type, value, defaultValue);
        this.fields.set(fieldName, field);
      }
    }

    if (this.isProto) {
      const tokenizer = new Tokenizer(Node.cProtoModels.get(this.url)['rawBody'], this);
      tokenizer.tokenize();
      this.configureNodeFromTokenizer(tokenizer);
    }
  };

  configureNodeFromTokenizer(tokenizer) {
    console.log('configureNodeFromTokenizer')
    tokenizer.skipToken('{');

    while (tokenizer.peekWord() !== '}') {
      console.log('here')
      const field = tokenizer.nextWord();
      for (const [fieldName, fieldValue] of this.fields) {
        if (field === fieldName) {
          console.log('configuring ' + fieldName + ' of ' + this.name);

          if (tokenizer.peekWord() === 'IS') {
            tokenizer.skipToken('IS');
            const alias = tokenizer.nextWord();
            console.log('alias:', alias);
            if (!this.parameters.has(alias))
              throw new Error('Alias "' + alias + '" not found in PROTO ' + this.name);

            this.fields.set(fieldName, this.parameters.get(alias));
            console.log(this)
          } else {
            console.log('setting value from tokenizer')
            fieldValue.value.setValueFromTokenizer(tokenizer, this);
          }
        }
      }
    }

    tokenizer.skipToken('}');
  }

  toX3d(isUse, parameterReference) {
    this.xml = document.implementation.createDocument('', '', null);

    const nodeElement = this.xml.createElement(this.baseType);
    //nodeElement.setAttribute('id', this.id);
    for (const [fieldName, field] of this.fields) {
      console.log('  ENCODE FIELD ' + fieldName);
      //if (typeof fiel.value === 'undefined') // note: SFNode can be null, not undefined
      //  throw new Error('All parameters should be defined, ' + parameterName + ' is not.');

      console.log(field)
      //if (field.isDefault())
      //  continue;

      field.value.toX3d(fieldName, nodeElement);
    }

    this.xml.appendChild(nodeElement);
    console.log('RESULT:', new XMLSerializer().serializeToString(this.xml));

    return nodeElement;
  }

  static async prepareProtoDependencies(protoUrl) {
    return new Promise((resolve, reject) => {
      const xmlhttp = new XMLHttpRequest();
      xmlhttp.open('GET', protoUrl, true);
      xmlhttp.overrideMimeType('plain/text');
      xmlhttp.onreadystatechange = async() => {
        if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
          resolve(xmlhttp.responseText);
      };
      xmlhttp.send();
    }).then(async text => {
      // interface (i.e. parameters) only needs to be parsed once and persists through regenerations
      const indexBeginInterface = text.search(/(^\s*PROTO\s+[a-zA-Z0-9\-_+]+\s*\[\s*$)/gm);
      const rawHeader = text.substring(0, indexBeginInterface);

      const promises = [];
      const externProto = []
      const lines = rawHeader.split('\n');
      for (let i = 0; i < lines.length; i++) {
        let line = lines[i];
        if (line.indexOf('EXTERNPROTO') !== -1) { // get only the text after 'EXTERNPROTO' for the single line
          line = line.split('EXTERNPROTO')[1].trim();
          let address = line.replaceAll('"', '');
          if (address.startsWith('webots://'))
            address = 'https://raw.githubusercontent.com/cyberbotics/webots/R2022b/' + address.substring(9);
          else
            address = combinePaths(address, protoUrl);

          externProto.push(address);
          promises.push(await Node.prepareProtoDependencies(address));
        }
      }

      Promise.all(promises).then(async() => {
        await this.generateProtoModel(protoUrl, text, externProto);
      });

    });
  }

  static async generateProtoModel(protoUrl, protoText, externProto) {
    // if it depends on other models, create them first
    console.log('GENERATING MODEL OF ', protoUrl)

    if (Node.cProtoModels.has(protoUrl)) {
      console.log('> model known already, skipping.');
      return;
    }


    // interface (i.e. parameters) only needs to be parsed once and persists through regenerations
    const indexBeginInterface = protoText.search(/(^\s*PROTO\s+[a-zA-Z0-9\-_+]+\s*\[\s*$)/gm);
    const indexBeginBody = protoText.search(/(?<=\]\s*\n*\r*)({)/g);

    let rawInterface = protoText.substring(indexBeginInterface, indexBeginBody);

    // change all relative paths to remote ones
    const re = /"(?:[^"]*)\.(jpe?g|png|hdr|obj|stl|dae)"/g;
    let result;
    while ((result = re.exec(rawInterface)) !== null)
      rawInterface = rawInterface.replace(result[0], '"' + combinePaths(result[0].slice(1, -1),protoUrl) + '"');

    let rawBody = protoText.substring(indexBeginBody);
    // change all relative paths to remote ones
    while ((result = re.exec(rawBody)) !== null)
      rawBody = rawBody.replace(result[0], '"' + combinePaths(result[0].slice(1, -1), protoUrl) + '"');

    const tokenizer = new Tokenizer(rawInterface, this); // TODO: remove this
    tokenizer.externProto = externProto;
    tokenizer.tokenize();

    // build parameter list
    tokenizer.skipToken('PROTO');
    this.protoName = tokenizer.nextWord();

    const model = {};
    model['rawBody'] = rawBody;
    model['parameters'] = {}
    while (!tokenizer.peekToken().isEof()) {
      const token = tokenizer.nextToken();
      let nextToken = tokenizer.peekToken();

      const restrictions = [];
      //if (token.isKeyword() && nextToken.isPunctuation()) {
      //  if (nextToken.word() === '{') {
      //    // parse field restrictions
      //    tokenizer.skipToken('{');
      //    const parameterType = token.fieldTypeFromVrml();
      //    while (tokenizer.peekWord() !== '}') {
      //      const value = vrmlFactory(parameterType, tokenizer);
      //      restrictions.push(value);
      //    }
      //    tokenizer.skipToken('}');
      //    nextToken = tokenizer.peekToken(); // we need to update the nextToken as it has to point after the restrictions
      //  }
      //}

      if (token.isKeyword() && nextToken.isIdentifier()) {
        const parameterName = nextToken.word();
        const parameterType = token.fieldTypeFromVrml();
        //const isRegenerator = this.isTemplate ? (this.rawBody.search('fields.' + parameterName + '.') !== -1) : false;
        tokenizer.nextToken(); // consume the token containing the parameter name
        //const defaultValue = vrmlFactory(parameterType, tokenizer);
        //console.log('found ', parameterName, parameterType)
        const value = jsifyFromTokenizer(parameterType, tokenizer);

        const parameter = {}
        parameter['type'] = parameterType;
        parameter['defaultValue'] = value;
        model['parameters'][parameterName] = parameter;
        //model[protoUrl][parameterName]['defaultValue'] = defaultValue.toJS(false);
        //model[protoUrl][parameterName]['isRegenerator'] = isRegenerator;
      }
    }

    Node.cProtoModels.set(protoUrl, model);
  }
};

// TODO: move to utility?
function combinePaths(url, parentUrl) {
  if (url.startsWith('http://') || url.startsWith('https://'))
    return url; // url is already resolved

  if (url.startsWith('webots://')) {
    if (parentUrl.startsWith('http://') || parentUrl.startsWith('https://')) {
      // eslint-disable-next-line
      const match = parentUrl.match(/(https:\/\/raw.githubusercontent.com\/cyberbotics\/webots\/[a-zA-Z0-9\_\-\+]+\/)/);
      if (match === null) {
        console.warn('Expected prefix not found in parent url.');
        return url;
      }

      return url.replace('webots://', match[0]);
    } else
      return url;
  }

  let newUrl;
  if (parentUrl.startsWith('http://' || url.startsWith('https://')))
    newUrl = new URL(url, parentUrl.slice(0, parentUrl.lastIndexOf('/') + 1)).href;
  else
    newUrl = parentUrl.slice(0, parentUrl.lastIndexOf('/') + 1) + url;

  // console.log('FROM >' + url + '< AND >' + parentUrl + "< === " + newUrl);
  return newUrl;
}

export { Node };
