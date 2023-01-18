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
    if (this.isProto && Node.cProtoModels.has(this.url)) {
      const protoModel = Node.cProtoModels.get(this.url);
      console.log(protoModel)
      for (const [parameterName, parameterValue] of Object.entries(protoModel['parameters'])) {
        console.log(parameterName, parameterValue)
        const parameterType = parameterValue['type'];
        const v = vrmlFactory(parameterType);
        v.value = parameterValue['defaultValue'];
        this.parameters.set(parameterName, v)
      }


    } else
      throw new Error('A PROTO model should be available for', this.name, 'by now.')

    if (this.isProto) {
      // determine basetype
      const match = this.rawBody.match(/\{\s*([a-zA-Z0-9\_\-\+]+)\s*\{/);
      const baseType = match[1];
      console.log(this.name, 'derives from', baseType);

      if (typeof FieldModel[baseType] !== 'undefined') {
        console.log('> derives from a base node')
        const f = FieldModel[baseType];

        // set field values based on field model
        for (const fieldName of Object.keys(f)) {
          const type = FieldModel[baseType][fieldName]['type'];
          const value = vrmlFactory(type);
          value.setValueFromJavaScript(FieldModel[baseType][fieldName]['defaultValue']);
          const field = new Field(fieldName, type, value);
          this.fields.set(fieldName, field);
        }
      } else {
        console.log('> derives from a PROTO')

      }
    }
  };


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
      rawInterface = rawInterface.replace(result[0], '"' + combinePaths(result[0].slice(1, -1), this.url) + '"');

    const tokenizer = new Tokenizer(rawInterface, this); // TODO: remove this
    tokenizer.externProto = externProto;
    tokenizer.tokenize();

    // build parameter list
    tokenizer.skipToken('PROTO');
    this.protoName = tokenizer.nextWord();

    const model = {};
    model['rawBody'] = protoText.substring(indexBeginBody);
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
