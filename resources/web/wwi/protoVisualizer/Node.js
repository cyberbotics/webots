'use strict';

import {getAnId} from '../nodes/utils/id_provider.js';
import TemplateEngine from './TemplateEngine.js';
import Tokenizer from './Tokenizer.js';
import {vrmlFactory, jsifyFromTokenizer} from './Vrml.js';
import {FieldModel} from './FieldModel.js';
import {Parameter} from './Parameter.js';
import Field from './Field.js';
import { VRML } from './vrml_type.js';

export default class Node {
  static cProtoModels = new Map();

  constructor(url, parameterTokenizer, isRoot = false) {
    this.url = url;
    this.isProto = this.url.toLowerCase().endsWith('.proto');
    this.isDerived = false; // updated when the model is determined
    this.isTemplate = false; // updated when the model is determined
    this.name = this.isProto ? this.url.slice(this.url.lastIndexOf('/') + 1).replace('.proto', '') : url;
    console.log(this.name, ':', 'create new node:', this.name, 'from', this.url);

    this.fields = new Map();
    this.parameters = new Map();
    this.def = new Map();

    this.ids = [];
    this.id = getAnId();
    this.isRoot = isRoot;

    // create parameters based on the PROTO model
    if (this.isProto) {
      if (!Node.cProtoModels.has(this.url))
        throw new Error('A PROTO model should be available for', this.name, 'by now.');

      this.model = Node.cProtoModels.get(this.url);
      this.isDerived = typeof FieldModel[this.model['baseType']] === 'undefined';

      // compile externproto list
      this.externProto = new Map();
      for (const item of this.model['externProto']) {
        const protoName = item.split('/').pop().replace('.proto', '');
        this.externProto.set(protoName, item);
      }

      this.isTemplate = this.model['isTemplate'];

      console.log(this.name, 'creating parameters based on PROTO model', this.model)
      for (const [parameterName, parameterModel] of Object.entries(this.model['parameters'])) {
        // console.log('parameter name:', parameterName, 'model:', parameterModel);
        const parameterType = parameterModel['type'];
        const isTemplateRegenerator = parameterModel['isTemplateRegenerator'];
        const restrictions = parameterModel['restrictions'];
        const defaultValue = vrmlFactory(parameterType, parameterModel['defaultValue']);
        const value = vrmlFactory(parameterType, parameterModel['defaultValue']);

        const parameter = new Parameter(this, parameterName, parameterType, restrictions, defaultValue,
          value, isTemplateRegenerator);
        // console.log(parameterName, parameter);
        this.parameters.set(parameterName, parameter);
      }

      // NEED A "CONFIG PARAM FROM HEADER TOKENIZER" HERE
      if (typeof parameterTokenizer !== 'undefined') {
        console.log(this.name, ': configuring parameters of node', this.name, 'from provided parameter tokenizer')
        this.configureParametersFromTokenizer(parameterTokenizer);
      }
      console.log(this.name, ': finished creating parameters of ', this.name);
    } else {
      this.model = FieldModel[this.name];
    }

    //if (isDerivedInstance)
    //  return;

    if (this.isProto)
      this.createBaseType();
    else
      this.generateInternalFields();

    //if (this.isProto) {
    //  this.regenerate();
    //} else {
    //  this.baseType = this.name;
    //  this.model = FieldModel[this.name];
    //
    //  this.generateInternalFields(FieldModel[this.baseType]);
    //}

    if (this.isRoot) {
      this.assignId();
      this.printStructure();
    }
  };

  generateInternalFields() {
    console.log(this.name, ':', 'generating internal fields from model', this.model);
    // set field values based on field model
    for (const fieldName of Object.keys(this.model)) {
      const type = this.model[fieldName]['type'];
      const value = vrmlFactory(type, this.model[fieldName]['defaultValue']);
      const defaultValue = vrmlFactory(type, this.model[fieldName]['defaultValue']);
      const field = new Field(this, fieldName, type, value, defaultValue);
      this.fields.set(fieldName, field);
    }
  }

  getBaseNode() {
    if (this.isProto) {
      console.assert(typeof this.baseType !== 'undefined')
      return this.baseType.getBaseNode();
    }

    return this;
  }

  createBaseType() {
    let protoBody = this.model['rawBody'];

    if (this.isTemplate)
      protoBody = this.regenerateBodyVrml(protoBody);

    // console.log(this.name, ':', 'Upper node after regen', protoBody);

    // configure non-default fields from tokenizer
    const tokenizer = new Tokenizer(protoBody, this);
    tokenizer.tokenize();

    if (this.isDerived) {
      console.log(this.name, ': is a derived PROTO, generating ancestor node')

      console.assert(this.externProto.has(this.model['baseType']));
      const baseTypeUrl = this.externProto.get(this.model['baseType']);
      this.baseType = new Node(baseTypeUrl, tokenizer)
      //this.baseType.configureParametersFromTokenizer(tokenizer);
    } else {
      console.log(this.name, ': is derives from base-node', this.model['baseType'], 'generating it from model')

      this.baseType = new Node(this.model['baseType']); // base nodes don't have parameters
      this.baseType.configureFieldsFromTokenizer(tokenizer);
    }
  }


  regenerate() {

    if (!this.isProto)
      throw new Error('Attempting to regenerate a base node');

    console.log('REGENERATE!')

    this.createBaseType();
  }

  /*
  generateBaseType(baseTypeUrl, tokenizer) {
    console.log(this.name, ':', 'GENERATE BASETYPE OF ', this.name, 'WHICH IS', baseTypeUrl)
    let baseType = new Node(baseTypeUrl, true);
    baseType.configureParametersFromTokenizer(tokenizer);
    console.log(this.name, ':', '>>>>>>>>>AFTER CONFIG', baseType)
    baseType.regenerate();
    console.log(this.name, ':', 'BASETYPE AFTER REFCONFIG', baseType);
    return baseType;
  }
  */

  clearReferences() {
    this.def = new Map();
    this.fields = new Map();
  };


  /*
  collapseParameterLinks() {
    for (const [parameterName, parameter] of this.parameters.entries()) {
      const toAdd = [];
      const toRem = [];
      for (const link of parameter.parameterLinks) {
        console.log('--', parameterName, 'is linked to', link.name);
        for (const l of link.parameterLinks) {
          if (!toRem.includes(link))
            toRem.push(link);
          console.log('----', link.name, 'is linked to', l.name);
          toAdd.push(l);
        }
      }
      console.log(parameterName, 'toAdd', toAdd, 'toRem', toRem);
      for (const item of toRem) {
        const ix = parameter.parameterLinks.indexOf(item);
        parameter.parameterLinks = parameter.parameterLinks.splice(ix, 1);
      }
      for (const item of toAdd)
        parameter.parameterLinks.push(item);
    }

  }
  */

  configureParametersFromTokenizer(tokenizer) {
    console.log(this.name, ':', 'configureParametersFromTokenizer', tokenizer);
    tokenizer.skipToken('{');

    while (tokenizer.peekWord() !== '}') {
      const field = tokenizer.nextWord();
      for (const [fieldName, fieldValue] of this.parameters) {
        if (field === fieldName) {
          console.log(this.name, ':', 'configuring ' + fieldName + ' of ' + this.name);

          if (tokenizer.peekWord() === 'IS') {
            tokenizer.skipToken('IS');
            const alias = tokenizer.nextWord();
            // console.log(this.name, ':', 'alias:', alias, 'reference PROTO:', tokenizer.proto.name);
            if (!tokenizer.proto.parameters.has(alias))
              throw new Error('Alias "' + alias + '" not found in PROTO ' + tokenizer.proto.name);

            console.log(this.name, ':', 'ALIAS:', alias, '<<<---->>>', fieldName)
            const p = tokenizer.proto.parameters.get(alias);
            fieldValue.value = p.value;
            console.log('SETTING VAL TO', p.value)
            p.insertLink(fieldValue);
            p.ref += 1;
            console.log('INSERTED2')

          } else {
            console.log(this.name, ':', 'setting value from tokenizer');
            fieldValue.value.setValueFromTokenizer(tokenizer, this);
          }
        }
      }
    }

    tokenizer.skipToken('}');
    console.log(this.name, ':', 'configureParametersFromTokenizer FINISHED')
  }

  configureFieldsFromTokenizer(tokenizer) {
    console.log(this.name, ':', 'configureFieldsFromTokenizer');
    tokenizer.skipToken('{');

    while (tokenizer.peekWord() !== '}') {
      const field = tokenizer.nextWord();
      for (const [fieldName, fieldValue] of this.fields) {
        if (field === fieldName) {
          console.log(this.name, ':', 'configuring ' + fieldName + ' of ' + this.name);

          if (tokenizer.peekWord() === 'IS') {
            tokenizer.skipToken('IS');
            const alias = tokenizer.nextWord();
            console.log(this.name, ':', 'alias:', alias, 'reference PROTO:', tokenizer.proto.name);
            if (!tokenizer.proto.parameters.has(alias))
              throw new Error('Alias "' + alias + '" not found in PROTO ' + tokenizer.proto.name);

            const p = tokenizer.proto.parameters.get(alias);
            fieldValue.value = p.value;

            p.insertLink(fieldValue);
            p.ref = p.ref + 1
            console.log('INSERTED1', p.name, p.ref)
          } else {
            console.log(this.name, ':', 'setting value of ', fieldName,' from tokenizer');
            fieldValue.value.setValueFromTokenizer(tokenizer);
          }
        }
      }
    }

    tokenizer.skipToken('}');
  }

  printStructure(depth = 0) {
    const index = '--'.repeat(depth);
    if (this.isProto)
      return this.baseType.printStructure(depth);

    console.log(index + this.name, this.ids)

    for (const [fieldName, field] of this.fields) {
      if (field.type === VRML.SFNode && field.value.value !== null) {
        field.value.value.printStructure(depth + 1);
      } else if (field.type === VRML.MFNode) {
        for (const child of field.value.value)
          child.value.printStructure(depth + 1);
      }
    }
  }

  assignId() {
    const index = '--'.repeat();
    if (this.isProto)
      return this.baseType.assignId();

    this.ids.push(getAnId());

    for (const [fieldName, field] of this.fields) {
      if (field.type === VRML.SFNode && field.value.value !== null) {
        field.value.value.assignId();
      } else if (field.type === VRML.MFNode) {
        for (const child of field.value.value)
          child.value.assignId();
      }
    }
  }

  toX3d(isUse, parameterReference) {
    this.xml = document.implementation.createDocument('', '', null);

    if (this.isProto)
      return this.baseType.toX3d();

    const nodeElement = this.xml.createElement(this.name);
    if (isUse) {
      // console.log('is USE! Will reference id: ' + this.id);
      nodeElement.setAttribute('USE', this.id);
      // TODO: needed here as well or sufficient in vrml.js?
      if (['Shape', 'Group', 'Transform', 'Solid', 'Robot'].includes(this.name)) {
        if (parameterReference === 'boundingObject')
          nodeElement.setAttribute('role', 'boundingObject');
      } else if (['BallJointParameters', 'JointParameters', 'HingeJointParameters'].includes(this.name))
        nodeElement.setAttribute('role', parameterReference); // identifies which jointParameter slot the node belongs to
      else if (['Brake', 'PositionSensor', 'Motor'].includes(this.name))
        nodeElement.setAttribute('role', parameterReference); // identifies which device slot the node belongs to
    } else {
      nodeElement.setAttribute('id', this.id);
      for (const [fieldName, field] of this.fields) {
        // console.log('  ENCODE FIELD ' + fieldName);
        //if (typeof fiel.value === 'undefined') // note: SFNode can be null, not undefined
        //  throw new Error('All parameters should be defined, ' + parameterName + ' is not.');
        if (field.isDefault())
          continue;

        field.value.toX3d(fieldName, nodeElement);
      }
    }

    this.xml.appendChild(nodeElement);
    return nodeElement;
  }

  regenerateBodyVrml(protoBody) {
    const fieldsEncoding = this.toJS(true); // make current proto parameters in a format compliant to template engine
    console.log('Encoded fields:', fieldsEncoding);

    const templateEngine = new TemplateEngine();

    return templateEngine.generateVrml(fieldsEncoding, protoBody);
    // console.log('Regenerated Proto Body:\n' + this.protoBody);
  };

  toJS(isFirstNode = false) {
    let jsFields = '';
    for (const [parameterName, parameter] of this.parameters) {
      console.log('JS-encoding of ' + parameterName, parameter);
      jsFields += `${parameterName}: {value: ${parameter.value.toJS()}, defaultValue: ${parameter.defaultValue.toJS()}}, `;
    }

    if (isFirstNode)
      return jsFields.slice(0, -2);

    return `{node_name: '${this.name}', fields: {${jsFields.slice(0, -2)}}}`;
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

    const baseType = rawBody.match(/\{\s*(?:\%\<[\s\S]*?(?:\>\%\s*))?(?:DEF\s+[^\s]+)?\s+([a-zA-Z0-9\_\-\+]+)\s*\{/)[1];

    const tokenizer = new Tokenizer(rawInterface, this); // TODO: remove this
    tokenizer.externProto = externProto; // TODO: still needed?
    tokenizer.tokenize();

    // build parameter list
    tokenizer.skipToken('PROTO');
    this.protoName = tokenizer.nextWord();

    const model = {};
    model['rawBody'] = rawBody;
    model['isTemplate'] = protoText.substring(0, indexBeginInterface).search('template language: javascript') !== -1;
    model['externProto'] = externProto;
    model['baseType'] = baseType;
    model['parameters'] = {}
    while (!tokenizer.peekToken().isEof()) {
      const token = tokenizer.nextToken();
      let nextToken = tokenizer.peekToken();

      const restrictions = [];
      if (token.isKeyword() && nextToken.isPunctuation()) {
        if (nextToken.word() === '{') {
          // parse field restrictions
          tokenizer.skipToken('{');
          const parameterType = token.fieldTypeFromVrml();
          while (tokenizer.peekWord() !== '}') {
            // TODO: handle MF/SFNode restrictions (encode urls?)
            const value = vrmlFactory(parameterType, tokenizer);
            restrictions.push(value);
          }
          tokenizer.skipToken('}');
          nextToken = tokenizer.peekToken(); // we need to update the nextToken as it has to point after the restrictions
        }
      }

      //console.log('restrictions', restrictions)

      if (token.isKeyword() && nextToken.isIdentifier()) {
        const parameterName = nextToken.word();
        const parameterType = token.fieldTypeFromVrml();
        const isRegenerator = rawBody.search('fields.' + parameterName + '.') !== -1;
        tokenizer.nextToken(); // consume the token containing the parameter name
        //const defaultValue = vrmlFactory(parameterType, tokenizer);
        //console.log('found ', parameterName, parameterType)
        const value = jsifyFromTokenizer(parameterType, tokenizer);
        const parameter = {}
        parameter['type'] = parameterType;
        parameter['defaultValue'] = value;
        parameter['isTemplateRegenerator'] = isRegenerator;
        parameter['restrictions'] = restrictions;
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
