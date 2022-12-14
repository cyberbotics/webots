'use strict';

import {getAnId} from '../nodes/utils/id_provider.js';
import TemplateEngine from './TemplateEngine.js';
import Tokenizer from './Tokenizer.js';
import {VRML} from './vrml_type.js';
import {vrmlFactory} from './Vrml.js';
import {FieldModel} from './FieldModel.js';
import {Parameter} from './Parameter.js';

export default class Node {
  static cProtoModels = new Map();
  static cBaseModels = new Map();

  constructor(url, protoText, isRoot = false) {
    // IMPORTANT! When adding new member variables of type Map, modify the .clone method so that it creates a copy of it
    this.id = getAnId();
    this.baseType = undefined; // may correspond to a base-node or another PROTO if it's a derived PROTO
    this.isRoot = isRoot;

    this.url = url;
    this.isProto = this.url.toLowerCase().endsWith('.proto');

    this.name = this.isProto ? this.url.slice(this.url.lastIndexOf('/') + 1).replace('.proto', '') : url;
    // console.log('CREATING ' + (this.isProto ? 'PROTO ' : 'BASENODE ') + this.name + ', id: ', this.id);

    this.parameters = new Map();
    this.externProto = new Map();
    this.def = new Map();

    if (!this.isProto) {
      // create parameters from the pre-defined FieldModel
      const fields = FieldModel[this.name];
      for (const parameterName of Object.keys(fields)) {
        const type = FieldModel[this.name][parameterName]['type'];
        const defaultValue = vrmlFactory(type);
        defaultValue.setValueFromJavaScript(FieldModel[this.name][parameterName]['defaultValue']);
        const value = defaultValue.clone();
        const parameter = new Parameter(this, parameterName, type, defaultValue, value, false);
        // console.log(parameterName + ' has parent ' + this.name);
        this.parameters.set(parameterName, parameter);
      }

      return;
    }

    // for PROTO only
    this.isTemplate = protoText.search('template language: javascript') !== -1;
    if (this.isTemplate) {
      // console.log('PROTO is a template!');
      this.templateEngine = new TemplateEngine();
    }

    // change all relative paths to remote ones
    const re = /"(?:[^"]*)\.(jpe?g|png|hdr|obj|stl|dae)"/g;
    let result;
    while ((result = re.exec(protoText)) !== null)
      protoText = protoText.replace(result[0], '"' + combinePaths(result[0].slice(1, -1), this.url) + '"');

    // raw PROTO body text must be kept in case the template needs to be regenerated
    const indexBeginBody = protoText.search(/(?<=\]\s*\n*\r*)({)/g);
    this.rawBody = protoText.substring(indexBeginBody);
    if (!this.isTemplate)
      this.protoBody = this.rawBody; // body already in VRML format

    // interface (i.e. parameters) only needs to be parsed once and persists through regenerations
    const indexBeginInterface = protoText.search(/(^\s*PROTO\s+[a-zA-Z0-9\-_+]+\s*\[\s*$)/gm);
    this.rawInterface = protoText.substring(indexBeginInterface, indexBeginBody);

    // header defines tags and EXTERNPROTO, persists through regenerations
    this.rawHeader = protoText.substring(0, indexBeginInterface);

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
          address = combinePaths(address, this.url);

        this.externProto.set(protoName, address);
        this.promises.push(this.getExternProto(address));
      }
    }
  };

  async getExternProto(protoUrl) {
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
      // console.log('downloaded ' + protoUrl + ', generating prototype');
      const prototype = await this.createPrototype(text, protoUrl);
      return prototype;
    });
  }

  async createPrototype(protoText, protoUrl) {
    if (!Node.cProtoModels.has(protoUrl)) {
      const proto = new Node(protoUrl, protoText);
      await proto.generateInterface();
      // console.log('adding proto model: ', protoUrl);
      Node.cProtoModels.set(protoUrl, proto);
    }
  }

  async generateInterface() {
    return Promise.all(this.promises).then(async() => {
      // parse header and map each parameter entry
      // console.log(this.name + ': all EXTERNPROTO promises have been resolved');
      await this.parseHead();
    });
  }

  clone() {
    let copy = Object.assign(Object.create(Object.getPrototypeOf(this)), this);

    if (typeof this.baseType !== 'undefined')
      copy.baseType = this.baseType.clone();

    copy.id = getAnId();
    // console.log('cloned ' + this.name + ' id : ' + this.id + ' -> ' + copy.id);
    copy.parameters = new Map();
    for (const [parameterName, parameter] of this.parameters) {
      if (typeof parameter !== 'undefined') {
        // console.log('cloning parameter ' + parameterName + ' (type ' + parameter.type + ')');
        const parameterCopy = parameter.clone();
        parameterCopy.node = copy;
        //console.log('NODE ORIGINAL', parameter);
        //console.log('NODE COPY', parameterCopy);
        copy.parameters.set(parameterName, parameterCopy);
      }
    }

    copy.def = new Map(this.def); // TODO: probably wrong, need to clone nodes?
    copy.externProto = new Map(this.externProto);
    return copy;
  }

  async parseHead() {
    // console.log('PARSE HEAD OF ' + this.name);
    const headTokenizer = new Tokenizer(this.rawInterface, this);
    headTokenizer.tokenize();

    // build parameter list
    headTokenizer.skipToken('PROTO');
    this.protoName = headTokenizer.nextWord();

    while (!headTokenizer.peekToken().isEof()) {
      const token = headTokenizer.nextToken();
      let nextToken = headTokenizer.peekToken();

      if (token.isKeyword() && nextToken.isPunctuation()) {
        if (nextToken.word() === '{') {
          // TODO: field restrictions are not supported yet, consume the tokens
          headTokenizer.consumeTokensByType(VRML.SFNode);
          nextToken = headTokenizer.peekToken(); // update upcoming token reference after consumption
        }
      }

      if (token.isKeyword() && nextToken.isIdentifier()) {
        const parameterName = nextToken.word();
        const parameterType = token.fieldTypeFromVrml();
        const isRegenerator = this.isTemplate ? (this.rawBody.search('fields.' + parameterName + '.') !== -1) : false;
        headTokenizer.nextToken(); // consume the token containing the parameter name

        // console.log('INTERFACE PARAMETER ' + parameterName + ', TYPE: ' + parameterType + ', VALUE:');
        const defaultValue = vrmlFactory(parameterType, headTokenizer);
        const value = defaultValue.clone();
        const parameter = new Parameter(this, parameterName, parameterType, defaultValue, value, isRegenerator);
        // console.log(parameterName + ' has parent ' + this.name);
        this.parameters.set(parameterName, parameter);
      }
    }
  };

  parseBody(isRegenerating = false) {
    // console.log('PARSE BODY OF ' + this.name);
    this.clearReferences();
    // note: if not a template, the body is already pure VRML
    if (this.isTemplate)
      this.regenerateBodyVrml(); // overwrites this.protoBody with a purely VRML compliant body

    // tokenize body
    const tokenizer = new Tokenizer(this.protoBody, this);
    tokenizer.tokenize();

    // skip bracket opening the PROTO body
    tokenizer.skipToken('{');

    this.baseType = Node.createNode(tokenizer);
    // console.log('STRUCT', this);

    tokenizer.skipToken('}');
  };

  configureNodeFromTokenizer(tokenizer) {
    // console.log('configure ' + (this.isProto ? 'proto ' : 'base node ') + this.name + ' from tokenizer');
    tokenizer.skipToken('{');

    while (tokenizer.peekWord() !== '}') {
      const fieldName = tokenizer.nextWord();
      for (const [parameterName, parameter] of this.parameters) {
        // console.log(parameter);
        if (fieldName === parameterName) {
          // console.log('configuring ' + fieldName + ' of ' + this.name + ', node id: ', this.id);

          if (tokenizer.peekWord() === 'IS') {
            tokenizer.skipToken('IS');
            const alias = tokenizer.nextWord();
            // console.log('alias:', alias);
            if (!tokenizer.proto.parameters.has(alias))
              throw new Error('Alias "' + alias + '" not found in PROTO ' + this.name);

            const exposedParameter = tokenizer.proto.parameters.get(alias);
            parameter.value = exposedParameter.value.clone();
            //console.log('ORIGINAL:', exposedParameter.value);
            //console.log('CLONE:', parameter.value);
            exposedParameter.insertLink(parameter);
          } else
            parameter.value.setValueFromTokenizer(tokenizer, this);
        }
      }
    }

    tokenizer.skipToken('}');
  }

  toX3d(isUse, parameterReference) {
    this.xml = document.implementation.createDocument('', '', null);

    // console.log('ENCODE NODE ' + this.name + ', isUse? ', isUse, ' parameterReference ?', parameterReference);
    // if this node has a value (i.e. this.baseType is defined) then it means we have not yet reached the bottom as only
    // base-nodes should write x3d. If it has a value, then it means the current node is a derived PROTO.
    if (typeof this.baseType !== 'undefined')
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
      // console.log('ENCODE ' + this.name, ', id: ', this.id)
      for (const [parameterName, parameter] of this.parameters) {
        // console.log('  ENCODE PARAMETER ' + parameterName + ', is default? ', parameter.isDefault());
        if (typeof parameter.value === 'undefined') // note: SFNode can be null, not undefined
          throw new Error('All parameters should be defined, ' + parameterName + ' is not.');

        if (parameter.isDefault())
          continue;

        parameter.value.toX3d(parameterName, nodeElement);
      }
    }

    this.xml.appendChild(nodeElement);
    // console.log('RESULT:', new XMLSerializer().serializeToString(this.xml));

    return nodeElement;
  }

  toJS(isFirstNode = false) {
    let jsFields = '';
    for (const [parameterName, parameter] of this.parameters) {
      // console.log('JS-encoding of ' + parameterName);
      jsFields += `${parameterName}: {value: ${parameter.value.toJS()}, defaultValue: ${parameter.defaultValue.toJS()}}, `;
    }

    if (isFirstNode)
      return jsFields.slice(0, -2);

    return `{node_name: '${this.name}', fields: {${jsFields.slice(0, -2)}}}`;
  }

  toVrml() {
    let vrml = '';
    vrml += `${this.name}{`;
    for (const [parameterName, parameter] of this.parameters) {
      if (!parameter.isDefault())
        vrml += `${parameterName} ${parameter.value.toVrml()}`;
    }
    vrml += '}\n';

    return vrml.slice(0, -1);
  }

  regenerateBodyVrml() {
    const fieldsEncoding = this.toJS(true); // make current proto parameters in a format compliant to template engine
    // console.log(fieldsEncoding);

    if (typeof this.templateEngine === 'undefined')
      throw new Error('Regeneration was called but the template engine is not defined (i.e this.isTemplate is false)');

    this.protoBody = this.templateEngine.generateVrml(fieldsEncoding, this.rawBody);
    console.log('Regenerated Proto Body:\n' + this.protoBody);
  };

  clearReferences() {
    this.def = new Map();

    for (const [parameterName, parameter] of this.parameters)
      parameter.resetParameterLinks();
  };

  getParameterByName(name) {
    if (!this.parameters.has(name))
      throw new Error('Node ' + this.name + ' does not have a parameter named: ' + name);

    return this.parameters.get(name);
  }

  getBaseNode() {
    if (this.isProto)
      return this.baseType.getBaseNode();

    return this;
  }

  static createNode(tokenizer) {
    let defName;
    if (tokenizer.peekWord() === 'DEF') {
      tokenizer.skipToken('DEF');
      defName = tokenizer.nextWord();
    } else if (tokenizer.peekWord() === 'USE') {
      tokenizer.skipToken('USE');
      const useName = tokenizer.nextWord();
      if (!tokenizer.proto.def.has(useName))
        throw new Error('No DEF name ' + useName + ' found in PROTO ' + tokenizer.proto.name);
      // else
      //   console.log('USE reference of ' + useName + ' exists');

      return tokenizer.proto.def.get(useName);
    }

    const nodeName = tokenizer.nextWord();
    if (nodeName === 'NULL')
      // console.log('is null.');
      return null;

    // console.log('CREATE NODE ' + nodeName);
    let node;
    if (typeof FieldModel[nodeName] !== 'undefined') { // it's a base node
      if (!Node.cBaseModels.has(nodeName)) {
        // create prototype if none is available
        const model = new Node(nodeName);
        Node.cBaseModels.set(nodeName, model);
      }

      node = Node.cBaseModels.get(nodeName).clone();
      // console.log('ORIGINAL', Node.cBaseModels.get(nodeName))
      // console.log('CLONE', node)
    } else { // it's a PROTO node
      // note: protoModels are expected to already contain models for all PROTO we need this session since these have been
      // downloaded when retrieving the EXTERNPROTO and a prototype for each should have been computed already
      if (!tokenizer.proto.externProto.has(nodeName))
        throw new Error('Node name ' + nodeName + ' is not recognized. Was it declared as EXTERNPROTO?');

      const url = tokenizer.proto.externProto.get(nodeName);
      if (!Node.cProtoModels.has(url))
        throw new Error('Model of PROTO ' + nodeName + ' not available. Was it declared as EXTERNPROTO?');

      node = Node.cProtoModels.get(url).clone();
    }

    if (typeof defName !== 'undefined')
      tokenizer.proto.def.set(defName, node);

    node.configureNodeFromTokenizer(tokenizer);
    if (node.isProto)
      node.parseBody();

    return node;
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
