'use strict';

import {generateProtoId, generateParameterId} from './utility/utility.js';
//import Parameter from './Parameter.js';
import { FieldModel } from './FieldModel.js';
import { typeFactory, SFNode } from './Vrml.js';
import { NodeFactory } from './NodeFactory.js';

export default class BaseNode {
  constructor(name) {
    this.id = generateProtoId(); // TODO: rename
    this.name = name;
    if (typeof FieldModel[name] === 'undefined')
      throw new Error(`${name} is not a supported BaseNode.`);

    this.model = FieldModel[name];
    console.log('CREATING BASE NODE ' + this.name)

    this.parameters = new Map();
    const fields = this.model['supported'];
    for (const parameterName of Object.keys(fields)) {
      //const parameter = new Parameter(this, generateParameterId(), parameterName, fields[parameterName], false)
      const parameter = typeFactory(fields[parameterName]);
      this.parameters.set(parameterName, parameter);
    }

    this.xml = document.implementation.createDocument('', '', null)
  }

  configureNodeFromTokenizer(tokenizer) {
    console.log('configure base node ' + this.name + ' from tokenizer')
    //if (tokenizer.peekWord() === '{')
    tokenizer.skipToken('{');

    while (tokenizer.peekWord() !== '}') {
      const fieldName = tokenizer.nextWord();
      if (this.model['unsupported'].hasOwnProperty(fieldName)) {
        console.log('consuming unsupported: ' + fieldName)
        tokenizer.consumeTokensByType(this.model['unsupported'][fieldName]);
        continue;
      }

      for (const [parameterName, parameter] of this.parameters) { // TODO: unnecessary to loop through all everytime, do as with unsupported
        if (fieldName === parameterName) {
          console.log('configuring ' + fieldName + ' of ' + this.name + ', node id: ', this.id);

          if (tokenizer.peekWord() === 'IS') {
            tokenizer.skipToken('IS');
            const alias = tokenizer.nextWord();
            // console.log('alias:', alias)
            if (!tokenizer.proto.parameters.has(alias))
              throw new Error('Alias "' + alias + '" not found in PROTO ' + this.name);

            parameter.value = tokenizer.proto.parameters.get(alias).value;
          } else if (tokenizer.peekWord() === 'USE') {
            tokenizer.skipToken('USE');
            const useName = tokenizer.nextWord();
            if (!tokenizer.proto.def.has(useName))
              throw new Error('No DEF name ' + useName + ' found in PROTO ' + tokenizer.proto.name);

            parameter.value = tokenizer.proto.def.get(useName);
            parameter.isUse = true;
          } else {
            if (parameter instanceof SFNode) {
              //let defName;
              //if (tokenizer.peekWord() === 'DEF') {
              //  tokenizer.skipToken('DEF');
              //  defName = tokenizer.nextWord();
              //}

              const nodeFactory = new NodeFactory();
              const node = nodeFactory.createNode(tokenizer);
              parameter.value = node;

              //if (typeof defName !== 'undefined')
              //  tokenizer.proto.def.set(defName, node);
            } else
              parameter.setValueFromTokenizer(tokenizer);

            console.log('> value set to ', parameter.value)
          }

        }
      }
    }

    tokenizer.skipToken('}');
  }

  toX3d(isUse) {
    const nodeElement = this.xml.createElement(this.name);

    if (isUse)
      nodeElement.setAttribute('USE', this.id);
    else {
      nodeElement.setAttribute('id', this.id);
      console.log('ENCODE ' + this.name)
      for (const [parameterName, parameter] of this.parameters) {
        console.log('  ENCODE ' +  parameterName + ' ? ', typeof parameter.value !== 'undefined');
        if (typeof parameter.value === 'undefined')
          continue;

        parameter.toX3d(parameterName, nodeElement);
      }
    }

    this.xml.appendChild(nodeElement);
    console.log('RESULT:', new XMLSerializer().serializeToString(this.xml));

    return nodeElement;
  }

  toJS(isRoot) {
    let jsFields = '';
    for (const [parameterName, parameter] of this.parameters) {
      console.log('JS-encoding of ' + parameterName)
      jsFields += `${parameterName}: {value: ${parameter.toJS()}, defaultValue: ${parameter.toJS()}}, `; // TODO: distinguish value/default value
    }

    if (isRoot)
      return jsFields.slice(0, -2);

    return `{node_name: '${this.name}', fields: {${jsFields.slice(0, -2)}}}`
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

    return copy;
  }
}

export { BaseNode };