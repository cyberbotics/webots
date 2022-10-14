'use strict';

import {FieldModel} from './FieldModel.js';
import {VRML, generateProtoId, generateParameterId} from './utility/utility.js';
import Parameter from './Parameter.js';
import Tokenizer from './Tokenizer.js';
import ProtoNode from './ProtoNode.js';

export default class BaseNode {
  constructor(name) {
    this.id = generateProtoId(); // TODO: rename
    this.name = name
    if (typeof FieldModel[name] === 'undefined')
      throw new Error(`${name} is not a supported BaseNode.`);

    this.model = FieldModel[name];
    console.log('CREATING BASE NODE ' + this.name)

    this.parameters = new Map();
    const fields = FieldModel[name]['supported'];
    for (const [parameterName, parameterType] in fields) {
      const parameter = new Parameter(this, generateParameterId(), parameterName, parameterType, false)
      this.parameters.set(parameterName, parameter);
    }
  }

  configureFromTokenizer(tokenizer) {

  }

}

export { BaseNode };