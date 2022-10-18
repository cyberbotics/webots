'use strict';

import {getAParameterId} from '../nodes/utils/id_provider.js'
import {typeFactory, SFNode, SFBool} from './Vrml.js';


export default class Parameter {
  constructor(proto, name, isTemplateRegenerator, defaultValue, value) {
    this.proto = proto; // proto this parameter belongs to
    this.id = getAParameterId();
    this.name = name;
    this.isTemplateRegenerator = isTemplateRegenerator;
    this.value = value;
    this.defaultValue = defaultValue;
  }

  type() {
    if (typeof this.defaultValue === 'undefined')
      throw new Error('Requesting type of an undefined parameter');

    return this.defaultValue.type();
  }

  isDefault() {
    throw new Error('TODO: implement isDefault');
  }
}

export { Parameter };