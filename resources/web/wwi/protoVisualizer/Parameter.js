'use strict';

import {getAParameterId} from '../nodes/utils/id_provider.js'
import {typeFactory, SFNode, SFBool} from './Vrml.js';


export default class Parameter {
  #value;
  #defaultValue;
  #id = getAParameterId();
  constructor(proto, name, isTemplateRegenerator, defaultValue, value) {
    this.proto = proto; // proto this parameter belongs to
    this.name = name;
    this.isTemplateRegenerator = isTemplateRegenerator;
    this.value = value;
    this.defaultValue = defaultValue;
  }

  get value() {
    return this.#value;
  }

  set value(v) {
    this.#value = v;
  }

  get defaultValue() {
    return this.#value;
  }

  set defaultValue(v) {
    this.#value = v;
  }

  get id() {
    return this.#id;
  }

  set id(value) {
    this.#id = value;
  }

  type() {
    if (typeof this.defaultValue === 'undefined')
      throw new Error('Requesting type of an undefined parameter');

    return this.defaultValue.type();
  }

  isDefault() {
    throw new Error('TODO: implement isDefault');
  }

  clone() {
    throw new Error('Implement clone Parameter')
  }
}

export { Parameter };