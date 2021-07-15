'use strict';

import {VRML_TYPE} from './FieldModel.js';

export default class Parameter {
  constructor(name, type, isRegenerator, defaultValue, value) {
    this.nodeRef = undefined;
    this.name = name;
    this.type = type;
    this.isRegenerator = isRegenerator;
    this.defaultValue = defaultValue;
    this.value = value;
  };
};
