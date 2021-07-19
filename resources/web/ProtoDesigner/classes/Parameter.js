'use strict';

import {VRML} from './FieldModel.js';

export default class Parameter {
  constructor(vrmlName, name, type, isRegenerator, defaultValue, value) {
    this.nodeRef = undefined;
    this.vrmlName = vrmlName;
    this.name = name;
    this.type = type;
    this.isTemplateRegenerator = isRegenerator;
    this.defaultValue = defaultValue;
    this.value = value;
  };
};
