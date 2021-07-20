'use strict';

import {VRML} from './FieldModel.js';

export default class Parameter {
  constructor(name, type, isRegenerator, defaultValue, value) {
    this.nodeRefs = [];
    this.refNames = []; // name as defined in the body of the proto (i.e value before an IS)
    this.name = name; // name as defined in the proto header (i.e value after an IS)
    this.type = type;
    this.isTemplateRegenerator = isRegenerator;
    this.defaultValue = defaultValue;
    this.value = value;
  };
};
