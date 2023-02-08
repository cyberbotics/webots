'use strict';

import {SFNode, stringifyType} from './Vrml.js';
import Field from './Field.js';

export default class Parameter extends Field {
  #aliasLinks;
  #isTemplateRegenerator;
  constructor(node, name, type, defaultValue, value, restrictions, isTemplateRegenerator) {
    super(node, name, type, value, defaultValue);

    this.restrictions = restrictions;
    this.#isTemplateRegenerator = isTemplateRegenerator;
    this.#aliasLinks = []; // list of other parameters to notify whenever this instance changes
  }

  get value() {
    return super.value;
  }

  set value(newValue) {
    if (newValue.type() !== this.type)
      throw new Error(`Type mismatch, setting ${stringifyType(newValue.type())} to ${stringifyType(this.type)} parameter.`);

    if (this.restrictions.length > 0) {
      for (const item of this.restrictions) {
        if ((newValue instanceof SFNode && newValue.value === null) || item.equals(newValue)) {
          super.value = newValue;
          return;
        }
      }

      throw new Error(`Parameter ${this.name} is restricted and the value being set is not permitted.`);
    }

    super.value = newValue;
  }

  get isTemplateRegenerator() {
    return this.#isTemplateRegenerator;
  }

  set isTemplateRegenerator(value) {
    this.#isTemplateRegenerator = value;
  }

  get aliasLinks() {
    return this.#aliasLinks;
  }

  set aliasLinks(newValue) {
    this.#aliasLinks = newValue;
  }

  addAliasLink(parameter) {
    this.#aliasLinks.push(parameter);
  }

  resetAliasLinks() {
    this.#aliasLinks = [];
  }

  linksToNotify() {
    let links = [];
    for (const link of this.aliasLinks) {
      if (link instanceof Parameter && link.aliasLinks.length > 0)
        links = links.concat(link.linksToNotify());

      if (!link.node.isProto)
        links.push(link);
    }

    return links;
  }
}

export { Parameter };
