'use strict';

import {SFNode, MFNode, stringifyType} from './Vrml.js';
import Field from './Field.js';

export default class Parameter extends Field {
  #aliasLinks;
  #reverseAliasLinks;
  #hidden;
  #isTemplateRegenerator;
  constructor(node, name, type, defaultValue, value, restrictions, isTemplateRegenerator, hidden) {
    super(node, name, type, value, defaultValue);

    this.restrictions = restrictions;
    this.#isTemplateRegenerator = isTemplateRegenerator;
    this.#aliasLinks = []; // list of other parameters to notify whenever this instance changes
    this.#reverseAliasLinks = []; // list of parameters that have this parameter as IS link
    this.#hidden = hidden;
  }

  get value() {
    return super.value;
  }

  set value(newValue) {
    if (newValue.type() !== this.type)
      throw new Error(`Type mismatch, setting ${stringifyType(newValue.type())} to ${stringifyType(this.type)} parameter.`);

    if (this.restrictions.length > 0) {
      if (newValue instanceof MFNode) {
        if (newValue.value === null) {
          super.value = newValue;
          return;
        } else {
          let canBeInserted = true;
          for (const value of newValue.value) {
            let found = false;
            for (const item of this.restrictions[0].value) {
              if (item.equals(value)) {
                found = true;
                break;
              }
            }
            if (found === false) {
              canBeInserted = false;
              break;
            }
          }
          if (canBeInserted) {
            super.value = newValue;
            return;
          }
        }
      } else {
        for (const item of this.restrictions) {
          if ((newValue instanceof SFNode && newValue.value === null) || item.equals(newValue)) {
            super.value = newValue;
            return;
          }
        }
      }

      throw new Error(`Parameter ${this.name} is restricted and the value being set is not permitted.`);
    }

    super.value = newValue;
  }

  get hidden() {
    return this.#hidden;
  }

  get isTemplateRegenerator() {
    return this.#isTemplateRegenerator;
  }

  set isTemplateRegenerator(value) {
    this.#isTemplateRegenerator = value;
    if (this.#isTemplateRegenerator) {
      // if this parameter was set as template regenerator after the creation of the parameter instance, it means an alias link
      // has been created and therefore the information needs to be propagated upwards to all IS parameters in the chain
      for (const item of this.#reverseAliasLinks)
        item.isTemplateRegenerator = this.isTemplateRegenerator;
    }
  }

  get aliasLinks() {
    return this.#aliasLinks;
  }

  set aliasLinks(newValue) {
    this.#aliasLinks = newValue;
  }

  get reverseAliasLinks() {
    return this.#reverseAliasLinks;
  }

  addAliasLink(parameter) {
    this.#aliasLinks.push(parameter);
    if (parameter instanceof Parameter)
      parameter.reverseAliasLinks.push(this);

    // trigger propagation of regeneration status up the IS chain
    if (parameter.isTemplateRegenerator)
      this.isTemplateRegenerator = parameter.isTemplateRegenerator;
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
