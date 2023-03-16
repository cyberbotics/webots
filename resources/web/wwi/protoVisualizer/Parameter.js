'use strict';

import {SFNode, MFNode, stringifyType} from './Vrml.js';
import Field from './Field.js';
import Node from './Node.js';

export default class Parameter extends Field {
  #aliasLinks;
  #reverseAliasLinks;
  #hidden;
  #isTemplateRegenerator;
  constructor(node, name, type, defaultValue, value, restrictions, isTemplateRegenerator, hidden) {
    super(node, name, type, value, defaultValue);

    this.restrictions = restrictions;
    this.#isTemplateRegenerator = isTemplateRegenerator;
    this.triggerParentRegeneration = isTemplateRegenerator;
    this.#aliasLinks = []; // list of other parameters to notify whenever this instance changes
    this.#reverseAliasLinks = []; // list of parameters that have this parameter as IS link
    this.#hidden = hidden;

    this.setParentRegenerationFlag();
  }

  get value() {
    return super.value;
  }

  set value(newValue) {
    if (newValue.type() !== this.type)
      throw new Error(`Type mismatch, setting ${stringifyType(newValue.type())} to ${stringifyType(this.type)} parameter.`);

    if (this.restrictions.length > 0) {
      if (newValue instanceof MFNode) {
        let canBeInserted = true;
        for (const value of newValue.value) {
          let found = false;
          for (const item of this.restrictions[0].value) {
            if (item.url === value.url) {
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
      } else {
        for (const item of this.restrictions) {
          if ((newValue instanceof SFNode && newValue.value === null) || item.url === newValue.url) {
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

    if (parameter.triggerParentRegeneration && !this.triggerParentRegeneration) {
      this.triggerParentRegeneration = true;
      this.setParentRegenerationFlag();
    }

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

  setParentRegenerationFlag() {
    if (this.triggerParentRegeneration) {
      if (this.value instanceof MFNode) {
        for (const item of this.value.value) {
          if (item.value.isProto === true)
            this.recursivelySetParentRegenerationFlag(item.value.parameters);
        }
      } else if (this.value instanceof SFNode && this.value.value) {
        const node = this.value.value;
        if (node.isProto === true)
          this.recursivelySetParentRegenerationFlag(node.parameters);
      }
    }
  }

  insertNode(view, v, index) {
    if (this.triggerParentRegeneration && v.isProto)
      this.recursivelySetParentRegenerationFlag(v.parameters);

    super.insertNode(view, v, index);
  }

  setValueFromJavaScript(view, v) {
    if (v instanceof Node && this.triggerParentRegeneration && v.isProto)
      this.recursivelySetParentRegenerationFlag(v.parameters);

    super.setValueFromJavaScript(view, v);

    if (this.triggerParentRegeneration)
      this.regenerateParent(view);
  }

  regenerateParent(view) {
    if (this.triggerParentRegeneration) {
      const node = this.node;
      const newField = node.parentField;
      if (typeof newField === 'undefined')
        this.regenerate(view);
      else
        newField.regenerateParent(view);
    } else
      this.regenerate(view);
  }

  recursivelySetParentRegenerationFlag(parameters) {
    for (const parameter of parameters) {
      parameter[1].triggerParentRegeneration = true;
      parameter[1].setParentRegenerationFlag();
    }
  }
}

export { Parameter };
