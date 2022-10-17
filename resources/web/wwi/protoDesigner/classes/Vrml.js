'use strict';

import NodeFactory from './NodeFactory.js';
import {VRML} from './constants.js';

export class SFBool {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = tokenizer.nextToken().toBool();
  }

  toX3d(name, parentElement) {
    parentElement.setAttribute(name, this.#value);
  }

  toJS() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class SFInt32 {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = tokenizer.nextToken().toInt();
  }

  toX3d(name, parentElement) {
    parentElement.setAttribute(name, this.#value);
  }

  toJS() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class SFFloat {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = tokenizer.nextToken().toFloat();
  }

  toX3d(name, parentElement) {
    parentElement.setAttribute(name, this.#value);
  }

  toJS() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class SFString {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }


  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = tokenizer.nextWord();
  }

  toX3d(name, parentElement) {
    parentElement.setAttribute(name, this.#value);
  }

  toJS() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class SFVec2f {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat()};
  }

  toX3d(name, parentElement) {
    parentElement.setAttribute(name, `${this.#value.x} ${this.#value.y}`);
  }

  toJS() {
    return `{x: ${this.#value.x}, y: ${this.#value.y}}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.#value.x === other.value.x && this.value.y === other.value.y;
  }
}

export class SFVec3f {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    console.log(value)
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat(), z: tokenizer.nextToken().toFloat()};
  }

  toX3d(name, parentElement) {
    parentElement.setAttribute(name, `${this.#value.x} ${this.#value.y} ${this.#value.z}`);
  }

  toJS() {
    return `{x: ${this.#value.x}, y: ${this.#value.y}, z: ${this.#value.z}}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.x === other.value.x && this.value.y === other.value.y && this.value.z === other.value.z;
  }
}

export class SFColor {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = {r: tokenizer.nextToken().toFloat(), g: tokenizer.nextToken().toFloat(), b: tokenizer.nextToken().toFloat()};
  }

  toX3d(name, parentElement) {
    parentElement.setAttribute(name, `${this.#value.r} ${this.#value.g} ${this.#value.b}`);
  }

  toJS() {
    return `{r: ${this.#value.r}, g: ${this.#value.g}, b: ${this.#value.b}}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.r === other.value.r && this.value.g === other.value.g && this.value.b === other.value.b;
  }
}

export class SFRotation {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat(),
                   z: tokenizer.nextToken().toFloat(), a: tokenizer.nextToken().toFloat()};
  }

  toX3d(name, parentElement) {
    parentElement.setAttribute(name, `${this.#value.x} ${this.#value.y} ${this.#value.z} ${this.#value.a}`);
  }

  toJS() {
    return `{x: ${this.#value.x}, y: ${this.#value.y}, z: ${this.#value.z}, a: ${this.#value.a}}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.x === other.value.x && this.value.y === other.value.y &&
           this.value.z === other.value.z && this.value.a === other.value.a;
  }
}

export class SFNode {
  #value;
  constructor(tokenizer) {
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }


  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    const nodeFactory = new NodeFactory();
    this.#value = nodeFactory.createNode(tokenizer);
  }

  toX3d(name, parentElement) {
    if (typeof this.#value === 'undefined')
      return;

    const nodeX3d = this.#value.toX3d();
    if (typeof nodeX3d !== 'undefined')
      parentElement.appendChild(nodeX3d);
  }

  toJS() {
    throw new Error('TODO: toJS of SFNode')
  }

  equals(other) {
    throw new Error('TODO: equals for SFNode')
  }
}

export class MFBool {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFBool(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFBool(tokenizer));
  }

  toX3d() {
    let x3d = '';
    if (this.#value.length > 0){
      this.#value.forEach(element => x3d += element.toX3d() + ' ');
      x3d.slice(0, -1)
    }
    return x3d;
  }

  toJS() {
    throw new Error('TODO: toJS of MFBool');
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class MFInt32 {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFInt32(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFInt32(tokenizer));
  }

  toX3d() {
    x3d = '';
    if (this.#value.length > 0){
      this.#value.forEach(element => x3d += element.toX3d() + ' ');
      x3d.slice(0, -1)
    }
    return x3d;
  }

  toJS() {
    throw new Error('TODO: toJS of MFInt32');
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class MFFloat {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFFloat(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFFloat(tokenizer));
  }

  toX3d() {
    let x3d = '';
    if (this.#value.length > 0){
      this.#value.forEach(element => x3d += element.toX3d() + ' ');
      x3d.slice(0, -1)
    }
    return x3d;
  }

  toJS() {
    throw new Error('TODO: toJS of MFFloat');
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class MFString {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFString(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFString(tokenizer));
  }

  toX3d() {
    let x3d = '';
    if (this.#value.length > 0){
      this.#value.forEach(element => x3d += element.toX3d() + ' ');
      x3d.slice(0, -1)
    }
    return x3d;
  }

  toJS() {
    throw new Error('TODO: toJS of MFString');
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class MFVec2f {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFVec2f(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFVec2f(tokenizer));
  }

  toX3d() {
    x3d = '';
    if (this.#value.length > 0){
      this.#value.forEach(element => x3d += element.toX3d() + ' ');
      x3d.slice(0, -1)
    }
    return x3d;
  }

  toJS() {
    throw new Error('TODO: toJS of MFVec2f');
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class MFVec3f {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFVec3f(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFVec3f(tokenizer));
  }

  toX3d() {
    x3d = '';
    if (this.#value.length > 0){
      this.#value.forEach(element => x3d += element.toX3d() + ' ');
      x3d.slice(0, -1)
    }
    return x3d;
  }

  toJS() {
    throw new Error('TODO: toJS of MFVec3f');
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class MFColor {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFColor(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFColor(tokenizer));
  }

  toX3d() {
    let x3d = '';
    if (this.#value.length > 0){
      this.#value.forEach(element => x3d += element.toX3d() + ' ');
      x3d.slice(0, -1)
    }
    return x3d;
  }

  toJS() {
    throw new Error('TODO: toJS of MFColor');
  }

  equals(other) {
    return this.#value === other.value;
  }
}


export class MFRotation {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFRotation(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFRotation(tokenizer));
  }

  toX3d(name, parentElement) {
    let x3d = '';
    if (this.#value.length > 0){
      this.#value.forEach(element => x3d += element.toX3d() + ' ');
      x3d.slice(0, -1)
    }
    return x3d;
  }

  toJS() {
    throw new Error('TODO: toJS of MFRotation');
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class MFNode {
  #value;
  constructor(tokenizer) {
    this.#value = []
    if (typeof tokenizer !== 'undefined')
      this.setValueFromTokenizer(tokenizer);
  }

  get value() {
    return this.#value;
  }

  set value(value) {
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFNode(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.#value.push(new SFNode(tokenizer));
  }

  toX3d(name, parentElement) {
    this.#value.forEach((item) => parentElement.appendChild(item.value.toX3d()));
  }
  //toX3d() {
  //  let x3d;
  //  //if (this.#value.length > 0){
  //  //  this.#value.forEach(element => x3d += element.value.toX3d() + ' ');
  //  //  x3d.slice(0, -1)
  //  //}
  //  for (let i = 0; i < this.#value.length; ++i) {
  //    x3d += this.#value[i].value;
  //  }
  //  //console.log('ENCODE MF')
  //  //x3d += this.#value[0].value.toX3d();
  //  //const a = this.#value;
  //  //console.log('ENCODED MF', a[0].value)
  //  //console.log('ENCODED MF2', a[0].value.toX3d())
  //  return x3d;
  //}

  toJS() {
    throw new Error('TODO: toJS of MFNode');
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export function typeFactory(type, tokenizer) {
  switch(type) {
    case VRML.SFBool:
      return new SFBool(tokenizer);
    case VRML.SFInt32:
      return new SFInt32(tokenizer);
    case VRML.SFFloat:
      return new SFFloat(tokenizer);
    case VRML.SFString:
      return new SFString(tokenizer);
    case VRML.SFVec2f:
      return new SFVec2f(tokenizer);
    case VRML.SFVec3f:
      return new SFVec3f(tokenizer);
    case VRML.SFColor:
      return new SFColor(tokenizer);
    case VRML.SFRotation:
      return new SFRotation(tokenizer);
    case VRML.SFNode:
      return new SFNode(tokenizer);
    case VRML.MFBool:
      return new MFBool(tokenizer);
    case VRML.MFInt32:
      return new MFInt32(tokenizer);
    case VRML.MFFloat:
      return new MFFloat(tokenizer);
    case VRML.MFString:
      return new MFString(tokenizer);
    case VRML.MFVec2f:
      return new MFVec2f(tokenizer);
    case VRML.MFVec3f:
      return new MFVec3f(tokenizer);
    case VRML.MFColor:
      return new MFColor(tokenizer);
    case VRML.MFRotation:
      return new MFRotation(tokenizer);
    case VRML.MFNode:
      return new MFNode(tokenizer);
    default:
      throw new Error('Unknown type: ', type);
  }
}
