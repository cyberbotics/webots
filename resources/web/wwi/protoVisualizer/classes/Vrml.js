'use strict';

import NodeFactory from './NodeFactory.js';
import {VRML} from './constants.js';
import BaseNode from './BaseNode.js';
import ProtoNode from './ProtoNode.js';

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
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is a member of an MF*
      parentElement.setAttribute(name, this.#value);
      return;
    }

    return this.#value;
  }

  toJS() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }

  clone() {
    const copy = new SFBool();

    if (typeof this.value !== 'undefined')
      copy.value = this.#value

    return copy;
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
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is a member of an MF*
      parentElement.setAttribute(name, this.#value);
      return;
    }

    return this.#value;
  }

  toJS() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }

  clone() {
    const copy = new SFInt32();

    if (typeof this.value !== 'undefined')
      copy.value = this.#value

    return copy;
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
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is a member of an MF*
      parentElement.setAttribute(name, this.#value);
      return;
    }

    return this.#value;
  }

  toJS() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }

  clone() {
    const copy = new SFFloat();

    if (typeof this.value !== 'undefined')
      copy.value = this.#value

    return copy;
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
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is a member of an MF*
      parentElement.setAttribute(name, this.#value);
      return;
    }

    return this.#value;
  }

  toJS() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }

  clone() {
    const copy = new SFString();

    if (typeof this.value !== 'undefined')
      copy.value = this.#value;

    return copy;
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
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is a member of an MF*
      parentElement.setAttribute(name, `${this.#value.x} ${this.#value.y}`);
      return;
    }

    return `${this.#value.x} ${this.#value.y}`;
  }


  toJS() {
    return `{x: ${this.#value.x}, y: ${this.#value.y}}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.#value.x === other.value.x && this.value.y === other.value.y;
  }

  clone() {
    const copy = new SFVec2f();

    if (typeof this.value !== 'undefined')
      copy.value = JSON.parse(JSON.stringify(this.#value));

    return copy;
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
    this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat(), z: tokenizer.nextToken().toFloat()};
  }

  toX3d(name, parentElement) {
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is an item of a MF*
      parentElement.setAttribute(name, `${this.#value.x} ${this.#value.y} ${this.#value.z}`);
      return;
    }

    return `${this.#value.x} ${this.#value.y} ${this.#value.z}`;
  }

  toJS() {
    return `{x: ${this.#value.x}, y: ${this.#value.y}, z: ${this.#value.z}}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.x === other.value.x && this.value.y === other.value.y && this.value.z === other.value.z;
  }

  clone() {
    const copy = new SFVec3f();

    if (typeof this.value !== 'undefined')
      copy.value = JSON.parse(JSON.stringify(this.#value));

    return copy;
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
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is an item of a MF*
      parentElement.setAttribute(name, `${this.#value.r} ${this.#value.g} ${this.#value.b}`);
      return;
    }

    return `${this.#value.r} ${this.#value.g} ${this.#value.b}`;
  }

  toJS() {
    return `{r: ${this.#value.r}, g: ${this.#value.g}, b: ${this.#value.b}}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.r === other.value.r && this.value.g === other.value.g && this.value.b === other.value.b;
  }

  clone() {
    const copy = new SFColor();

    if (typeof this.value !== 'undefined')
      copy.value = JSON.parse(JSON.stringify(this.#value));

    return copy;
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
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is an item of a MF*
      parentElement.setAttribute(name, `${this.#value.x} ${this.#value.y} ${this.#value.z} ${this.#value.a}`);
      return;
    }

    return `${this.#value.x} ${this.#value.y} ${this.#value.z} ${this.#value.a}`;
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

  clone() {
    const copy = new SFRotation();

    if (typeof this.value !== 'undefined')
      copy.value = JSON.parse(JSON.stringify(this.#value));

    return copy;
  }
}

export class SFNode {
  #value;
  constructor(tokenizer) {
    this.isUse = false;
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
    if (tokenizer.peekWord() === 'USE')
      this.isUse = true;

    const nodeFactory = new NodeFactory();
    this.#value = nodeFactory.createNode(tokenizer);
  }

  toX3d(name, parentElement) {
    console.log('sf got', name, parentElement)
    if (typeof this.#value === 'undefined')
      return;

    const nodeX3d = this.#value.toX3d(this.isUse);

    // handle exceptions
    if (this.#value.name === 'ImageTexture') {
      nodeX3d.setAttribute('role', name.slice(0, -3)); // TODO: rename on the JS side so it matches the field name?
    }

    if (typeof nodeX3d !== 'undefined')
      parentElement.appendChild(nodeX3d);
  }

  toJS() {
    if (typeof this.#value === 'undefined')
      return;

    return this.#value.toJS();
  }

  equals(other) {
    throw new Error('TODO: equals for SFNode')
  }

  clone() {
    const copy = new SFNode();

    if (typeof this.#value !== 'undefined')
      copy.value = this.#value.clone();

    return copy
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
    if (!Array.isArray(value))
      this.#value.push(value);
    else
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

  toX3d(name, parentElement) {
    let x3d = ''
    this.#value.forEach((item) => x3d += item.toX3d() + ' ');
    parentElement.setAttribute(name, x3d.slice(0, -1));
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    throw new Error('TODO: implement equals for MFBool');
  }

  clone() {
    const copy = new MFBool();

    const v = [];
    this.#value.forEach((item) => v.push(item.clone()));
    copy.value = v;

    return copy;
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
    if (!Array.isArray(value))
      this.#value.push(value);
    else
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

  toX3d(name, parentElement) {
    let x3d = ''
    this.#value.forEach((item) => x3d += item.toX3d() + ' ');
    parentElement.setAttribute(name, x3d.slice(0, -1));
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    throw new Error('TODO: implement equals for MFInt32');
  }

  clone() {
    const copy = new MFInt32();

    const v = [];
    this.#value.forEach((item) => v.push(item.clone()));
    copy.value = v;

    return copy;
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
    if (!Array.isArray(value))
      this.#value.push(value);
    else
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

  toX3d(name, parentElement) {
    let x3d = ''
    this.#value.forEach((item) => x3d += item.toX3d() + ' ');
    parentElement.setAttribute(name, x3d.slice(0, -1));
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    throw new Error('TODO: implement equals for MFFloat');
  }

  clone() {
    const copy = new MFFloat();

    const v = [];
    this.#value.forEach((item) => v.push(item.clone()));
    copy.value = v;

    return copy;
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
    if (!Array.isArray(value))
      this.#value.push(value);
    else
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

  toX3d(name, parentElement) {
    let x3d = ''
    this.#value.forEach((item) => x3d += item.toX3d() + ' ');
    parentElement.setAttribute(name, x3d.slice(0, -1));
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    throw new Error('TODO: implement equals for MFString');
  }

  clone() {
    const copy = new MFString();

    const v = [];
    this.#value.forEach((item) => v.push(item.clone()));
    copy.value = v;

    return copy;
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
    if (!Array.isArray(value))
      this.#value.push(value);
    else
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

  toX3d(name, parentElement) {
    let x3d = ''
    this.#value.forEach((item) => x3d += item.toX3d() + ' ');
    parentElement.setAttribute(name, x3d.slice(0, -1));
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    throw new Error('TODO: implement equals for MFVec2f');
  }

  clone() {
    const copy = new MFVec2f();

    const v = [];
    this.#value.forEach((item) => v.push(item.clone()));
    copy.value = v;

    return copy;
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
    if (!Array.isArray(value))
      this.#value.push(value);
    else
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

  toX3d(name, parentElement) {
    let x3d = ''
    console.log(this.#value)
    this.#value.forEach((item) => x3d += item.toX3d() + ' ');
    parentElement.setAttribute(name, x3d.slice(0, -1));
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    throw new Error('TODO: implement equals for MFVec3f');
  }

  clone() {
    const copy = new MFVec3f();

    const v = [];
    this.#value.forEach((item) => v.push(item.clone()));
    copy.value = v;

    return copy;
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
    if (!Array.isArray(value))
      this.#value.push(value);
    else
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

  toX3d(name, parentElement) {
    let x3d = ''
    this.#value.forEach((item) => x3d += item.toX3d() + ' ');
    parentElement.setAttribute(name, x3d.slice(0, -1));
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    throw new Error('TODO: implement equals for MFColor');
  }

  clone() {
    const copy = new MFColor();

    const v = [];
    this.#value.forEach((item) => v.push(item.clone()));
    copy.value = v;

    return copy;
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
    if (!Array.isArray(value))
      this.#value.push(value);
    else
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
    let x3d = ''
    this.#value.forEach((item) => x3d += item.toX3d() + ' ');
    parentElement.setAttribute(name, x3d.slice(0, -1));
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    throw new Error('TODO: implement equals for MFRotation');
  }

  clone() {
    const copy = new MFRotation();

    const v = [];
    this.#value.forEach((item) => v.push(item.clone()));
    copy.value = v;

    return copy;
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
    if (!Array.isArray(value)) {
      if (value instanceof BaseNode || value instanceof ProtoNode) {
        // TODO: can we avoid doing this here and ensure it's sent as SFNode already?
        const sf = new SFNode();
        sf.value = value;
        this.#value.push(sf);
      } else
        this.#value.push(value);
    }
    else
      this.#value = value;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.#value.push(new SFNode(tokenizer));

      tokenizer.skipToken(']');
    } else {
      this.#value.push(new SFNode(tokenizer));

    }
  }

  toX3d(name, parentElement) {
    this.#value.forEach((item) => {
      item.toX3d(name, parentElement);
    });
  }

  toJS() {
    let js = '['
    this.#value.forEach((item) => js += item.value.toJS() + ', ');
    if (this.#value.length > 0)
      js == js.slice(0, -2)
    return  js + ']';
  }

  equals(other) {
    return this.#value === other.value;
  }

  clone() {
    const copy = new MFNode();

    const v = [];
    this.#value.forEach((item) => v.push(item.value.clone()));
    copy.value = v;

    return copy;
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
      throw new Error('Unknown VRML type: ', type);
  }
}
