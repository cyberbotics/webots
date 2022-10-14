'use strict';

export const VRML = {
  MFBool: 0,
  SFBool: 1,
  MFColor: 2,
  SFColor: 3,
  MFFloat: 4,
  SFFloat: 5,
  MFInt32: 6,
  SFInt32: 7,
  MFNode: 8,
  SFNode: 9,
  MFRotation: 10,
  SFRotation: 11,
  MFString: 12,
  SFString: 13,
  MFVec2f: 14,
  SFVec2f: 15,
  MFVec3f: 16,
  SFVec3f: 17
};

export class SFBool {
  #value;

  get value() {
    return this.#value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = tokenizer.nextToken().toBool();
  }

  toX3d() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class SFInt32 {
  #value;

  get value() {
    return this.#value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = tokenizer.nextToken().toInt();
  }

  toX3d() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class SFFloat {
  #value;

  get value() {
    return this.#value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = tokenizer.nextToken().toFloat();
  }

  toX3d() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class SFString {
  #value;

  get value() {
    return this.#value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = tokenizer.nextWord();
  }

  toX3d() {
    return this.#value;
  }

  equals(other) {
    return this.#value === other.value;
  }
}

export class SFVec2f {
  #value;

  get value() {
    return this.#value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat()};
  }

  toX3d() {
    return `${this.#value.x} ${this.#value.y}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.#value.x === other.value.x && this.value.y === other.value.y;
  }
}

export class SFVec3f {
  #value;

  get value() {
    return this.#value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat(), z: tokenizer.nextToken().toFloat()};
  }

  toX3d() {
    return `${this.#value.x} ${this.#value.y} ${this.#value.z}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.x === other.value.x && this.value.y === other.value.y && this.value.z === other.value.z;
  }
}

export class SFColor {
  #value;

  get value() {
    return this.#value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = {r: tokenizer.nextToken().toFloat(), g: tokenizer.nextToken().toFloat(), b: tokenizer.nextToken().toFloat()};
  }

  toX3d() {
    return `${this.#value.r} ${this.#value.g} ${this.#value.b}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.r === other.value.r && this.value.g === other.value.g && this.value.b === other.value.b;
  }
}

export class SFNode {
  #value;

  get value() {
    return this.#value;
  }

  setValueFromTokenizer(tokenizer) {
    this.#value = createNode(tokenizer, );
  }

  toX3d() {
    if (typeof this.#value === 'undefined')
      return;

    return this.#value.toX3d()
  }

  setValue(value) {
    this.#value = value;
  }

  equals(other) {
    throw new Error('TODO: equals for SFNode')
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
    case VRML.SFNode:
      return new SFNode(tokenizer);
    default:
      throw new Error('TODO: implement other types')
  }
}
