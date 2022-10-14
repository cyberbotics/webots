'use strict';

const VRML = {
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

class SFBool {
  constructor() {
    this.value = undefined;
  }

  setValueFromTokenizer(tokenizer) {
    this.value = tokenizer.nextToken().toBool();
  }

  toX3d() {
    return this.value;
  }

  equals(other) {
    return this.value === other;
  }
}

class SFInt32 {
  constructor() {
    this.value = undefined;
  }

  setValueFromTokenizer(tokenizer) {
    this.value = tokenizer.nextToken().toInt();
  }

  toX3d() {
    return this.value;
  }

  equals(other) {
    return this.value === other;
  }
}

class SFFloat {
  constructor() {
    this.value = undefined;
  }

  setValueFromTokenizer(tokenizer) {
    this.value = tokenizer.nextToken().toFloat();
  }

  toX3d() {
    return this.value;
  }

  equals(other) {
    return this.value === other;
  }
}

class SFString {
  constructor() {
    this.value = undefined;
  }

  setValueFromTokenizer(tokenizer) {
    this.value = tokenizer.nextWord();
  }

  toX3d() {
    return this.value;
  }

  equals(other) {
    return this.value === other;
  }
}

class SFVec2f {
  constructor() {
    this.value = undefined;
  }

  setValueFromTokenizer(tokenizer) {
    this.value = {"x": tokenizer.nextToken().toFloat(), "y": tokenizer.nextToken().toFloat()}
  }

  toX3d() {
    return this.value.x + ' ' + this.value.y;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.x === other.value.x && this.value.y === other.value.y;
  }
}

class SFVec3f {
  constructor() {
    this.value = undefined;
  }

  setValueFromTokenizer(tokenizer) {
    this.value = {
      "x": tokenizer.nextToken().toFloat(),
      "y": tokenizer.nextToken().toFloat(),
      "z": tokenizer.nextToken().toFloat()
    }
  }

  toX3d() {
    return this.value.x + ' ' + this.value.y + ' ' + this.value.z;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.x === other.value.x && this.value.y === other.value.y && this.value.z === other.value.z;
  }
}

class SFColor {
  constructor() {
    this.value = undefined;
  }

  setValueFromTokenizer(tokenizer) {
    this.value = {
      "r": tokenizer.nextToken().toFloat(),
      "g": tokenizer.nextToken().toFloat(),
      "b": tokenizer.nextToken().toFloat()
    }
  }

  toX3d() {
    return this.value.r + ' ' + this.value.g + ' ' + this.value.b;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.r === other.value.r && this.value.g === other.value.g && this.value.b === other.value.b;
  }
}

class SFNode {
  constructor() {
    this.value = undefined;
  }

  setValueFromTokenizer(tokenizer) {
  }

  toX3d() {

  }

  equals(other) {

  }
}

module.exports = { VRML, SFBool, SFInt32, SFFloat, SFString, SFVec2f, SFVec3f, SFColor, SFNode}