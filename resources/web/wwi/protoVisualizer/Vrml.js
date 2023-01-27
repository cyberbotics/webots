'use strict';

import {Node} from './Node.js';
import {VRML} from './vrml_type.js';
import {FieldModel} from './FieldModel.js';

import {DOUBLE_EQUALITY_TOLERANCE} from '../nodes/utils/constants.js';
import Tokenizer from './Tokenizer.js';

class SingleValue {
  #value;
  constructor(v) {
    if (typeof v !== 'undefined') {
      if (v instanceof Tokenizer)
        this.setValueFromTokenizer(v);
      else
        this.setValueFromModel(v);
    }
  }

  get value() {
    return this.#value;
  }

  set value(v) {
    this.#value = v;
  }

  setValueFromJavaScript(value) {
    this.#value = value;
  }

  toX3d(parameterName, parentElement) {
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is a member of an MF*
      parentElement.setAttribute(parameterName, this.value);
      return;
    }

    return this.value;
  }

  // used to encode the fields in a format understandable by the template engine
  toJS() {
    return this.value;
  }

  // used to encode commands that need to be sent to WebotsJS
  toJson() {
    return `${this.value}`;
  }

  toVrml() {
    return this.value;
  }

  equals(other) {
    return this.value === other.value;
  }
}

export class SFBool extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    super.value = tokenizer.nextToken().toBool();
  }

  setValueFromModel(v) {
    super.value = v
  }

  type() {
    return VRML.SFBool;
  }

  clone() {
    const copy = new SFBool();

    if (typeof this.value !== 'undefined')
      copy.value = this.value;

    return copy;
  }
}

export class SFInt32 extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    super.value = tokenizer.nextToken().toInt();
  }

  setValueFromModel(v) {
    super.value = v;
  }

  type() {
    return VRML.SFInt32;
  }

  clone() {
    const copy = new SFInt32();

    if (typeof this.value !== 'undefined')
      copy.value = this.value;

    return copy;
  }
}

export class SFFloat extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    this.value = tokenizer.nextToken().toFloat();
  }

  setValueFromModel(v) {
    super.value = v;
  }

  type() {
    return VRML.SFFloat;
  }

  clone() {
    const copy = new SFFloat();

    if (typeof this.value !== 'undefined')
      copy.value = this.value;

    return copy;
  }
}

export class SFString extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    this.value = tokenizer.nextWord();
  }

  setValueFromModel(v) {
    super.value = v;
  }

  setValueFromJavaScript(v) {
    if (!v.startsWith('"') && !v.endsWith('"'))
      this.value = '"' + v + '"';
    else
      this.value = v;
  }

  toJson(parameterName) {
    return this.value;
  }

  type() {
    return VRML.SFString;
  }

  clone() {
    const copy = new SFString();

    if (typeof this.value !== 'undefined')
      copy.value = this.value;

    return copy;
  }
}

export class SFVec2f extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    this.value = {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat()};
  }

  setValueFromModel(v) {
    super.value = v;
  }

  toX3d(parameterName, parentElement) {
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is a member of an MF*
      parentElement.setAttribute(parameterName, `${this.value.x} ${this.value.y}`);
      return;
    }

    return `${this.value.x} ${this.value.y}`;
  }

  toJS(stringified = true) {
    const js = `{"x": ${this.value.x}, "y": ${this.value.y}}`;
    return stringified ? js : JSON.parse(js);
  }

  toJson() {
    return `${this.value.x} ${this.value.y}`;
  }

  toVrml() {
    return `${this.value.x} ${this.value.y}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.x === other.value.x && this.value.y === other.value.y;
  }

  type() {
    return VRML.SFVec2f;
  }

  clone() {
    const copy = new SFVec2f();

    if (typeof this.value !== 'undefined')
      copy.value = JSON.parse(JSON.stringify(this.value));

    return copy;
  }
}

export class SFVec3f extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    this.value = {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat(), z: tokenizer.nextToken().toFloat()};
  }

  setValueFromModel(v) {
    super.value = v;
  }

  toX3d(parameterName, parentElement) {
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is an item of a MF*
      parentElement.setAttribute(parameterName, `${this.value.x} ${this.value.y} ${this.value.z}`);
      return;
    }

    return `${this.value.x} ${this.value.y} ${this.value.z}`;
  }

  toJS(stringified = true) {
    const js = `{"x": ${this.value.x}, "y": ${this.value.y}, "z": ${this.value.z}}`;
    return stringified ? js : JSON.parse(js);
  }

  toJson() {
    return `${this.value.x} ${this.value.y} ${this.value.z}`;
  }

  toVrml() {
    return `${this.value.x} ${this.value.y} ${this.value.z}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      return false;

    return this.value.x === other.value.x && this.value.y === other.value.y && this.value.z === other.value.z;
  }

  type() {
    return VRML.SFVec3f;
  }

  clone() {
    const copy = new SFVec3f();

    if (typeof this.value !== 'undefined')
      copy.value = JSON.parse(JSON.stringify(this.value));

    return copy;
  }
}

export class SFColor extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    this.value = {r: tokenizer.nextToken().toFloat(), g: tokenizer.nextToken().toFloat(), b: tokenizer.nextToken().toFloat()};
  }

  setValueFromModel(v) {
    super.value = v;
  }

  toX3d(parameterName, parentElement) {
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is an item of a MF*
      parentElement.setAttribute(parameterName, `${this.value.r} ${this.value.g} ${this.value.b}`);
      return;
    }

    return `${this.value.r} ${this.value.g} ${this.value.b}`;
  }

  toJS(stringified = true) {
    const js = `{"r": ${this.value.r}, "g": ${this.value.g}, "b": ${this.value.b}}`;
    return stringified ? js : JSON.parse(js);
  }

  toJson() {
    return `${this.value.r} ${this.value.g} ${this.value.b}`;
  }

  toVrml() {
    return `${this.value.r} ${this.value.g} ${this.value.b}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      throw new Error('Values should be defined for them to be compared.');

    return this.value.r === other.value.r && this.value.g === other.value.g && this.value.b === other.value.b;
  }

  type() {
    return VRML.SFColor;
  }

  clone() {
    const copy = new SFColor();

    if (typeof this.value !== 'undefined')
      copy.value = JSON.parse(JSON.stringify(this.value));

    return copy;
  }
}

export class SFRotation extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    let xValue = tokenizer.nextToken().toFloat();
    let yValue = tokenizer.nextToken().toFloat();
    let zValue = tokenizer.nextToken().toFloat();
    let aValue = tokenizer.nextToken().toFloat();

    this.normalize(xValue, yValue, zValue, aValue);
  }

  setValueFromModel(v) {
    super.value = v;
  }

  setValueFromJavaScript(v) {
    if (isNaN(v.x))
      v.x = 0;
    if (isNaN(v.y))
      v.y = 0;
    if (isNaN(v.z))
      v.z = 0;
    if (isNaN(v.a))
      v.a = 0;

    this.normalize(v.x, v.y, v.z, v.a);
  }

  toX3d(parameterName, parentElement) {
    if (typeof parentElement !== 'undefined') { // this is the case if this instance is an item of a MF*
      parentElement.setAttribute(parameterName, `${this.value.x} ${this.value.y} ${this.value.z} ${this.value.a}`);
      return;
    }

    return `${this.value.x} ${this.value.y} ${this.value.z} ${this.value.a}`;
  }

  normalize(xValue, yValue, zValue, aValue) {
    const invl = 1.0 / Math.sqrt(xValue * xValue + yValue * yValue + zValue * zValue);
    if (Math.abs(invl - 1.0) > DOUBLE_EQUALITY_TOLERANCE) {
      xValue *= invl;
      yValue *= invl;
      zValue *= invl;
    }

    this.value = {x: xValue, y: yValue, z: zValue, a: aValue};
  }

  toJS(stringified = true) {
    const js = `{"x": ${this.value.x}, "y": ${this.value.y}, "z": ${this.value.z}, "a": ${this.value.a}}`;
    return stringified ? js : JSON.parse(js);
  }

  toJson() {
    return `${this.value.x} ${this.value.y} ${this.value.z} ${this.value.a}`;
  }

  toVrml() {
    return `${this.value.x} ${this.value.y} ${this.value.z} ${this.value.a}`;
  }

  equals(other) {
    if (typeof this.value === 'undefined' || typeof other.value === 'undefined')
      throw new Error('Values should be defined for them to be compared.');

    return this.value.x === other.value.x && this.value.y === other.value.y &&
      this.value.z === other.value.z && this.value.a === other.value.a;
  }

  type() {
    return VRML.SFRotation;
  }

  clone() {
    const copy = new SFRotation();

    if (typeof this.value !== 'undefined')
      copy.value = JSON.parse(JSON.stringify(this.value));

    return copy;
  }
}

export class SFNode extends SingleValue {
  setValueFromTokenizer(tokenizer) {
    console.log('setValueFromTokenizer with tokens:');
    tokenizer.printTokens();
    if (tokenizer.peekWord() === 'USE') {
      this.isUse = true;
      tokenizer.skipToken('USE');
      const useName = tokenizer.nextWord();
      if (!tokenizer.proto.def.has(useName))
        throw new Error('No DEF name ' + useName + ' found in PROTO:' + tokenizer.proto.name);

      this.value = tokenizer.proto.def.get(useName);
      return;
    }

    let defName;
    if (tokenizer.peekWord() === 'DEF') {
      tokenizer.skipToken('DEF');
      defName = tokenizer.nextWord();
    } else if (tokenizer.peekWord() === 'NULL') {
      this.value = null;
      return;
    }

    let url;
    if (tokenizer.externProto.has(tokenizer.peekWord()))
      url = tokenizer.externProto.get(tokenizer.nextWord());
    else
      url = tokenizer.nextWord();

    console.log('create node in sfnode parameter:', url);
    this.value = new Node(url, tokenizer);

    if (this.value.isProto) {
    //  //throw new Error('REACHED, NEED TO DO SOMETHING?')
    //this.value.configureParametersFromTokenizer(tokenizer);
    } else {
      this.value.configureFieldsFromTokenizer(tokenizer);
    }

    if (typeof defName !== 'undefined')
      tokenizer.proto.def.set(defName, this.value);
  }

  setValueFromModel(v) {
    if (typeof v === 'undefined')
      throw new Error('Cannot initialize undefined VRML type.')

    if (v === null)
      this.value = v;
    else
      this.value = new Node(v);
  }

  setValueFromJavaScript(value) {
    this.value = value;
  }

  toX3d(parameterName, parentElement) {
    if (this.value === null)
      return;

    const nodeX3d = this.value.toX3d(this.isUse, parameterName);

    // handle exceptions
    if (this.value.name === 'ImageTexture')
      nodeX3d.setAttribute('role', parameterName.slice(0, -3)); // TODO: rename on the JS side so it matches the field name?
    else if (['Shape', 'Group', 'Transform', 'Solid', 'Robot'].includes(this.value.name)) {
      if (parameterName === 'boundingObject')
        nodeX3d.setAttribute('role', 'boundingObject');
    } else if (['BallJointParameters', 'JointParameters', 'HingeJointParameters'].includes(this.value.name))
      nodeX3d.setAttribute('role', parameterName); // identifies which jointParameter slot the node belongs to
    else if (['Brake', 'PositionSensor', 'RotationalMotor', 'LinearMotor'].includes(this.value.name))
      nodeX3d.setAttribute('role', parameterName); // identifies which device slot the node belongs to

    if (typeof nodeX3d !== 'undefined')
      parentElement.appendChild(nodeX3d);
  }

  toJS() {
    if (typeof this.value === 'undefined')
      throw new Error('When exporting to JavaScript, the field should always be defined (or null).');

    if (this.value === null)
      return;

    return this.value.toJS();
  }

  toJson() {
    throw new Error('SFNodes should not be encoded as strings, the x3d needs to be sent instead.');
  }

  toVrml() {
    if (this.value === null)
      return 'NULL';

    if (typeof this.value === 'undefined')
      throw new Error('When exporting to VRML, the field should always be defined (or null).');

    return this.value.toVrml();
  }

  equals(other) {
    if (this.value === null && other.value === null)
      return true;

    if ((this.value === null && other.value !== null) || (this.value !== null && other.value === null))
      return false;

    if (this.value.url !== other.value.url)
      return false;

    if (this.value.parameters.size !== other.value.parameters.size)
      return false;

    for (const [parameterName, parameter] of this.value.parameters) {
      if (!other.value.parameters.has(parameterName))
        return false;

      const otherParameter = other.value.parameters.get(parameterName);
      if (!parameter.value.equals(otherParameter.value))
        return false;
      if (!parameter.defaultValue.equals(otherParameter.defaultValue))
        return false;
    }

    return true;
  }

  type() {
    return VRML.SFNode;
  }

  clone() {
    const copy = new SFNode();

    if (typeof this.value !== 'undefined') {
      if (this.value === null) {
        copy.value = null; // necessary since being null and being undefined have different meanings in this context
        return copy;
      }

      copy.value = this.value.clone();
    }

    return copy;
  }
}

class MultipleValue {
  #value = [];
  constructor(v) {
    if (typeof v === 'undefined')
      throw new Error('Cannot initialize undefined VRML type.')

    if (v instanceof Tokenizer)
      this.setValueFromTokenizer(v);
    else
      this.setValueFromModel(v);
  }

  get value() {
    return this.#value;
  }

  set value(v) {
    if (!Array.isArray(v))
      this.#value.push(v);
    else
      this.#value = v;
  }

  insert(item) {
    this.#value.push(item);
  }

  at(index) {
    if (typeof this.#value === 'undefined')
      throw new Error('Value is not defined.');

    if (index >= this.#value.length)
      throw new Error('Attempting to access item out of bounds.');

    return this.#value[index];
  }

  toX3d(parameterName, parentElement) {
    let x3d = '';
    this.#value.forEach((item) => { x3d += item.toX3d() + ' '; });
    parentElement.setAttribute(parameterName, x3d.slice(0, -1));
  }

  toJS() {
    let js = '[';
    this.#value.forEach((item) => { js += item.toJS() + ', '; });
    if (this.#value.length > 0)
      js = js.slice(0, -2);
    return js + ']';
  }

  toJson() {
    let wjs = '[';
    this.#value.forEach((item) => { wjs += item.toJson() + ', '; });
    if (this.#value.length > 0)
      wjs = wjs.slice(0, -2);
    return wjs + ']';
  }

  toVrml() {
    let vrml = '[';
    this.#value.forEach((item) => { vrml += item.toVrml() + ', '; });
    if (this.#value.length > 0)
      vrml = vrml.slice(0, -2);
    return vrml + ']';
  }

  equals(other) {
    if (typeof this.#value === 'undefined' || typeof other.value === 'undefined')
      throw new Error('Values should be defined for them to be compared.');

    if (this.#value.length !== other.value.length)
      return false;

    let isEqual = true;
    this.#value.forEach((item, index) => { isEqual = isEqual && item.equals(other.at(index)); });
    return isEqual;
  }
}

export class MFBool extends MultipleValue {
  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFBool(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFBool(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFBool(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sfbool = new SFBool();
      sfbool.setValueFromJavaScript(item);
      this.insert(sfbool);
    });
  }

  type() {
    return VRML.MFBool;
  }

  clone() {
    const copy = new MFBool();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export class MFInt32 extends MultipleValue {
  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFInt32(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFInt32(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFInt32(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sfint32 = new SFInt32();
      sfint32.setValueFromJavaScript(parseInt(item));
      this.insert(sfint32);
    });
  }

  type() {
    return VRML.MFInt32;
  }

  clone() {
    const copy = new MFInt32();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export class MFFloat extends MultipleValue {
  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFFloat(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFFloat(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFFloat(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sffloat = new SFFloat();
      sffloat.setValueFromJavaScript(parseFloat(item));
      this.insert(sffloat);
    });
  }

  type() {
    return VRML.MFFloat;
  }

  clone() {
    const copy = new MFFloat();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export class MFString extends MultipleValue {
  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFString(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFString(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFString(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sfstring = new SFString();
      sfstring.setValueFromJavaScript(item);
      this.insert(sfstring);
    });
  }

  type() {
    return VRML.MFString;
  }

  clone() {
    const copy = new MFString();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export class MFVec2f extends MultipleValue {
  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFVec2f(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFVec2f(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFVec2f(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sfvec2f = new SFVec2f();
      sfvec2f.setValueFromJavaScript(item);
      this.insert(sfvec2f);
    });
  }

  type() {
    return VRML.MFVec2f;
  }

  clone() {
    const copy = new MFVec2f();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export class MFVec3f extends MultipleValue {
  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFVec3f(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFVec3f(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFVec3f(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sfvec3f = new SFVec3f();
      sfvec3f.setValueFromJavaScript(item);
      this.insert(sfvec3f);
    });
  }

  type() {
    return VRML.MFVec3f;
  }

  clone() {
    const copy = new MFVec3f();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export class MFColor extends MultipleValue {
  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFColor(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFColor(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFColor(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sfcolor = new SFColor();
      sfcolor.setValueFromJavaScript(item);
      this.insert(sfcolor);
    });
  }

  type() {
    return VRML.MFColor;
  }

  clone() {
    const copy = new MFColor();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export class MFRotation extends MultipleValue {
  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFRotation(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFRotation(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFRotation(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sfrotation = new SFRotation();
      sfrotation.setValueFromJavaScript(item);
      this.insert(sfrotation);
    });
  }

  type() {
    return VRML.MFRotation;
  }

  clone() {
    const copy = new MFRotation();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export class MFNode extends MultipleValue {
  get value() {
    return super.value;
  }

  set value(v) {
    if (!Array.isArray(v)) {
      if (v instanceof Node) {
        // TODO: can we avoid doing this here and ensure it's sent as SFNode already?
        // TODO: still needed?
        const sf = new SFNode();
        sf.value = v;
        this.insert(sf);
      } else
        this.insert(v);
    } else
      super.value = v;
  }

  setValueFromTokenizer(tokenizer) {
    if (tokenizer.peekWord() === '[') {
      tokenizer.skipToken('[');

      while (tokenizer.peekWord() !== ']')
        this.insert(new SFNode(tokenizer));

      tokenizer.skipToken(']');
    } else
      this.insert(new SFNode(tokenizer));
  }

  setValueFromModel(v) {
    this.value = [];
    v.forEach((item) => this.value.push(new SFNode(item)));
  }

  setValueFromJavaScript(items) {
    this.value = [];
    items.forEach((item) => {
      const sfnode = new SFNode();
      sfnode.setValueFromJavaScript(item);
      this.insert(sfnode);
    });
  }

  insertNode(node, index) {
    const sfnode = new SFNode();
    sfnode.setValueFromJavaScript(node);
    this.value.splice(index, 0, sfnode);
  }

  removeNode(index) {
    if (index > this.value.length - 1)
      throw new Error('Node at index ' + index + ' cannot be removed because out of bounds.');

    this.value.splice(index, 1);
  }

  toX3d(parameterName, parentElement) {
    if (this.value === [])
      return;

    this.value.forEach((item) => item.toX3d(parameterName, parentElement));
  }

  toJS() {
    let js = '[';
    this.value.forEach((item) => { js += item.value.toJS() + ', '; });
    if (this.value.length > 0)
      js = js.slice(0, -2);
    return js + ']';
  }

  toJson() {
    throw new Error('MFNodes should not be encoded as strings, the x3d needs to be sent instead.');
  }

  type() {
    return VRML.MFNode;
  }

  clone() {
    const copy = new MFNode();
    this.value.forEach((item) => copy.insert(item.clone()));
    return copy;
  }
}

export function vrmlFactory(type, value) {
  switch (type) {
    case VRML.SFBool:
      return new SFBool(value);
    case VRML.SFInt32:
      return new SFInt32(value);
    case VRML.SFFloat:
      return new SFFloat(value);
    case VRML.SFString:
      return new SFString(value);
    case VRML.SFVec2f:
      return new SFVec2f(value);
    case VRML.SFVec3f:
      return new SFVec3f(value);
    case VRML.SFColor:
      return new SFColor(value);
    case VRML.SFRotation:
      return new SFRotation(value);
    case VRML.SFNode:
      return new SFNode(value);
    case VRML.MFBool:
      return new MFBool(value);
    case VRML.MFInt32:
      return new MFInt32(value);
    case VRML.MFFloat:
      return new MFFloat(value);
    case VRML.MFString:
      return new MFString(value);
    case VRML.MFVec2f:
      return new MFVec2f(value);
    case VRML.MFVec3f:
      return new MFVec3f(value);
    case VRML.MFColor:
      return new MFColor(value);
    case VRML.MFRotation:
      return new MFRotation(value);
    case VRML.MFNode:
      return new MFNode(value);
    default:
      throw new Error('Unknown VRML type: ', type);
  }
}

export function stringifyType(type) {
  switch (type) {
    case VRML.SFBool:
      return 'SFBool';
    case VRML.SFInt32:
      return 'SFInt32';
    case VRML.SFFloat:
      return 'SFFloat';
    case VRML.SFString:
      return 'SFString';
    case VRML.SFVec2f:
      return 'SFVec2f';
    case VRML.SFVec3f:
      return 'SFVec3f';
    case VRML.SFColor:
      return 'SFColor';
    case VRML.SFRotation:
      return 'SFRotation';
    case VRML.SFNode:
      return 'SFNode';
    case VRML.MFBool:
      return 'MFBool';
    case VRML.MFInt32:
      return 'MFInt32';
    case VRML.MFFloat:
      return 'MFFloat';
    case VRML.MFString:
      return 'MFString';
    case VRML.MFVec2f:
      return 'MFVec2f';
    case VRML.MFVec3f:
      return 'MFVec3f';
    case VRML.MFColor:
      return 'MFColor';
    case VRML.MFRotation:
      return 'MFRotation';
    case VRML.MFNode:
      return 'MFNode';
    default:
      throw new Error('Unknown VRML type: ', type);
  }
}

export function jsifyFromTokenizer(type, tokenizer) {

  switch (type) {
    case VRML.SFBool:
      return tokenizer.nextToken().toBool();
    case VRML.SFInt32:
      return tokenizer.nextToken().toInt();
    case VRML.SFFloat:
      return tokenizer.nextToken().toFloat();
    case VRML.SFString: {
      const value = tokenizer.nextWord();
      if (!value.startsWith('"') && !value.endsWith('"'))
        return '"' + value + '"';
      return value;
    }
    case VRML.SFVec2f:
      return {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat()};
    case VRML.SFVec3f:
      return {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat(), z: tokenizer.nextToken().toFloat()};
    case VRML.SFColor:
      return {r: tokenizer.nextToken().toFloat(), g: tokenizer.nextToken().toFloat(), b: tokenizer.nextToken().toFloat()};
    case VRML.SFRotation:
      return {x: tokenizer.nextToken().toFloat(), y: tokenizer.nextToken().toFloat(), z: tokenizer.nextToken().toFloat(), a: tokenizer.nextToken().toFloat()};
    case VRML.SFNode:
      return tokenizer.spliceTokenizerByType(VRML.SFNode);
    case VRML.MFBool:
    case VRML.MFInt32:
    case VRML.MFFloat:
    case VRML.MFString:
    case VRML.MFVec2f:
    case VRML.MFVec3f:
    case VRML.MFColor:
    case VRML.MFRotation:
    case VRML.MFNode: {
      const value = [];
      if (tokenizer.peekWord() === '[') {
        tokenizer.skipToken('[');

        while (tokenizer.peekWord() !== ']')
          value.push(jsifyFromTokenizer(type + 1, tokenizer))

        tokenizer.skipToken(']');
      } else
        value.push(jsifyFromTokenizer(type + 1, tokenizer));

      return value;
    }
    default:
      throw new Error('Unknown VRML type: ', type);
  }
}