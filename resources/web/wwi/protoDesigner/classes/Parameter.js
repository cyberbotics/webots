'use strict';

import {VRML, vrmlTypeAsString} from './utility/utility.js';

import WbVector2 from '../../nodes/utils/WbVector2.js';
import WbVector3 from '../../nodes/utils/WbVector3.js';
import WbVector4 from '../../nodes/utils/WbVector4.js';

export default class Parameter {
  constructor(protoRef, id, name, type, isRegenerator, defaultValue, value) {
    this.protoRef = protoRef; // proto this parameter belongs to
    this.id = id;
    this.name = name; // name as defined in the proto header (i.e value after an IS)
    this.type = type;
    this.isTemplateRegenerator = isRegenerator;
    this.defaultValue = defaultValue;
    this.value = value;
    this.xml = document.implementation.createDocument('', '', null);
  }

  isSFNode() {
    return this.type === VRML.SFNode;
  }

  exportVrmlHeader() {
    return 'field ' + vrmlTypeAsString(this.type) + ' ' + this.name + ' ' + this.vrmlify() + '\n';
  }

  isDefaultValue() {
    switch (this.type) {
      case VRML.SFBool:
      case VRML.SFFloat:
      case VRML.SFInt32:
      case VRML.SFString:
        return this.value.valueOf() === this.defaultValue.valueOf();
      case VRML.SFVec2f:
      case VRML.SFVec3f:
      case VRML.SFColor:
      case VRML.SFRotation:
        return this.value.equal(this.defaultValue);
      case VRML.SFNode:
        return JSON.stringify(this.value) === JSON.stringify(this.defaultValue);
      case VRML.MFBool:
      case VRML.MFFloat:
      case VRML.MFInt32:
      case VRML.MFString:
        if (this.value.length !== this.defaultValue.length)
          return false;
        for (let i = 0; i < this.value.length; ++i) {
          if (this.value.valueOf() !== this.defaultValue.valueOf())
            return false;
        }
        return true;
      case VRML.MFVec2f:
      case VRML.MFVec3f:
      case VRML.MFColor:
      case VRML.MFRotation:
        if (this.value.length !== this.defaultValue.length)
          return false;
        for (let i = 0; i < this.value.length; ++i) {
          if (!this.value.equal(this.defaultValue))
            return false;
        }
        return true;
      case VRML.MFNode:
        if (this.value.length !== this.defaultValue.length)
          return false;
        for (let i = 0; i < this.value.length; ++i) {
          if (JSON.stringify(this.value) !== JSON.stringify(this.defaultValue))
            return false;
        }
        return true;
      default:
        throw new Error('Cannot determine if parameter value was change, unknown type ' + this.type);
    }
  }

  setValueFromString(value) {
    switch (this.type) {
      case VRML.SFBool:
        this.value = value === 'true';
        break;
      case VRML.SFFloat:
        this.value = parseFloat(value);
        break;
      case VRML.SFInt32:
        this.value = parseInt(value);
        break;
      case VRML.SFString:
        this.value = value;
        break;
      case VRML.SFVec2f:
        const v2 = value.split(/\s/);
        this.value = new WbVector2(parseFloat(v2[0]), parseFloat(v2[1]));
        break;
      case VRML.SFVec3f:
      case VRML.SFColor:
        const v3 = value.split(/\s/);
        this.value = new WbVector3(parseFloat(v3[0]), parseFloat(v3[1]), parseFloat(v3[2]));
        break;
      case VRML.SFRotation:
        const v4 = value.split(/\s/);
        this.value = new WbVector4(parseFloat(v4[0]), parseFloat(v4[1]), parseFloat(v4[2]), parseFloat(v4[3]));
        break;
      case VRML.SFNode:
        console.error('Attempting to set SFNode from string, but not yet implemented.');
        break;
      case VRML.MFString:
        this.value = value.split(' ');
        break;
      default:
        throw new Error('Unknown type \'' + this.type + '\' in setValueFromString.');
    }
  }

  vrmlify() {
    switch (this.type) {
      case VRML.SFBool:
        return this.value.toString().toUpperCase();
      case VRML.SFFloat:
      case VRML.SFInt32:
        return this.value.toString();
      case VRML.SFString:
        return this.value;
      case VRML.SFVec2f:
        return this.value.x + ' ' + this.value.y;
      case VRML.SFVec3f:
      case VRML.SFColor:
        return this.value.x + ' ' + this.value.y + ' ' + this.value.z;
      case VRML.SFRotation:
        return this.value.x + ' ' + this.value.y + ' ' + this.value.z + ' ' + this.value.w;
      case VRML.SFNode:
        if (typeof this.value === 'undefined')
          return 'NULL';
        else
          console.warning('Pre-defined SFNodes should not be exported using vrmlify.');
        break;
      case VRML.MFBool:
        let mfb = '[';
        for (let i = 0; i < this.value.length; ++i)
          mfb += this.value[i].toString().toUpperCase() + ', ';
        if (mfb.length > 1)
          mfb = mfb.slice(0, -2);
        return mfb + ']';
      case VRML.MFInt32:
      case VRML.MFString:
        let mfs = '[';
        for (let i = 0; i < this.value.length; ++i)
          mfs += this.value[i] + ', ';
        if (mfs.length > 1)
          mfs = mfs.slice(0, -2);
        return mfs + ']';
      case VRML.MFNode:
        if (this.value.length === 0)
          return '[]';
        else
          console.warn('Pre-defined MFNodes should not be exported using vrmlify.');
        break;
      default:
        throw new Error('Unknown type \'' + this.type + '\' in vrmlify.');
    }
  }

  x3dify() {
    switch (this.type) {
      case VRML.SFBool:
      case VRML.SFFloat:
      case VRML.SFInt32:
      case VRML.SFString:
        return this.value;
      case VRML.SFVec2f:
        return this.value.x + ' ' + this.value.y;
      case VRML.SFVec3f:
      case VRML.SFColor:
        return this.value.x + ' ' + this.value.y + ' ' + this.value.z;
      case VRML.SFRotation:
        return this.value.x + ' ' + this.value.y + ' ' + this.value.z + ' ' + this.value.w;
      case VRML.SFNode:
        if (typeof this.value === 'undefined')
          return;

        // let x3d
        const topElement = this.xml.createElement(this.value[0]);
        switch (this.role) {
          case 'baseColorMap':
            topElement.setAttribute('role', 'baseColor');
            break;
          case 'roughnessMap':
            topElement.setAttribute('role', 'roughness');
            break;
          case 'metalnessMap':
            topElement.setAttribute('role', 'metalness');
            break;
          case 'normalMap':
            topElement.setAttribute('role', 'normal');
            break;
          case 'occlusionMap':
            topElement.setAttribute('role', 'occlusion');
            break;
          case 'emissiveColorMap':
            topElement.setAttribute('role', 'emissiveColor');
        }

        let currentElement = topElement;
        let pendingAttribute;
        let pendingArray = [];
        let step = 1;
        // -1 because we know that it must end with '}'
        for (let i = 2; i < this.value.length - 1; i += step) {
          const word = this.value[i];
          const nextWord = this.value[i + 1];
          if (nextWord === '{') {
            let newElement = this.xml.createElement(word);
            currentElement.appendChild(newElement);
            currentElement = newElement;
            step = 2;
          } else if (nextWord === '[') { // works only if there is no possible nested array
            pendingAttribute = word;
            step = 2;
          } else {
            if (typeof pendingAttribute === 'undefined') {
              currentElement.setAttribute(word, nextWord);
              step = 2;
            } else {
              pendingArray.push(word);
              if (nextWord === ']') {
                currentElement.setAttribute(pendingAttribute, pendingArray.join(' '));
                pendingAttribute = undefined;
                pendingArray = [];
                step = 2;
              } else
                step = 1;
            }
          }
          if (word === '}' || nextWord === '}')
            currentElement = currentElement.parentNode;
        }

        return topElement;
      case VRML.MFString:
      case VRML.MFInt32:
      case VRML.MFFloat:
      case VRML.MFNode:
        if (!Array.isArray(this.value))
          console.error('Expected an array, but value is not. Is it normal?');
        let s = '';
        for (let i = 0; i < this.value.length; ++i)
          s += this.value[i] + ' ';
        s = s.slice(0, -1);
        return s;
      default:
        throw new Error('Unknown type \'' + this.type + '\' in x3dify.');
    }
  }

  jsify(isColor = false) { // encodes field values in a format compliant for the template engine VRLM generation
    return '{value: ' + this._jsifyVariable(this.value, isColor) + ', defaultValue: ' + this._jsifyVariable(this.defaultValue, isColor) + '}';
  }

  _jsifyVariable(variable, isColor) {
    switch (this.type) {
      case VRML.SFBool:
      case VRML.SFFloat:
      case VRML.SFInt32:
      case VRML.SFString: // note: when parsing SFStrings the quotation marks are kept, so no need to add it here
        return variable;
      case VRML.SFVec2f:
        return '{x: ' + variable.x + ', y: ' + variable.y + '}';
      case VRML.SFVec3f:
      case VRML.SFColor:
        if (isColor)
          return '{r: ' + variable.x + ', g: ' + variable.y + ', b: ' + variable.z + '}';
        return '{x: ' + variable.x + ', y: ' + variable.y + ', z: ' + variable.z + '}';
      case VRML.SFRotation:
        return '{x: ' + variable.x + ', y: ' + variable.y + ', z: ' + variable.z + ', w: ' + variable.w + '}';
      case VRML.SFNode:
        if (typeof variable !== 'undefined')
          console.error('TODO: implement SFNode in _jsifyVariable.');
        return;
      case VRML.MFString:
        let a = '[';
        for (let i = 0; i < variable.length; ++i)
          a += '\'' + variable[i] + '\', ';
        if (a.length > 2)
          a = a.slice(0, -2);
        a += ']';
        return a;
      default:
        throw new Error('Unknown type \'' + this.type + '\' in _jsifyVariable.');
    }
  }
};
