'use strict';

import {VRML, vrmlTypeAsString} from './utility/utility.js';

import WbVector2 from '../../wwi/nodes/utils/WbVector2.js';
import WbVector3 from '../../wwi/nodes/utils/WbVector3.js';
import WbVector4 from '../../wwi/nodes/utils/WbVector4.js';

export default class Parameter {
  constructor(protoRef, name, type, isRegenerator, defaultValue, value) {
    this.protoRef = protoRef; // proto this parameter belongs to
    this.name = name; // name as defined in the proto header (i.e value after an IS)
    this.type = type;
    this.isTemplateRegenerator = isRegenerator;
    this.defaultValue = defaultValue;
    this.value = value;
    this.linkedProto = undefined; // SFNode fields can reference other proto files
  };

  isSFNode() {
    return this.type === VRML.SFNode;
  };

  exportVrml() {
    return 'field ' + vrmlTypeAsString(this.type) + ' ' + this.name + ' ' + this.vrmlify();
  };

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
        console.log('VV', this.value)
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
        if (typeof this.value !== 'undefined')
          console.error('TODO: implement SFNode in vrmlify.');
        return 'NULL';
      default:
        throw new Error('Unknown type \'' + this.type + '\' in x3dify.');
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
        if (typeof this.value !== 'undefined')
          console.error('TODO: implement SFNode in x3dify.');
        return;
      case VRML.MFString:
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
  };

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
