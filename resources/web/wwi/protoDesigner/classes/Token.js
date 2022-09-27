import {VRML} from './utility/utility.js';
import {FieldModel} from './FieldModel.js';

export default class Token {
  constructor(word, line, column) {
    this._word = word;
    this._line = line;
    this._column = column;

    const w0 = word[0];
    const NUMERIC_CHARS = '+-0123456789.';

    if (word === 'end of file')
      this._type = Token.TYPES.END;
    else if (word.startsWith('"') && word.endsWith('"'))
      this._type = Token.TYPES.STRING;
    else if (word.startsWith('%<') && word.endsWith('>%'))
      this._type = Token.TYPES.TEMPLATE_STATEMENT;
    else if (NUMERIC_CHARS.includes(w0))
      this._type = isNaN(Number(word)) ? Token.TYPES.INVALID : Token.TYPES.NUMERIC;
    else if (this._isKeywordType(word))
      this._type = Token.TYPES.KEYWORD;
    else if (this._isValidIdentifier(word))
      this._type = Token.TYPES.IDENTIFIER;
    else if (word.length === 1 && isPunctuation(w0))
      this._type = Token.TYPES.PUNCTUATION;
    else
      this._type = Token.TYPES.INVALID;

    this._isNode = Object.keys(FieldModel).includes(word);
  };

  word() { return this._word; };
  line() { return this._line; };
  column() { return this._column; };

  isValid() { return this._type !== Token.TYPES.INVALID; };
  isString() { return this._type === Token.TYPES.STRING; };
  isIdentifier() { return this._type === Token.TYPES.IDENTIFIER; };
  isKeyword() { return this._type === Token.TYPES.KEYWORD; };
  isNumeric() { return this._type === Token.TYPES.NUMERIC; };
  isPunctuation() { return this._type === Token.TYPES.PUNCTUATION; };
  isBoolean() { return this._type === Token.TYPES.KEYWORD && (this._word === 'TRUE' || this._word === 'FALSE'); };
  isTemplateStatement() { return this._type === Token.TYPES.TEMPLATE_STATEMENT; };
  isEof() { return this._type === Token.TYPES.END; };

  isNode() { return this._isNode; };

  toString() {
    if (!this.isString())
      throw new Error('Expected STRING type token but is not.');

    return this._word.replace('"', '');
  };

  toInt() {
    if (!this.isNumeric())
      throw new Error('Expected NUMERIC type token but is not.');

    return parseInt(this._word);
  };

  toDouble() {
    return this.toFloat();
  }

  toFloat() {
    if (!this.isNumeric())
      throw new Error('Expected NUMERIC type token but is not.');

    return parseFloat(this._word);
  };

  toBool() {
    if (!this.isBoolean())
      throw new Error('Expected boolean KEYWORD type token but is not.');

    return this._word === 'TRUE';
  };

  fieldTypeFromVrml() {
    if (this._word === 'MFBool')
      return VRML.MFBool;
    else if (this._word === 'SFBool')
      return VRML.SFBool;
    else if (this._word === 'MFColor')
      return VRML.MFColor;
    else if (this._word === 'SFColor')
      return VRML.SFColor;
    else if (this._word === 'MFFloat')
      return VRML.MFFloat;
    else if (this._word === 'SFFloat')
      return VRML.SFFloat;
    else if (this._word === 'MFInt32')
      return VRML.MFInt32;
    else if (this._word === 'SFInt32')
      return VRML.SFInt32;
    else if (this._word === 'MFNode')
      return VRML.MFNode;
    else if (this._word === 'SFNode')
      return VRML.SFNode;
    else if (this._word === 'MFRotation')
      return VRML.MFRotation;
    else if (this._word === 'SFRotation')
      return VRML.SFRotation;
    else if (this._word === 'MFString')
      return VRML.MFString;
    else if (this._word === 'SFString')
      return VRML.SFString;
    else if (this._word === 'MFVec2f')
      return VRML.MFVec2f;
    else if (this._word === 'SFVec2f')
      return VRML.SFVec2f;
    else if (this._word === 'MFVec3f')
      return VRML.MFVec3f;
    else if (this._word === 'SFVec3f')
      return VRML.SFVec3f;
  };

  _isKeywordType(word) {
    return Token.KEYWORDS.includes(word);
  };

  _isValidIdentifierChar(c, pos) {
    const v = c.charCodeAt();
    if (v <= 0x20 || v === 0x22 || v === 0x23 || v === 0x27 || v === 0x2c || v === 0x2e || v === 0x5b || v === 0x5c || v === 0x5d || v === 0x7b || v === 0x7d)
      return false;

    if (pos === 0 && v >= 0x30 && v <= 0x39)
      return false;

    return true;
  };

  _isValidIdentifier(word) {
    if (word === '')
      return false;

    for (let i = 0; i < word.length; ++i) {
      if (!this._isValidIdentifierChar(word[i], i))
        return false;
    }

    return !this._isKeywordType(word);
  };
};

function isSpace(c) {
  return c === '' || c === ' ' || c === '\n' || c === '\r' || c === '\r\n' || c === '\t' || c === ',';
};

function isPunctuation(c) {
  return '{}[]'.includes(c);
};

Token.TYPES = {
  INVALID: 0,
  STRING: 1,
  IDENTIFIER: 2,
  KEYWORD: 3,
  NUMERIC: 4,
  PUNCTUATION: 5,
  TEMPLATE_STATEMENT: 6,
  END: 7
};

Token.KEYWORDS = [
  'field',
  'vrmlField',
  'hiddenField',
  'deprecatedField',
  'DEF',
  'USE',
  'PROTO',
  'IS',
  'TRUE',
  'FALSE',
  'NULL',
  'MFBool',
  'SFBool',
  'SFColor',
  'MFColor',
  'SFFloat',
  'MFFloat',
  'SFInt32',
  'MFInt32',
  'SFNode',
  'MFNode',
  'SFRotation',
  'MFRotation',
  'SFString',
  'MFString',
  'SFVec2f',
  'MFVec2f',
  'SFVec3f',
  'MFVec3f'
];
/*
Token.SUPPORTED_NODE_LIST = [
  'Appearance',
  'Background',
  'Billboard',
  'Box',
  'Capsule',
  'Cone',
  'Cylinder',
  'DirectionalLight',
  'ElevationGrid',
  'Fog',
  'Geometry',
  'Group',
  'Image',
  'ImageTexture',
  'IndexedFaceSet',
  'IndexedLineSet',
  'Light',
  'Material',
  'PBRAppearance',
  'Plane',
  'PointLight',
  'PointSet',
  'Scene',
  'Shape',
  'Sphere',
  'SpotLight',
  'TextureTransform',
  'Transform',
  'TriangleMeshGeometry',
  'Viewpoint',
  'World'
];
*/
export {isSpace, isPunctuation};
