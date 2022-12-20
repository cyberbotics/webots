import {VRML} from './vrml_type.js';

export default class Token {
  #column;
  #line;
  #word;
  constructor(word, line, column) {
    this.#word = word;
    this.#line = line;
    this.#column = column;

    const w0 = word[0];
    const NUMERIC_CHARS = '+-0123456789.';

    if (word === 'end of file')
      this.type = Token.TYPES.END;
    else if (word.startsWith('"') && word.endsWith('"'))
      this.type = Token.TYPES.STRING;
    else if (word.startsWith('%<') && word.endsWith('>%'))
      this.type = Token.TYPES.TEMPLATE_STATEMENT;
    else if (NUMERIC_CHARS.includes(w0))
      this.type = isNaN(Number(word)) ? Token.TYPES.INVALID : Token.TYPES.NUMERIC;
    else if (this.#isKeywordType(word))
      this.type = Token.TYPES.KEYWORD;
    else if (this.#isValidIdentifier(word))
      this.type = Token.TYPES.IDENTIFIER;
    else if (word.length === 1 && isPunctuation(w0))
      this.type = Token.TYPES.PUNCTUATION;
    else
      this.type = Token.TYPES.INVALID;
  };

  word() { return this.#word; };
  line() { return this.#line; };
  column() { return this.#column; };

  isValid() { return this.type !== Token.TYPES.INVALID; };
  isString() { return this.type === Token.TYPES.STRING; };
  isIdentifier() { return this.type === Token.TYPES.IDENTIFIER; };
  isKeyword() { return this.type === Token.TYPES.KEYWORD; };
  isNumeric() { return this.type === Token.TYPES.NUMERIC; };
  isPunctuation() { return this.type === Token.TYPES.PUNCTUATION; };
  isBoolean() { return this.type === Token.TYPES.KEYWORD && (this.#word === 'TRUE' || this.#word === 'FALSE'); };
  isTemplateStatement() { return this.type === Token.TYPES.TEMPLATE_STATEMENT; };
  isEof() { return this.type === Token.TYPES.END; };

  toString() {
    if (!this.isString())
      throw new Error('Expected STRING type token but is not.');

    return this.#word.replace('"', '');
  };

  toInt() {
    if (!this.isNumeric())
      throw new Error('Expected NUMERIC type token but is not.');

    return parseInt(this.#word);
  };

  toDouble() {
    return this.toFloat();
  }

  toFloat() {
    if (!this.isNumeric())
      throw new Error('Expected NUMERIC type token but is not.');

    return parseFloat(this.#word);
  };

  toBool() {
    if (!this.isBoolean())
      throw new Error('Expected boolean KEYWORD type token but is not.');

    return this.#word === 'TRUE';
  };

  fieldTypeFromVrml() {
    if (this.#word === 'MFBool')
      return VRML.MFBool;
    else if (this.#word === 'SFBool')
      return VRML.SFBool;
    else if (this.#word === 'MFColor')
      return VRML.MFColor;
    else if (this.#word === 'SFColor')
      return VRML.SFColor;
    else if (this.#word === 'MFFloat')
      return VRML.MFFloat;
    else if (this.#word === 'SFFloat')
      return VRML.SFFloat;
    else if (this.#word === 'MFInt32')
      return VRML.MFInt32;
    else if (this.#word === 'SFInt32')
      return VRML.SFInt32;
    else if (this.#word === 'MFNode')
      return VRML.MFNode;
    else if (this.#word === 'SFNode')
      return VRML.SFNode;
    else if (this.#word === 'MFRotation')
      return VRML.MFRotation;
    else if (this.#word === 'SFRotation')
      return VRML.SFRotation;
    else if (this.#word === 'MFString')
      return VRML.MFString;
    else if (this.#word === 'SFString')
      return VRML.SFString;
    else if (this.#word === 'MFVec2f')
      return VRML.MFVec2f;
    else if (this.#word === 'SFVec2f')
      return VRML.SFVec2f;
    else if (this.#word === 'MFVec3f')
      return VRML.MFVec3f;
    else if (this.#word === 'SFVec3f')
      return VRML.SFVec3f;
  };

  #isKeywordType(word) {
    return Token.KEYWORDS.includes(word);
  };

  #isValidIdentifierChar(c, pos) {
    const v = c.charCodeAt();
    if (v <= 0x20 || v === 0x22 || v === 0x23 || v === 0x27 || v === 0x2c || v === 0x2e || v === 0x5b || v === 0x5c ||
        v === 0x5d || v === 0x7b || v === 0x7d)
      return false;

    if (pos === 0 && v >= 0x30 && v <= 0x39)
      return false;

    return true;
  };

  #isValidIdentifier(word) {
    if (word === '')
      return false;

    for (let i = 0; i < word.length; ++i) {
      if (!this.#isValidIdentifierChar(word[i], i))
        return false;
    }

    return !this.#isKeywordType(word);
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

export {isSpace, isPunctuation};
