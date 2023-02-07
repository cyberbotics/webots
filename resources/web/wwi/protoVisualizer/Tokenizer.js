import Token, {isSpace, isPunctuation} from './Token.js';
import {VRML} from './vrml_type.js';

export default class Tokenizer {
  #atEndPos;
  #char;
  #column;
  #index;
  #line;
  #stream;
  #streamPos;
  #tokenColumn;
  #tokenLine;
  #proto;
  #externProto;
  constructor(stream, proto, externProto) {
    this.#char = '';
    this.vector = [];
    this.#stream = stream;
    this.#streamPos = 0;
    this.#line = 1;
    this.#column = 0;
    this.#tokenLine = 1;
    this.#tokenColumn = -1;
    this.#atEndPos = false;
    if (typeof proto === 'undefined')
      throw new Error('When tokenizing a string, a PROTO reference is required');
    this.#proto = proto; // proto reference from which we are tokenizing
    this.#externProto = externProto;
    this.#index = 0; // control position in token vector
  }

  configureFromOther(tokenizer, start, end) {
    this.vector = tokenizer.vector.filter((_, i) => { return i >= start && i < end; });
    this.rewind();
    this.#atEndPos = false;
  }

  get proto() {
    return this.#proto;
  }

  get externProto() {
    return this.#externProto;
  }

  tokenize() {
    try {
      while (1) {
        const word = this.readWord();
        const token = new Token(word, this.#tokenLine, this.#tokenColumn);
        this.vector.push(token);
      }
    } catch (e) {
    }
    // add EOF token
    this.vector.push(new Token('end of file', this.#tokenLine, this.#tokenColumn));
  };

  tokens() {
    return this.vector;
  };

  skipWhiteSpace() {
    while (isSpace(this.#char) || this.#char === '#') {
      // skip comments
      if (this.#char === '#') {
        this.#char = this.readChar();
        while (this.#char !== '\n')
          this.#char = this.readChar();
      } else
        this.#char = this.readChar();
    }
  };

  readChar() {
    if (this.#atEnd()) {
      if (!this.#atEndPos) {
        this.#atEndPos = true;
        return '\n';
      }
      throw new Error();
    }

    const c = this.#stream[this.#streamPos];
    this.#streamPos++;
    this.#column++;

    if (c === '\n') {
      this.#line++;
      this.#column = 0;
    }

    return c;
  };

  readLine() {
    this.#line++;
    this.#column = 0;
    const ix = this.#stream.indexOf('\n', this.#streamPos);
    this.#streamPos = ix !== -1 ? ix + 1 : this.#streamPos; // update stream position if found

    return ix !== -1 ? this.#stream.substring(this.#streamPos, ix) : '';
  };

  readWord() {
    this.skipWhiteSpace();

    let word = this.#char;
    this.#markTokenStart();

    // handle string literals
    if (this.#char === '"') {
      this.#char = this.readChar();
      // find closing double quote
      while (this.#char !== '"') {
        if (this.#char === '\\') {
          this.#char = this.readChar();
          if (this.#char === 'n') // '\n' is allowed to create new line in SFString
            word += '\\';
          else if (this.#char !== '\\' && this.#char !== '"') // only allowed to escape double quotes and backslash
            throw new Error('invalid escaped character at line ' + this.#line + ' column ' + this.#column + '.');
        }
        if (this.#char === '\n') {
          this.#char = '"';
          throw new Error('unclosed string literal.');
        }
        word += this.#char;
        this.#char = this.readChar();
      }
      word += this.#char;
      this.#char = this.readChar();
      return word;
    }

    // handle "[]{}"
    if (isPunctuation(this.#char)) {
      this.#char = this.readChar();
      return word;
    }

    this.#char = this.readChar();

    while (!isSpace(this.#char) && !isPunctuation(this.#char) && this.#char !== '#') {
      word += this.#char;
      this.#char = this.readChar();
    }

    return word;
  };

  rewind() {
    this.#index = 0;
  };

  forward() {
    this.#index = this.vector.length;
  };

  ungetToken() {
    --this.#index;
  };

  pos() {
    return this.#index;
  }

  seek(pos) {
    this.#index = pos;
  };

  lastToken() {
    return this.#index > 0 ? this.vector[this.#index - 1] : undefined;
  };

  lastWord() {
    return this.lastToken().word();
  };

  nextToken() {
    return this.vector[this.#index++];
  };

  nextWord() {
    return this.nextToken().word();
  };

  peekToken() {
    return this.vector[this.#index];
  };

  peekWord() {
    return this.peekToken().word();
  };

  recallWord() {
    return this.vector[this.#index - 1].word();
  };

  hasMoreTokens() {
    return this.#index < this.vector.length;
  };

  totalTokensNumber() {
    return this.vector.length;
  };

  skipToken(expectedWord) {
    if (typeof expectedWord !== 'string')
      throw new Error('When using skipToken, the argument must be a string');

    if (!this.hasMoreTokens())
      throw new Error('Expected word ' + expectedWord + ' but reached end of file.');

    const token = this.nextToken();

    if (token.word() !== expectedWord)
      throw new Error('Expected word ' + expectedWord + ' but found ' + token.word() + '.');
  };

  skipTokens(n) {
    if (typeof n !== 'number')
      throw new Error('When using skipTokens, the argument must be a number');

    if (this.#index + n < this.vector.length)
      this.#index += n;
    else
      throw new Error('Cannot skip N = ' + n + ' tokens because there are not that many left.');
  }

  spliceTokenizerByType(type) {
    const start = this.#index;
    this.consumeTokensByType(type);
    const end = this.#index;

    const subTokenizer = new Tokenizer(this.#stream, this.proto, this.externProto);
    subTokenizer.configureFromOther(this, start, end);

    return subTokenizer;
  }

  consumeTokensByType(type) {
    if (['IS', 'USE'].includes(this.peekWord())) {
      this.nextToken(); // consume keyword
      this.nextToken(); // consume value
      return;
    }

    if (this.peekWord() === 'DEF') {
      this.skipToken('DEF');
      this.nextToken();
    }

    switch (type) {
      case VRML.MFBool:
      case VRML.MFColor:
      case VRML.MFFloat:
      case VRML.MFInt32:
      case VRML.MFNode:
      case VRML.MFRotation:
      case VRML.MFString:
      case VRML.MFVec2f:
      case VRML.MFVec3f: {
        if (this.peekWord() !== '[') {
          this.consumeTokensByType(type + 1); // the MF* only has one item
          break;
        } else
          this.skipToken('[');

        let ctr = 1; // because the first '[' is preemptively skipped
        while (ctr > 0) {
          ctr = this.peekWord() === '[' ? ++ctr : ctr;
          ctr = this.peekWord() === ']' ? --ctr : ctr;
          this.nextToken();
        }
        break;
      }
      case VRML.SFNode: {
        if (this.peekWord() === 'NULL') {
          this.nextToken();
          break;
        } else if (this.peekWord() !== '{')
          this.nextToken(); // skip node name

        let ctr = 1; // because the first '{' is preemptively skipped
        this.skipToken('{'); // skip first token, must be always present for an SFNode
        while (ctr > 0) {
          ctr = this.peekWord() === '{' ? ++ctr : ctr;
          ctr = this.peekWord() === '}' ? --ctr : ctr;
          this.nextToken();
        }
        break;
      }
      case VRML.SFBool:
      case VRML.SFInt32:
      case VRML.SFFloat:
      case VRML.SFString:
        this.nextToken();
        break;
      case VRML.SFVec2f:
        this.skipTokens(2);
        break;
      case VRML.SFVec3f:
      case VRML.SFColor:
        this.skipTokens(3);
        break;
      case VRML.SFRotation:
        this.skipTokens(4);
        break;
      default:
        throw new Error('Cannot consume tokens for type \'' + type + '\'.');
    }
  };

  skipField(deleteTokens) {
    if (!this.hasMoreTokens())
      throw new Error('End of file reached while a token is expected.');

    // skip node
    if (this.peekWord() === 'USE' || this.peekWord() === 'IS') {
      if (deleteTokens) {
        --this.#index;
        this.vector.splice(this.#index, 3);
      } else {
        this.nextToken();
        this.nextToken();
      }
      return;
    }

    // skip node
    if (this.peekToken().isIdentifier() || this.peekWord() === 'DEF') {
      this.skipNode(deleteTokens);
      // remove field name
      --this.#index;
      this.vector.splice(this.#index, 1);
      return;
    }

    // skip unknown multiple value
    if (this.peekWord() === '[') {
      const startPos = this.#index - 1;
      this.nextToken();
      let counter = 1;
      do {
        const word = this.nextWord();
        if (word === '[')
          counter++;
        else if (word === ']')
          counter--;
      } while (counter > 0);

      if (deleteTokens) {
        this.vector.splice(startPos, this.#index - startPos);
        this.#index = startPos;
      }

      return;
    }

    // skip unknown single value
    const startPos = this.#index - 1;
    while (this.peekToken().isNumeric() || this.peekToken().isString() || this.peekToken().isBoolean())
      this.nextToken();

    if (deleteTokens) {
      this.vector.splice(startPos, this.#index - startPos);
      this.#index = startPos;
    }
  };

  skipNode(deleteTokens) {
    let startPos = this.#index;
    if (deleteTokens && this.peekWord() === '{')
      --startPos; // delete node name

    // move to next "{"
    while (this.hasMoreTokens() && this.nextWord() !== '{') {
    }

    if (this.lastToken().isEof()) {
      this.ungetToken();
      return;
    }

    // count the same number of opening and closing braces
    let counter = 1;
    while (counter > 0 && this.hasMoreTokens()) {
      const word = this.nextWord();
      if (word === '{')
        counter++;
      else if (word === '}')
        counter--;
    }

    if (deleteTokens) {
      const count = this.#index - startPos;
      this.vector.splice(startPos, count);
      this.#index = startPos;
    }
  };

  printTokens() { // for debugging purposes
    let string = '';
    for (const item of this.vector)
      string += item.word() + ' > ';

    console.log(string.slice(0, -3));
  }

  #markTokenStart() {
    this.#tokenLine = this.#line;
    this.#tokenColumn = this.#column;
  };

  #atEnd() {
    return this.#streamPos >= this.#stream.length - 1;
  };
}
