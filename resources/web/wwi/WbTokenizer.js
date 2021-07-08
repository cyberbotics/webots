import WbToken, {isSpace, isPunctuation} from './WbToken.js';

export default class WbTokenizer {
  constructor(stream) {
    this._char = '';
    this._vector = [];
    this._stream = stream;
    this._streamPos = 0;
    this._line = 1;
    this._column = 0;
    this._tokenLine = 1;
    this._tokenColumn = -1;
    this._atEndPos = false;
    // control position in token vector
    this._index = 0;
  }

  tokenize() {
    try {
      while (1) {
        const word = this.readWord();
        const token = new WbToken(word, this._tokenLine, this._tokenColumn);
        this._vector.push(token);
      }
    } catch (e) {
    }

    // add EOF token
    this._vector.push(new WbToken('end of file', this._tokenLine, this._tokenColumn));
  };

  tokens() {
    return this._vector;
  };

  skipWhiteSpace() {
    while (isSpace(this._char) || this._char === '#') {
      // skip comments
      if (this._char === '#') {
        this._char = this.readChar();
        while (this._char !== '\n')
          this._char = this.readChar();
      } else
        this._char = this.readChar();
    }
  };

  readChar() {
    if (this._atEnd()) {
      if (!this._atEndPos) {
        this._atEndPos = true;
        return '\n';
      }
      throw new Error();
    }

    const c = this._stream[this._streamPos];
    this._streamPos++;
    this._column++;

    if (c === '\n') {
      this._line++;
      this._column = 0;
    }

    return c;
  };

  readLine() {
    this._line++;
    this._column = 0;
    const ix = this._stream.indexOf('\n', this._streamPos);
    this._streamPos = ix !== -1 ? ix + 1 : this._streamPos; // update stream position if found

    return ix !== -1 ? this._stream.substring(this._streamPos, ix) : '';
  };

  readWord() {
    this.skipWhiteSpace();

    let word = this._char;
    this._markTokenStart();

    // handle string literals
    if (this._char === '"') {
      this._char = this.readChar();
      // find closing double quote
      while (this._char !== '"') {
        if (this._char === '\\') {
          this._char = this.readChar();
          if (this._char === 'n') // '\n' is allowed to create new line in SFString
            word += '\\';
          else if (this._char !== '\\' && this._char !== '"') // only allowed to escape double quotes and backslash
            throw new Error('invalid escaped character at line ' + this._line + ' column ' + this._column + '.');
        }
        if (this._char === '\n') {
          this._char = '"';
          throw new Error('unclosed string literal.');
        }
        word += this._char;
        this._char = this.readChar();
      }
      word += this._char;
      this._char = this.readChar();
      return word;
    }

    // TODO: tokenize template

    // handle "[]{}"
    if (isPunctuation(this._char)) {
      this._char = this.readChar();
      return word;
    }

    this._char = this.readChar();

    while (!isSpace(this._char) && !isPunctuation(this._char) && this._char !== '#') {
      word += this._char;
      this._char = this.readChar();
    }

    return word;
  };

  rewind() {
    this._index = 0;
  };

  forward() {
    this._index = this._vector.length;
  };

  ungetToken() {
    --this._index;
  };

  pos() {
    return this._index;
  }

  seek(pos) {
    this._index = pos;
  };

  lastToken() {
    return this._index > 0 ? this._vector[this._index - 1] : undefined;
  };

  lastWord() {
    return this.lastToken().word();
  };

  nextToken() {
    return this._vector[this._index++];
  };

  nextWord() {
    return this.nextToken().word();
  };

  peekToken() {
    return this._vector[this._index];
  };

  peekWord() {
    return this.peekToken().word();
  };

  hasMoreTokens() {
    return this._index < this._vector.length;
  };

  totalTokensNumber() {
    return this._vector.length;
  };

  skipToken(expectedWord) {
    if (!this.hasMoreTokens())
      throw new Error('Expected word ' + expectedWord + ' but reached end of file.');

    const token = this.nextToken();

    if (token.word() !== expectedWord)
      throw new Error('Expected word ' + expectedWord + ' but found ' + token.word() + '.');
  };

  skipField(deleteTokens) {
    if (!this.hasMoreTokens())
      throw new Error('End of file reached while a token is expected.');

    // skip node
    if (this.peekWord() === 'USE' || this.peekWord() === 'IS') {
      if (deleteTokens) {
        --this._index;
        this._vector.splice(this._index, 3);
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
      --this._index;
      this._vector.splice(this._index, 1);
      return;
    }

    // skip unknown multiple value
    if (this.peekWord() === '[') {
      const startPos = this._index - 1;
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
        this._vector.splice(startPos, this._index - startPos);
        this._index = startPos;
      }

      return;
    }

    // skip unknown single value
    const startPos = this._index - 1;
    while (this.peekToken().isNumeric() || this.peekToken().isString() || this.peekToken().isBoolean())
      this.nextToken();

    if (deleteTokens) {
      this._vector.splice(startPos, this._index - startPos);
      this._index = startPos;
    }
  };

  skipNode(deleteTokens) {
    let startPos = this._index;
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
      const count = this._index - startPos;
      this._vector.splice(startPos, count);
      this._index = startPos;
    }
  };

  _markTokenStart() {
    this._tokenLine = this._line;
    this._tokenColumn = this._column;
  };

  _atEnd() {
    return this._streamPos >= this._stream.length - 1;
  };
}
