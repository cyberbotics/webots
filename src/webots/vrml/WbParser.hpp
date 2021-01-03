// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_PARSER_HPP
#define WB_PARSER_HPP

//
// Description:
//   WbParser allows to check (parse) the syntax of the various VRML-based file types used in Webots:
//   e.g.: .wbt, .proto, .wbo and .wrl files
//   Error messages are reported to WbLog
//

#include "../../../include/controller/c/webots/supervisor.h"  // WbFieldType

#include <QtCore/QString>

class WbNodeModel;
class WbProtoModel;
class WbToken;
class WbTokenizer;

class WbParser {
public:
  // create a parser for the specified tokenizer
  explicit WbParser(WbTokenizer *tokenizer);
  ~WbParser();

  // name of file being checked
  const QString &fileName() const;

  // check syntax, report errors to WbLog
  // return true if there were no errors
  bool parseWorld(const QString &worldPath);   // parse a .wbt file
  bool parseVrml(const QString &worldPath);    // parse a .wrl file before importing
  bool parseObject(const QString &worldPath);  // parse a .wbo files
  bool parseNodeModel();                       // parse a .wrl node model in resources/nodes

  bool parseProtoInterface(const QString &worldPath);  // parse PROTO interface in original file
  bool parseProtoBody(const QString &worldPath);       // parse resulting PROTO after template generation

  // skip PROTO definition in the specified tokenizer
  // this is useful to skip in file PROTO definition for VRML import
  // prerequisite: the tokenizer must point to the "PROTO" keyword
  static void skipProtoDefinition(WbTokenizer *tokenizer);
  static double legacyGravity();

private:
  WbTokenizer *mTokenizer;
  int mMode;

  enum { NONE, WBT, VRML, PROTO, WBO, WRL };
  void parseDoubles(int n);
  void parseInt();
  void parseBool();
  void parseString();
  void parseSingleFieldValue(WbFieldType type, const QString &worldPath);
  void parseFieldValue(WbFieldType type, const QString &worldPath);
  void parseField(const WbNodeModel *nodeModel, const QString &worldPath);
  void parseParameter(const WbProtoModel *protoModel, const QString &worldPath);
  void parseProtoDefinition(const QString &worldPath);
  void parseNode(const QString &worldPath);
  void parseExactWord(const QString &word);
  const QString &parseIdentifier(const QString &expected = "identifier");
  void parseEof();
  void parseFieldDeclaration(const QString &worldPath);
  void parseFieldAcceptedValues(WbFieldType type, const QString &worldPath);
  WbToken *nextToken();
  void skipToken();
  const QString &nextWord();
  const QString &peekWord() const;
  WbToken *peekToken();
  void reportFileError(const QString &message) const;
  void reportError(const QString &message, const WbToken *token = NULL) const;
  void reportUnexpected(const QString &expected) const;
  bool isFieldTypeName(const QString &typeName);
};

#endif
