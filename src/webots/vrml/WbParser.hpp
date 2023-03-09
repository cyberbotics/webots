// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
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
//   e.g.: .wbt, .proto, imported object strings and .wrl files
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
  bool parseWorld(const QString &worldPath, bool (*updateProgress)(int));  // parse a .wbt file
  bool parseObject(const QString &worldPath);                              // parse an imported object from string
  bool parseNodeModel();                                                   // parse a .wrl node model in resources/nodes

  bool parseProtoInterface(const QString &worldPath);  // parse PROTO interface in original file
  bool parseProtoBody(const QString &worldPath);       // parse resulting PROTO after template generation
  void skipExternProto();

  // skip PROTO definition in the specified tokenizer
  // this is useful to skip in file PROTO definition for VRML import
  // prerequisite: the tokenizer must point to the "PROTO" keyword
  static void skipProtoDefinition(WbTokenizer *tokenizer);
  static double legacyGravity();
  // returns the list of all PROTO nodes invoked by a tokenized vrml string
  QStringList protoNodeList();

private:
  WbTokenizer *mTokenizer;
  bool mProto;

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
  const QString parseUrl();
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
