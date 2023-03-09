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

#ifndef WB_NODE_READER_HPP
#define WB_NODE_READER_HPP

//
// Description: WbNodeReader is helper class that allows to read nodes within a DEF/USE scope
//

#include <QtCore/QMap>
#include <QtCore/QObject>

class WbNode;
class WbTokenizer;

class WbNodeReader : public QObject {
  Q_OBJECT;

public:
  // returns the current node reader in use
  // this is useful e.g. when several node readers are used, e.g. to read embedded proto definitions
  static WbNodeReader *current();

  // NORMAL: read and instantiate
  // PROTO_MODEL: read as WbNode (no instantiation)
  enum Mode { NORMAL, PROTO_MODEL };

  // create a new node reader and make it the current node reader
  explicit WbNodeReader(Mode mode = NORMAL);

  // destroy the node reader and make it not current
  ~WbNodeReader();

  void setReadingBoundingObject(bool bo) { mIsReadingBoundingObject = bo; }
  bool isReadingBoundingObject() const { return mIsReadingBoundingObject; }

  // read nodes and place them in a list
  // this function is suitable for reading all the nodes of a .wbt file
  // prerequisite: the syntax must have been checked with the WbParser
  QList<WbNode *> readNodes(WbTokenizer *tokenizer, const QString &worldPath);

  // read a single node, this is suitable for
  // 1. reading the root node of a .proto file
  // 2. reading a default SFNode value in a .wrl file
  // prerequisites:
  // 1. the syntax must have been checked with the WbParser
  // 2. the tokenizer must be rewinded
  WbNode *readNode(WbTokenizer *tokenizer, const QString &worldPath);

  // Add DEF node that can be referenced by the read node
  void addDefNode(WbNode *defNode);
  // Remove DEF node that becomes invalid
  void removeDefNode(WbNode *defNode);

public slots:
  void cancelReadNodes();

signals:
  void readNodesHasProgressed(int progress);  // progress 0: beginning 100: end

private:
  Mode mMode;
  QMap<QString, WbNode *> mDefs;
  bool mIsReadingBoundingObject;
  bool mReadNodesCanceled;

  WbNode *createNode(const QString &modelName, WbTokenizer *tokenizer, const QString &worldPath, const QString &fileName);
};

#endif
