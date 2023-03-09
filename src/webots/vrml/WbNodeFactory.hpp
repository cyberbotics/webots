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

#ifndef WB_NODE_FACTORY_HPP
#define WB_NODE_FACTORY_HPP

//
// Description: singleton class responsible for instantiating nodes and protos
//
// Inherited by: WbConcreteNodeFactory
//

#include <cstddef>

#include <QtCore/QString>

class WbField;
class WbNode;
class WbTokenizer;
class WbWriter;

class WbNodeFactory {
public:
  static WbNodeFactory *instance();

  // create a built-in node or proto instance matching 'modelName'
  // if 'tokenizer' is not specified, the node or proto instance is constructed with default field values
  // 'parentNode' specifies the global parent value to be set before creating PROTO instances, if it is
  // not specified, the current global parent is used
  virtual WbNode *createNode(const QString &modelName, WbTokenizer *tokenizer = 0, WbNode *parentNode = NULL,
                             const QString *protoFilePath = NULL) = 0;

  // create and return a copy of a node
  // the fields of the copy are initialized with the values found in the original
  // the copy will have the same model as the original
  virtual WbNode *createCopy(const WbNode &original) = 0;

  virtual const QString slotType(WbNode *node) = 0;
  virtual bool validateExistingChildNode(const WbField *field, const WbNode *childNode, const WbNode *node,
                                         bool isInBoundingObject, QString &errorMessage) const = 0;

protected:
  WbNodeFactory();
  virtual ~WbNodeFactory();

private:
};

#endif
