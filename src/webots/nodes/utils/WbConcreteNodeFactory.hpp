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

#ifndef WB_CONCRETE_NODE_FACTORY_HPP
#define WB_CONCRETE_NODE_FACTORY_HPP

//
// Description: a class used to instantiate nodes and protos
//

#include "WbNodeFactory.hpp"

class WbTokenizer;
class WbNode;
class QString;
class WbWriter;

class WbConcreteNodeFactory : public WbNodeFactory {
public:
  // reimplemented public functions
  WbNode *createNode(const QString &modelName, WbTokenizer *tokenizer = 0, WbNode *parentNode = NULL,
                     const QString *protoUrl = NULL) override;
  WbNode *createCopy(const WbNode &original) override;
  const QString slotType(WbNode *node) override;
  bool validateExistingChildNode(const WbField *field, const WbNode *childNode, const WbNode *node, bool isInBoundingObject,
                                 QString &errorMessage) const override;

private:
  WbConcreteNodeFactory() {}
  virtual ~WbConcreteNodeFactory() {}
  static WbConcreteNodeFactory gFactory;
};

#endif
