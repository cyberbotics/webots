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

#ifndef WB_ADD_ITEM_COMMAND_HPP
#define WB_ADD_ITEM_COMMAND_HPP

//
// Description: Representation of a 'add item' action in multiple value fields
//              and definition of respective undo and redo functions
//

#include <QtGui/QUndoCommand>

#include "WbVariant.hpp"

class WbField;
class WbMultipleValue;

class WbAddItemCommand : public QUndoCommand {
public:
  // add item with default value
  WbAddItemCommand(WbField *const field, WbMultipleValue *fieldValue, int index, QUndoCommand *parent = 0);
  // add item with given value
  WbAddItemCommand(WbMultipleValue *fieldValue, const WbVariant &item, int index, QUndoCommand *parent = 0);
  ~WbAddItemCommand() {}

  void undo() override;  // remove item
  void redo() override;  // add item

private:
  WbMultipleValue *mFieldValue;
  WbVariant mItem;
  const int mIndex;
};

#endif
