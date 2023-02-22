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

#ifndef WB_REMOVE_ITEM_COMMAND_HPP
#define WB_REMOVE_ITEM_COMMAND_HPP

//
// Description: Representation of a 'remove item' action in multiple value fields
//              and definition of respective undo and redo functions
//

#include "WbAddItemCommand.hpp"
#include "WbMultipleValue.hpp"

class WbRemoveItemCommand : public WbAddItemCommand {
public:
  WbRemoveItemCommand(WbMultipleValue *fieldValue, int index, QUndoCommand *parent = 0);

  void undo() override { WbAddItemCommand::redo(); }  // add item
  void redo() override { WbAddItemCommand::undo(); }  // remove item
};

inline WbRemoveItemCommand::WbRemoveItemCommand(WbMultipleValue *fieldValue, int index, QUndoCommand *parent) :
  WbAddItemCommand(fieldValue, fieldValue->variantValue(index), index, parent) {
  setText(QObject::tr("remove item"));
}

#endif
