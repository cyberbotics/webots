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

#ifndef WB_EDIT_COMMAND_HPP
#define WB_EDIT_COMMAND_HPP

//
// Description: Representation of an 'edit' action on field or nodes DEF name
//              and definition of respective undo and redo functions
//

#include <QtGui/QUndoCommand>

#include "WbVariant.hpp"

class WbValue;

class WbEditCommand : public QUndoCommand {
public:
  WbEditCommand(WbValue *fieldValue, const WbVariant &prevValue, const WbVariant &nextValue, int index = -1,
                QUndoCommand *parent = 0);
  ~WbEditCommand() {}

  void undo() override;
  void redo() override;

private:
  WbValue *mFieldValue;
  const WbVariant mPrevValue;
  const WbVariant mNextValue;
  const int mIndex;

  void resetValue(const WbVariant &newValue);
};

#endif
