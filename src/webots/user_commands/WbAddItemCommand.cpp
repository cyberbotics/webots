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

#include "WbAddItemCommand.hpp"

#include "WbField.hpp"
#include "WbMFBool.hpp"
#include "WbMFColor.hpp"
#include "WbMFDouble.hpp"
#include "WbMFInt.hpp"
#include "WbMFRotation.hpp"
#include "WbMFString.hpp"
#include "WbMFVector2.hpp"
#include "WbMFVector3.hpp"
#include "WbMultipleValue.hpp"
#include "WbSingleValue.hpp"

#include <cassert>

WbAddItemCommand::WbAddItemCommand(WbField *const field, WbMultipleValue *fieldValue, int index, QUndoCommand *parent) :
  QUndoCommand(parent),
  mFieldValue(fieldValue),
  mIndex(index) {
  assert(mIndex >= 0 && mFieldValue);
  setText(QObject::tr("add item"));
  if (field->hasRestrictedValues())
    mItem = field->acceptedValues()[0];
}

WbAddItemCommand::WbAddItemCommand(WbMultipleValue *fieldValue, const WbVariant &item, int index, QUndoCommand *parent) :
  QUndoCommand(parent),
  mFieldValue(fieldValue),
  mItem(item),
  mIndex(index) {
  assert(mIndex >= 0 && mFieldValue);
  assert(mItem.isEmpty() || WbValue::toSingle(mFieldValue->type()) == mItem.type());
  setText(QObject::tr("add item"));
}

void WbAddItemCommand::undo() {
  mFieldValue->removeItem(mIndex);
}

void WbAddItemCommand::redo() {
  if (mItem.isEmpty()) {
    mFieldValue->insertDefaultItem(mIndex);
    return;
  }

  switch (mFieldValue->type()) {
    case WB_MF_VEC2F:
      dynamic_cast<WbMFVector2 *>(mFieldValue)->insertItem(mIndex, mItem.toVector2());
      break;
    case WB_MF_VEC3F:
      static_cast<WbMFVector3 *>(mFieldValue)->insertItem(mIndex, mItem.toVector3());
      break;
    case WB_MF_COLOR:
      static_cast<WbMFColor *>(mFieldValue)->insertItem(mIndex, mItem.toColor());
      break;
    case WB_MF_STRING:
      static_cast<WbMFString *>(mFieldValue)->insertItem(mIndex, mItem.toString());
      break;
    case WB_MF_INT32:
      static_cast<WbMFInt *>(mFieldValue)->insertItem(mIndex, mItem.toInt());
      break;
    case WB_MF_FLOAT:
      static_cast<WbMFDouble *>(mFieldValue)->insertItem(mIndex, mItem.toDouble());
      break;
    case WB_MF_ROTATION:
      static_cast<WbMFRotation *>(mFieldValue)->insertItem(mIndex, mItem.toRotation());
      break;
    case WB_MF_BOOL:
      static_cast<WbMFBool *>(mFieldValue)->insertItem(mIndex, mItem.toBool());
      break;
    default:
      assert(false);
      break;
  }
}
