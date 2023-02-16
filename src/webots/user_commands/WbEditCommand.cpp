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

#include "WbEditCommand.hpp"
#include "WbMFBool.hpp"
#include "WbMFColor.hpp"
#include "WbMFDouble.hpp"
#include "WbMFInt.hpp"
#include "WbMFNode.hpp"
#include "WbMFRotation.hpp"
#include "WbMFString.hpp"
#include "WbMFVector2.hpp"
#include "WbMFVector3.hpp"
#include "WbNode.hpp"
#include "WbSFBool.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSFNode.hpp"
#include "WbSFRotation.hpp"
#include "WbSFString.hpp"
#include "WbSFVector2.hpp"
#include "WbSFVector3.hpp"
#include "WbValue.hpp"
#include "WbVariant.hpp"

#include <cassert>

WbEditCommand::WbEditCommand(WbValue *fieldValue, const WbVariant &prevValue, const WbVariant &nextValue, int index,
                             QUndoCommand *parent) :
  QUndoCommand(parent),
  mFieldValue(fieldValue),
  mPrevValue(prevValue),
  mNextValue(nextValue),
  mIndex(index) {
  assert(mFieldValue && (mFieldValue->isMultiple() ^ (mIndex < 0)));
  assert((WbValue::toSingle(mFieldValue->type()) == WB_SF_NODE) ||
         ((WbValue::toSingle(mFieldValue->type()) == prevValue.type()) && (prevValue.type() == nextValue.type())));
  setText(QObject::tr("edit"));
}

void WbEditCommand::undo() {
  resetValue(mPrevValue);
}

void WbEditCommand::redo() {
  resetValue(mNextValue);
}

void WbEditCommand::resetValue(const WbVariant &newValue) {
  switch (mFieldValue->type()) {
    case WB_MF_VEC2F:
      dynamic_cast<WbMFVector2 *>(mFieldValue)->setItem(mIndex, newValue.toVector2());
      break;
    case WB_SF_VEC2F:
      static_cast<WbSFVector2 *>(mFieldValue)->setValue(newValue.toVector2());
      break;
    case WB_MF_VEC3F:
      static_cast<WbMFVector3 *>(mFieldValue)->setItem(mIndex, newValue.toVector3());
      break;
    case WB_SF_VEC3F:
      static_cast<WbSFVector3 *>(mFieldValue)->setValueByUser(newValue.toVector3(), false);
      break;
    case WB_MF_COLOR:
      static_cast<WbMFColor *>(mFieldValue)->setItem(mIndex, newValue.toColor());
      break;
    case WB_SF_COLOR:
      static_cast<WbSFColor *>(mFieldValue)->setValue(newValue.toColor());
      break;
    case WB_MF_STRING:
      static_cast<WbMFString *>(mFieldValue)->setItem(mIndex, newValue.toString());
      break;
    case WB_SF_STRING:
      static_cast<WbSFString *>(mFieldValue)->setValue(newValue.toString());
      break;
    case WB_MF_INT32:
      static_cast<WbMFInt *>(mFieldValue)->setItem(mIndex, newValue.toInt());
      break;
    case WB_SF_INT32:
      static_cast<WbSFInt *>(mFieldValue)->setValue(newValue.toInt());
      break;
    case WB_MF_FLOAT:
      static_cast<WbMFDouble *>(mFieldValue)->setItem(mIndex, newValue.toDouble());
      break;
    case WB_SF_FLOAT:
      static_cast<WbSFDouble *>(mFieldValue)->setValue(newValue.toDouble());
      break;
    case WB_MF_ROTATION:
      static_cast<WbMFRotation *>(mFieldValue)->setItem(mIndex, newValue.toRotation());
      break;
    case WB_SF_ROTATION:
      static_cast<WbSFRotation *>(mFieldValue)->setValueByUser(newValue.toRotation(), false);
      break;
    case WB_MF_BOOL:
      static_cast<WbMFBool *>(mFieldValue)->setItem(mIndex, newValue.toBool());
      break;
    case WB_SF_BOOL:
      static_cast<WbSFBool *>(mFieldValue)->setValue(newValue.toBool());
      break;
    case WB_MF_NODE:
      assert(newValue.type() == WB_SF_STRING);
      static_cast<WbMFNode *>(mFieldValue)->item(mIndex)->setDefName(newValue.toString(), true);
      break;
    case WB_SF_NODE:
      assert(newValue.type() == WB_SF_STRING);
      static_cast<WbSFNode *>(mFieldValue)->value()->setDefName(newValue.toString(), true);
      break;
    default:
      assert(false);
      break;
  }
}
