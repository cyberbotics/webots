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

#include "WbResetCommand.hpp"
#include "WbField.hpp"
#include "WbTemplateManager.hpp"

#include <cassert>

WbResetCommand::WbResetCommand(WbField *field, QUndoCommand *parent) :
  QUndoCommand(parent),
  mField(field),
  mPrevField(new WbField(*field, field->parentNode())) {
  assert(mField);
  setText(QObject::tr("reset"));
}

WbResetCommand::~WbResetCommand() {
  delete mPrevField;
}

void WbResetCommand::undo() {
  mField->setValue(mPrevField->value());
}

void WbResetCommand::redo() {
  // temporarily block the regeneration, otherwise the field becomes invalid when resetting MF fields
  WbTemplateManager::instance()->blockRegeneration(true);
  mField->reset();
  WbTemplateManager::instance()->blockRegeneration(false);
}
