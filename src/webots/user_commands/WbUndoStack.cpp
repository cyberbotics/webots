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

#include "WbUndoStack.hpp"

#include "WbActionManager.hpp"
#include "WbNode.hpp"
#include "WbTemplateManager.hpp"

static WbUndoStack *gInstance = NULL;

WbUndoStack *WbUndoStack::instance() {
  if (gInstance == NULL)
    gInstance = new WbUndoStack();

  return gInstance;
}

void WbUndoStack::cleanup() {
  gInstance->clear();
  delete gInstance;
}

WbUndoStack::WbUndoStack() {
  setUndoLimit(50);

  connect(WbTemplateManager::instance(), &WbTemplateManager::postNodeRegeneration, this, &WbUndoStack::clearRequest);
};

WbUndoStack::~WbUndoStack() {
  gInstance = NULL;
}

void WbUndoStack::push(QUndoCommand *cmd) {
  // The clear request mechanism is about avoiding ordering issues.
  // Indeed, the clearRequest slot is fired inside the QUndoStack::push() call,
  // but the undo stack is incremented only at the end of this call.

  mClearRequest = false;

  QUndoStack::push(cmd);  // may change the value of mClearRequest via the clearRequest slot

  // cppcheck-suppress knownConditionTrueFalse
  if (mClearRequest)
    clear();

  updateActions();
  // notify the scene tree that some fields changed in order to update the
  // field editor if needed
  // for example in case of translation using the handles from the 3D window
  emit changed();
}

void WbUndoStack::undo() {
  QUndoStack::undo();
  updateActions();
  emit changed();
}

void WbUndoStack::redo() {
  QUndoStack::redo();
  updateActions();
  emit changed();
}

void WbUndoStack::updateActions() {
  WbActionManager::instance()->setEnabled(WbAction::UNDO, canUndo());
  WbActionManager::instance()->setEnabled(WbAction::REDO, canRedo());
}
