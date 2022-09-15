// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbFieldEditor.hpp"

#include "WbAction.hpp"
#include "WbActionManager.hpp"
#include "WbApplication.hpp"
#include "WbBoolEditor.hpp"
#include "WbColorEditor.hpp"
#include "WbDoubleEditor.hpp"
#include "WbExtendedStringEditor.hpp"
#include "WbExternProtoEditor.hpp"
#include "WbField.hpp"
#include "WbIntEditor.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbMultipleValue.hpp"
#include "WbNode.hpp"
#include "WbNodeEditor.hpp"
#include "WbNodePane.hpp"
#include "WbNodeUtilities.hpp"
#include "WbProject.hpp"
#include "WbProtoModel.hpp"
#include "WbRotationEditor.hpp"
#include "WbSFNode.hpp"
#include "WbSFVector2.hpp"
#include "WbSFVector3.hpp"
#include "WbSelection.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbUndoStack.hpp"
#include "WbVector2Editor.hpp"
#include "WbVector3Editor.hpp"

#include <QtGui/QAction>
#include <QtWidgets/QLabel>
#include <QtWidgets/QStackedLayout>
#include <QtWidgets/QToolButton>

#include <cassert>

// empty value editor: used when nothing needs editing
class WbEmptyEditor : public WbValueEditor {
public:
  explicit WbEmptyEditor(QWidget *parent = NULL) : WbValueEditor(parent) {}

  void recursiveBlockSignals(bool block) override { blockSignals(block); }

  QWidget *lastEditorWidget() override { return NULL; }

protected:
  void edit(bool copyOriginalValue) override {}

  void resetFocus() override {}

protected slots:
  void apply() override {}

private:
  void takeKeyboardFocus() override {}
};

static QSize gMinimumSizeOffset = QSize(0, 0);

WbFieldEditor::WbFieldEditor(QWidget *parent) :
  QWidget(parent),
  mNode(NULL),
  mField(NULL),
  mItem(-1),
  mNodeItem(NULL),
  mIsValidItemIndex(false) {
  setObjectName("fieldEditorGroupBox");

  WbExtendedStringEditor *const stringEditor = new WbExtendedStringEditor(this);
  connect(stringEditor, &WbExtendedStringEditor::editRequested, this, &WbFieldEditor::editRequested);

  WbNodePane *const nodePane = new WbNodePane(this);
  const WbNodeEditor *nodeEditor = nodePane->nodeEditor();
  connect(nodeEditor, &WbNodeEditor::dictionaryUpdateRequested, this, &WbFieldEditor::dictionaryUpdateRequested);

  // create editors
  mEditors.insert(WB_NO_FIELD, new WbEmptyEditor(this));
  mEditors.insert(WB_SF_BOOL, new WbBoolEditor(this));
  mEditors.insert(WB_SF_STRING, stringEditor);
  mEditors.insert(WB_SF_INT32, new WbIntEditor(this));
  mEditors.insert(WB_SF_FLOAT, new WbDoubleEditor(this));
  mEditors.insert(WB_SF_VEC2F, new WbVector2Editor(this));
  mEditors.insert(WB_SF_VEC3F, new WbVector3Editor(this));
  mEditors.insert(WB_SF_ROTATION, new WbRotationEditor(this));
  mEditors.insert(WB_SF_COLOR, new WbColorEditor(this));
  mEditors.insert(WB_SF_NODE, nodePane);

  mExternProtoEditor = new WbExternProtoEditor(this);

  // place all editors in a stacked layout
  mStackedLayout = new QStackedLayout();
  mStackedLayout->setSpacing(0);
  mStackedLayout->setContentsMargins(0, 0, 0, 0);
  foreach (WbValueEditor *editor, mEditors) {
    mStackedLayout->addWidget(editor);
    // trigger 3D view update after field value change
    connect(editor, &WbValueEditor::valueChanged, this, &WbFieldEditor::valueChanged);
  }
  mStackedLayout->addWidget(mExternProtoEditor);
  connect(nodePane->nodeEditor(), &WbValueEditor::valueChanged, this, &WbFieldEditor::valueChanged);
  connect(WbApplication::instance(), &WbApplication::worldLoadCompleted, this, &WbFieldEditor::refreshExternProtoEditor);

  mTitleLabel = new QLabel(this);
  mTitleLabel->setAlignment(Qt::AlignCenter);

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(mTitleLabel);
  mainLayout->addLayout(mStackedLayout);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  gMinimumSizeOffset = sizeHint() - mStackedLayout->sizeHint();

  setCurrentWidget(0);
}

WbFieldEditor::~WbFieldEditor() {
}

WbValueEditor *WbFieldEditor::currentEditor() const {
  return static_cast<WbValueEditor *>(mStackedLayout->currentWidget());
}

void WbFieldEditor::refreshExternProtoEditor() {
  WbExternProtoEditor *editor = dynamic_cast<WbExternProtoEditor *>(mExternProtoEditor);
  if (currentEditor() == editor)
    editor->updateContents();
}

void WbFieldEditor::setTitle(const QString &title) {
  if (title.isEmpty())
    mTitleLabel->setText("");
  else
    mTitleLabel->setText("Selection: " + title);
}

QWidget *WbFieldEditor::lastEditorWidget() {
  return currentEditor()->lastEditorWidget();
}

QString WbFieldEditor::nodeAsTitle(WbNode *node) {
  if (!node)
    return "";
  else if (node->isProtoInstance())
    return node->modelName() + " (" + node->nodeModelName() + ")";
  else
    return node->modelName();
}

void WbFieldEditor::updateTitle() {
  assert(mField);

  QString title;
  WbValue *value = mField->value();

  if (mField->type() == WB_MF_NODE && mItem != -1)
    title = nodeAsTitle(static_cast<WbMFNode *>(value)->item(mItem));
  else if (mField->type() == WB_SF_NODE)
    title = mField->name() + " " + nodeAsTitle(static_cast<WbSFNode *>(value)->value());
  else {
    WbMultipleValue *multipleValue = dynamic_cast<WbMultipleValue *>(value);
    if (multipleValue) {
      if (mItem == -1) {
        int size = multipleValue->size();
        QString type = WbValue::typeToShortName(value->singleType());
        if (size != 1)
          type += "s";

        title = QString("%1 (%2 %3)").arg(mField->name()).arg(size).arg(type);
      } else
        title = QString("%1 (%3 #%2)").arg(mField->name()).arg(mItem + 1).arg(WbValue::typeToShortName(value->singleType()));
    } else
      title = QString("%1 (%2)").arg(mField->name(), value->shortTypeName());
  }

  setTitle(title);
}

void WbFieldEditor::editExternProto() {
  mTitleLabel->setText("IMPORTABLE EXTERNPROTO");
  // disable current editor widget
  WbValueEditor *current = currentEditor();
  current->applyIfNeeded();
  current->stopEditing();
  disconnect(current, &WbValueEditor::valueInvalidated, this, &WbFieldEditor::invalidateValue);

  // enable extern proto
  WbExternProtoEditor *editor = dynamic_cast<WbExternProtoEditor *>(mExternProtoEditor);
  if (editor) {
    editor->updateContents();
    setCurrentWidget(mExternProtoEditor);
  }
}

void WbFieldEditor::editField(WbNode *node, WbField *field, int item) {
  disconnect(this, &WbFieldEditor::valueChanged, this, &WbFieldEditor::updateResetButton);
  if (node == mNode && field == mField && item == mItem) {
    if (field || node)
      updateValue(false);
    return;
  }

  mNode = node;
  mField = field;
  mItem = item;
  mNodeItem = NULL;
  mIsValidItemIndex = false;

  // disable current editor widget
  WbValueEditor *editor = currentEditor();
  editor->applyIfNeeded();
  editor->stopEditing();
  disconnect(editor, &WbValueEditor::valueInvalidated, this, &WbFieldEditor::invalidateValue);

  if (field == NULL && node == NULL) {
    invalidateValue();
    WbActionManager::instance()->action(WbAction::RESET_VALUE)->setEnabled(false);
    return;
  }

  computeFieldInformation();

  updateTitle();

  assert(field);
  WbActionManager::instance()
    ->action(WbAction::RESET_VALUE)
    ->setEnabled(!((field->isMultiple() && mIsValidItemIndex) || mField->isDefault()));

  if (field->isMultiple() && !mIsValidItemIndex) {
    setCurrentWidget(0);
    return;
  }

  // check if the selected item is a Solid node
  const QList<WbValueEditor *> &v = mEditors.values(field->value()->singleType());
  const bool editingSolid = dynamic_cast<WbSolid *>(mNodeItem) != NULL;
  if (v.size() == 1)
    editor = v.at(0);
  else
    editor = editingSolid ? v.at(0) : v.at(1);

  editor->edit(node, field, item);
  setCurrentWidget(editor);
  connect(editor, &WbValueEditor::valueInvalidated, this, &WbFieldEditor::invalidateValue);
  connect(this, &WbFieldEditor::valueChanged, this, &WbFieldEditor::updateResetButton);
}

void WbFieldEditor::invalidateValue() {
  mNode = NULL;
  mField = NULL;
  mItem = -1;
  setCurrentWidget(0);
}

void WbFieldEditor::resetFocus() {
  WbValueEditor *editor = currentEditor();
  editor->resetFocus();
}

void WbFieldEditor::updateResetButton() {
  const WbMultipleValue *const multipleValue = dynamic_cast<WbMultipleValue *>(mField->value());
  bool enabled = !((multipleValue && (mItem >= 0) && (mItem < multipleValue->size())) || mField->isDefault());
  WbActionManager::instance()->action(WbAction::RESET_VALUE)->setEnabled(enabled);
}

void WbFieldEditor::updateValue(bool copyOriginalValue) {
  WbValueEditor *editor = currentEditor();
  editor->edit(copyOriginalValue);
  updateResetButton();
}

void WbFieldEditor::applyChanges() {
  WbValueEditor *editor = currentEditor();
  editor->applyIfNeeded();
}

void WbFieldEditor::computeFieldInformation() {
  assert(mField);

  mNodeItem = NULL;
  mIsValidItemIndex = false;

  // check and store field type information
  WbValue *const value = mField->value();
  WbMultipleValue *const multipleValue = dynamic_cast<WbMultipleValue *>(value);
  WbMFNode *mfNode = NULL;

  if (multipleValue) {
    mIsValidItemIndex = (mItem >= 0) && (mItem < multipleValue->size());

    if (mIsValidItemIndex) {
      mfNode = dynamic_cast<WbMFNode *>(multipleValue);
      if (mfNode)
        mNodeItem = mfNode->item(mItem);
    }
  } else {
    WbSFNode *const sfNode = dynamic_cast<WbSFNode *>(value);
    if (sfNode)
      mNodeItem = sfNode->value();
  }
}

void WbFieldEditor::setCurrentWidget(int index) {
  setCurrentWidget(mEditors.value(WbFieldType(index)));
}

void WbFieldEditor::setCurrentWidget(WbValueEditor *editor) {
  mStackedLayout->setCurrentWidget(editor);
}
