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

#include "WbValueEditor.hpp"

#include "WbEditCommand.hpp"
#include "WbField.hpp"
#include "WbMultipleValue.hpp"
#include "WbSingleValue.hpp"
#include "WbUndoStack.hpp"
#include "WbValue.hpp"
#include "WbVariant.hpp"
#include "WbWorld.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

WbValueEditor::WbValueEditor(QWidget *parent) :
  QWidget(parent),
  mNewValue(new WbVariant()),
  mPreviousValue(new WbVariant()),
  mComboBox(new QComboBox(this)),
  mLayout(new QGridLayout(this)),
  mNode(NULL),
  mField(NULL),
  mValue(NULL),
  mSingleValue(NULL),
  mMultipleValue(NULL),
  mIndex(-1) {
  mLayout->setColumnStretch(0, 0);
  mLayout->addWidget(mComboBox, 0, 1);
  mLayout->setColumnStretch(2, 0);
  mComboBox->setVisible(false);
}

WbValueEditor::~WbValueEditor() {
  delete mNewValue;
  delete mPreviousValue;
}

void WbValueEditor::edit(WbNode *node, WbField *field, int index) {
  mNode = node;
  mField = field;
  mValue = field->value();
  mSingleValue = dynamic_cast<WbSingleValue *>(mValue);
  mMultipleValue = dynamic_cast<WbMultipleValue *>(mValue);
  mIndex = index;

  disconnect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(apply()));
  mComboBox->clear();
  if (mField->singleType() != WB_SF_NODE && mField->hasRestrictedValues()) {
    if (mField->singleType() != WB_SF_STRING) {
      foreach (const WbVariant acceptedVariant, mField->acceptedValues())
        mComboBox->addItem(acceptedVariant.toSimplifiedStringRepresentation());
    } else {  // In case of MF/SF_STRING we don't want to display the starting and ending '"'
      foreach (const WbVariant acceptedVariant, mField->acceptedValues())
        mComboBox->addItem(acceptedVariant.toSimplifiedStringRepresentation().chopped(1).remove(0, 1));
    }
    connect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(apply()), Qt::UniqueConnection);
    connect(field, &WbField::valueChanged, this, &WbValueEditor::updateComboBoxIndex, Qt::UniqueConnection);

    updateComboBoxIndex();
    mComboBox->setVisible(true);
  } else
    mComboBox->setVisible(false);

  // call subclass
  edit(true);

  // start watching this value
  if (mValue)
    connect(mValue, &WbValue::destroyed, this, &WbValueEditor::cleanValue);
}

// disconnect from previously watched value
void WbValueEditor::stopEditing() {
  if (mValue)
    disconnect(mValue, &WbValue::destroyed, this, &WbValueEditor::valueInvalidated);

  mNode = NULL;
  mField = NULL;
  mValue = NULL;
  mSingleValue = NULL;
  mMultipleValue = NULL;
  mIndex = -1;
}

void WbValueEditor::cleanValue() {
  stopEditing();
  emit valueInvalidated();
}

void WbValueEditor::apply() {
  if (mValue == NULL)
    // no valid field to edit
    return;

  WbWorld::instance()->setModifiedFromSceneTree();
  WbUndoStack *stack = WbUndoStack::instance();
  // avoid complete reset of field editor when setting a field value
  stack->blockSignals(true);
  stack->push(new WbEditCommand(mValue, *mPreviousValue, *mNewValue, mIndex));
  stack->blockSignals(false);
  emit valueChanged();
}

void WbValueEditor::updateComboBoxIndex() {
  if (!mField)
    return;
  disconnect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(apply()));
  if (mField->singleType() != WB_SF_STRING) {
    if (mMultipleValue)
      mComboBox->setCurrentText(mMultipleValue->itemToString(mIndex));
    else
      mComboBox->setCurrentText(mValue->toString());
  } else {  // In case of MF/SF_STRING we don't want to display the starting and ending '"'
    if (mMultipleValue)
      mComboBox->setCurrentText(mMultipleValue->itemToString(mIndex).chopped(1).remove(0, 1));
    else
      mComboBox->setCurrentText(mValue->toString().chopped(1).remove(0, 1));
  }
  connect(mComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(apply()), Qt::UniqueConnection);
}
