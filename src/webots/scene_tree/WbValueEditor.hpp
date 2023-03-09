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

#ifndef WB_VALUE_EDITOR_HPP
#define WB_VALUE_EDITOR_HPP

//
// Description: Abstract base class for all value editors of the Scene Tree
//
// Inherited by:
//    WbBoolEditor, WbColorEditor, WbDoubleEditor, WbIntEditor, WbRotationEditor,
//    WbStringEditor, WbVector2Editor, WbVector3Editor
//

#include <QtWidgets/QWidget>

class QComboBox;
class QGridLayout;

class WbNode;
class WbField;
class WbValue;
class WbSingleValue;
class WbMultipleValue;
class WbVariant;

class WbValueEditor : public QWidget {
  Q_OBJECT

public:
  virtual ~WbValueEditor();

  // start editing this value: this editor is shown
  virtual void edit(WbNode *node, WbField *field, int index);

  // copy from WbValue to editor widgets
  virtual void edit(bool copyOriginalValue) = 0;

  // stop editing: this editor is no longer visible
  virtual void stopEditing();

  // remove focus from children widgets
  virtual void resetFocus() = 0;

  // block signals of the widget and children
  virtual void recursiveBlockSignals(bool block) = 0;

  // set keyboard focus to first spinbox or line edit
  virtual void takeKeyboardFocus() = 0;

  virtual QWidget *lastEditorWidget() = 0;

signals:
  // the value was invalidated, e.g. deleted
  void valueInvalidated();
  void valueChanged();

public slots:
  virtual void cleanValue();
  // copy from editor widgets to WbValue if editor value has changed or world is being to be saved
  virtual void applyIfNeeded() { apply(); }

protected:
  explicit WbValueEditor(QWidget *parent = NULL);

  WbNode *node() const { return mNode; }
  WbField *field() const { return mField; }

  // return as WbSingleValue if the value is single
  WbSingleValue *singleValue() const { return mSingleValue; }

  // return as WbMultipleValue if the value is multiple
  WbMultipleValue *multipleValue() const { return mMultipleValue; }

  // return the item index in case the value is multiple
  int index() const { return mIndex; }

  WbVariant *mNewValue;
  WbVariant *mPreviousValue;
  QComboBox *mComboBox;
  QGridLayout *mLayout;

protected slots:
  virtual void apply();

private slots:
  void updateComboBoxIndex();

private:
  WbNode *mNode;
  WbField *mField;
  WbValue *mValue;
  WbSingleValue *mSingleValue;
  WbMultipleValue *mMultipleValue;
  int mIndex;
};

#endif
