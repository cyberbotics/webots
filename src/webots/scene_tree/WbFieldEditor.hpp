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

#ifndef WB_FIELD_EDITOR_HPP
#define WB_FIELD_EDITOR_HPP

//
// Description: editor for editing a WbField at the bottom of the Scene Tree
//

#include <QtCore/QMultiMap>
#include <QtWidgets/QWidget>

#include "../../../include/controller/c/webots/supervisor.h"

class WbExternProtoEditor;
class WbField;
class WbNode;
class WbValueEditor;

class QLabel;
class QStackedLayout;

class WbFieldEditor : public QWidget {
  Q_OBJECT

public:
  explicit WbFieldEditor(QWidget *parent = NULL);
  virtual ~WbFieldEditor();

  // start editing this field
  void editField(WbNode *node, WbField *field, int item = -1);

  // start editing the EXTERNPROTO panel
  void editExternProto();

  // update displayed values
  void updateValue(bool copyOriginalValue = true);

  // remove focus from children widgets
  void resetFocus();

  // apply changes to the field value
  void applyChanges();

  void setTitle(const QString &title);

  QWidget *lastEditorWidget();

  WbValueEditor *currentEditor() const;

signals:
  // emitted when the file has to be opened in text editor
  // title can be used for example for showing human-readable file name in case of cached assets
  void editRequested(const QString &fileName);
  // emitted when the dictionary needs to be updated (e.g., a DEF name was changed)
  void dictionaryUpdateRequested();
  void valueChanged();

protected:
  WbNode *mNode;

private:
  QMultiMap<WbFieldType, WbValueEditor *> mEditors;
  WbExternProtoEditor *mExternProtoEditor;
  QStackedLayout *mStackedLayout;
  QWidget *mEmptyPane;
  WbField *mField;
  int mItem;
  WbNode *mNodeItem;
  bool mIsValidItemIndex;

  QLabel *mTitleLabel;

  QString nodeAsTitle(WbNode *node);
  void updateTitle();
  void setCurrentWidget(int index);
  void setCurrentWidget(WbValueEditor *editor);
  void setTransformActionVisibile(bool visible);
  void computeFieldInformation();

private slots:
  void invalidateValue();
  void updateResetButton();
  void refreshExternProtoEditor();
};

#endif
