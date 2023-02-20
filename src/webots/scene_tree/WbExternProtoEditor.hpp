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

#ifndef WB_EXTERN_PROTO_EDITOR_HPP
#define WB_EXTERN_PROTO_EDITOR_HPP

//
// Description: editor for EXTERNPROTO panel
//

#include "WbValueEditor.hpp"

class QPushButton;

class WbExternProtoEditor : public WbValueEditor {
  Q_OBJECT

public:
  explicit WbExternProtoEditor(QWidget *parent = NULL);
  virtual ~WbExternProtoEditor();

  void recursiveBlockSignals(bool block) override {}
  void edit(WbNode *node, WbField *field, int index) override {}
  void edit(bool copyOriginalValue) override {}
  void stopEditing() override;
  QWidget *lastEditorWidget() override { return NULL; }

  void updateContents();

public slots:
  void cleanValue() override {}

protected:
  void resetFocus() override {}

protected slots:
  void apply() override {}

private slots:
  void insertImportableExternProto();
  void removeImportableExternProto();

signals:
  void changed(bool value);

private:
  QPushButton *mInsertButton;

  void takeKeyboardFocus() override {}
  void buttonCallback();

  void clearLayout();
};

#endif
