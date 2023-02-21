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

#ifndef WB_INSERT_EXTERN_PROTO_DIALOG_HPP
#define WB_INSERT_EXTERN_PROTO_DIALOG_HPP

//
// Description: Dialog that lets the user choose a PROTO to declare as EXTERNPROTO
//

#include <QtWidgets/QDialog>

class QLineEdit;
class QTreeWidget;
class QPushButton;

class WbInsertExternProtoDialog : public QDialog {
  Q_OBJECT

public:
  explicit WbInsertExternProtoDialog(QWidget *parent = NULL);

public slots:
  void accept() override;

private slots:
  void updateProtoTree();
  void updateSelection();

private:
  // cppcheck 2.10 seems to give a series of false positives here that we want to silence:
  // cppcheck-suppress unsafeClassCanLeak
  QTreeWidget *mTree;
  // cppcheck-suppress unsafeClassCanLeak
  QLineEdit *mSearchBar;
  // cppcheck-suppress unsafeClassCanLeak
  QPushButton *mInsertButton;
  // cppcheck-suppress unsafeClassCanLeak
  QPushButton *mCancelButton;
  bool mRetrievalTriggered;
  QString mProto;
  QString mPath;
};

#endif
