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

#ifndef WB_ADD_INERTIA_MATRIX_DIALOG_HPP
#define WB_ADD_INERTIA_MATRIX_DIALOG_HPP

//
// Description: Dialog that lets the user choose either a default inertia matrix (3x3 identity matrix) or a matrix computed by
// using the solid's bounding object
//

#include <QtWidgets/QDialog>

class QTreeWidget;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QGroupBox;

class WbAddInertiaMatrixDialog : public QDialog {
  Q_OBJECT

public:
  explicit WbAddInertiaMatrixDialog(bool validBoudingObject, QWidget *parent = NULL);
  enum MatrixType { IDENTITY_MATRIX, BOUNDING_OBJECT_BASED };
  int inertiaMatrixType() const;

private slots:
  void updateItemInfo();

private:
  QTreeWidget *mTree;
  QLabel *mPixmapLabel;
  QPlainTextEdit *mInfoText;
  QPushButton *mAddButton;
  QGroupBox *mGroupBox;
  bool mValidBoundingObject;

  void buildTree();
};

#endif
