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

#ifndef WB_NEW_PROTO_WIZARD_HPP
#define WB_NEW_PROTO_WIZARD_HPP

//
// Description: Wizard for the creation of a new PROTO
//

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QWizard>

class WbLineEdit;
class QLabel;

class WbNewProtoWizard : public QWizard {
  Q_OBJECT

public:
  explicit WbNewProtoWizard(QWidget *parent = NULL);

  // check user inputs
  void accept() override;
  bool validateCurrentPage() override;

  // user wants to edit the new PROTO file
  bool needsEdit() const;

  // name of the new PROTO file
  const QString &protoName() const;

protected:
private slots:
  void updateCheckBox(int state);

private:
  bool mNeedsEdit;
  bool mIsProtoNode;
  int mCategory;
  bool mRetrievalTriggered;

  QString mProtoDir;
  QString mProtoFullPath;
  QString mBaseNode;
  QLabel *mFilesLabel;
  WbLineEdit *mNameEdit;
  QPlainTextEdit *mDescription;
  QCheckBox *mEditCheckBox;
  QCheckBox *mHiddenCheckBox;
  QCheckBox *mNonDeterministicCheckbox;
  QCheckBox *mProceduralCheckBox;
  QTreeWidget *mTree;
  QWidget *mFields;
  QLineEdit *mFindLineEdit;
  QVector<QCheckBox *> mExposedFieldCheckBoxes;

  void updateUI();
  void updateBaseNode();
  void updateNodeTree();
  QWizardPage *createIntroPage();
  QWizardPage *createNamePage();
  QWizardPage *createTagsPage();
  QWizardPage *createBaseTypeSelectorPage();
  QWizardPage *createConclusionPage();

  bool generateProto();
};

#endif
