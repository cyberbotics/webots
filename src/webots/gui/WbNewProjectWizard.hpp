// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_NEW_PROJECT_WIZARD_HPP
#define WB_NEW_PROJECT_WIZARD_HPP

//
// Description: Wizard for creating a new Webots project
//

#include <QtWidgets/QWizard>

class WbLineEdit;
class WbProject;

class QLabel;
class QCheckBox;

class WbNewProjectWizard : public QWizard {
  Q_OBJECT

public:
  explicit WbNewProjectWizard(QWidget *parent = NULL);
  virtual ~WbNewProjectWizard();

  // check user inputs
  void accept() override;
  bool validateCurrentPage() override;

  // new project was created correctly
  bool isValidProject() const;

  // path of the new created world file
  QString newWorldFile() const;

protected:
private slots:
  void chooseDirectory();

private:
  bool mIsValidProject;

  WbProject *mProject;
  WbLineEdit *mDirEdit;
  WbLineEdit *mWorldEdit;
  QCheckBox *mBackgroundCheckBox;
  QCheckBox *mViewPointCheckBox;
  QCheckBox *mDirectionalLightCheckBox;
  QCheckBox *mArenaCheckBox;
  QLabel *mFilesLabel;

  void updateUI();
  QString proposeNewProjectPath() const;
  QWizardPage *createIntroPage();
  QWizardPage *createDirectoryPage();
  QWizardPage *createWorldPage();
  QWizardPage *createConclusionPage();
};

#endif
