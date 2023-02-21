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

#ifndef WB_NEW_PROJECT_WIZARD_HPP
#define WB_NEW_PROJECT_WIZARD_HPP

//
// Description: Wizard for creating a new Webots project
//

#include "WbNewWorldWizard.hpp"

class WbLineEdit;
class WbProject;

class WbNewProjectWizard : public WbNewWorldWizard {
  Q_OBJECT

public:
  explicit WbNewProjectWizard(QWidget *parent = NULL);
  virtual ~WbNewProjectWizard();

  void accept() override;
  bool validateCurrentPage() override;

protected:
  const int directoryId() const { return 2; }
  virtual const int worldId() const override { return 3; }
  virtual const int conclusionId() const override { return 4; }

  const QString title() const override { return tr("Create a Webots project directory"); }
  const QString introTitle() const override { return tr("New project creation"); }
  const QString introText() const override { return tr("This wizard will help you creating a new project folder."); }
  const QString conclusionText() const override { return tr("The following folders and files will be created:"); }
  void updateUI() override;

private slots:
  void chooseDirectory();

private:
  WbProject *mProject;
  WbLineEdit *mDirEdit;

  QString proposeNewProjectPath() const;
  QWizardPage *createDirectoryPage();
};

#endif
