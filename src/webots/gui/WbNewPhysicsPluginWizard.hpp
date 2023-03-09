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

#ifndef WB_NEW_PHYSICS_PLUGIN_WIZARD_HPP
#define WB_NEW_PHYSICS_PLUGIN_WIZARD_HPP

//
// Description: Wizard for creating a new physics plugin
//

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QWizard>

class WbLineEdit;
class QLabel;
class QButtonGroup;

class WbNewPhysicsPluginWizard : public QWizard {
  Q_OBJECT

public:
  explicit WbNewPhysicsPluginWizard(QWidget *parent = NULL);

  // check user inputs
  void accept() override;
  bool validateCurrentPage() override;

  // user wants to edit the new physics plugin file
  bool needsEdit() const;

  // name of the new physic lugin file
  const QString &physicsPluginName() const;

protected:
private slots:

private:
  bool mNeedsEdit;

  QString mPhysicsDir;
  QString mPhysicsFullPath;
  QString mMakefileFullPath;
  int mLanguage;
  QLabel *mFilesLabel;
  WbLineEdit *mNameEdit;
  QButtonGroup *mButtonGroup;
  QCheckBox *mEditCheckBox;

  void updateUI();
  QWizardPage *createIntroPage();
  QWizardPage *createLanguagePage();
  QWizardPage *createNamePage();
  QWizardPage *createConclusionPage();
};

#endif
