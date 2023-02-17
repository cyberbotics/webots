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

#ifndef WB_NEW_WORLD_WIZARD_HPP
#define WB_NEW_WORLD_WIZARD_HPP

//
// Description: Wizard for creating a new Webots world
//

#include <QtWidgets/QWizard>

class WbLineEdit;

class QLabel;
class QCheckBox;

class WbNewWorldWizard : public QWizard {
  Q_OBJECT

public:
  explicit WbNewWorldWizard(QWidget *parent = NULL);
  virtual ~WbNewWorldWizard();

  int exec() override;
  void accept() override;
  bool validateCurrentPage() override;
  QString fileName() const;

protected:
  virtual const int introId() const { return 1; }
  virtual const int worldId() const { return 2; }
  virtual const int conclusionId() const { return 3; }
  virtual const QString title() const { return tr("Create a Webots world file"); }
  virtual const QString introTitle() const { return tr("New world creation"); }
  virtual const QString introText() const { return tr("This wizard will help you creating a new world file."); }
  virtual const QString conclusionText() const { return tr("The following file will be created:"); }
  virtual void updateUI();
  void updateWorldUI();
  bool validateWorldPage();
  void createWorldFile();

  WbLineEdit *mWorldEdit;
  QCheckBox *mBackgroundCheckBox;
  QCheckBox *mViewPointCheckBox;
  QCheckBox *mDirectionalLightCheckBox;
  QCheckBox *mArenaCheckBox;
  QLabel *mFilesLabel;

private:
  QWizardPage *createIntroPage();
  QWizardPage *createWorldPage();
  QWizardPage *createConclusionPage();
};

#endif
