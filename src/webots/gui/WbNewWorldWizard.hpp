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

  // check user inputs
  void accept() override;
  bool validateCurrentPage() override;

  // path of the new created world file
  QString fileName() const;

private:
  WbLineEdit *mNameEdit;
  QCheckBox *mBackgroundCheckBox;
  QCheckBox *mViewPointCheckBox;
  QCheckBox *mDirectionalLightCheckBox;
  QCheckBox *mArenaCheckBox;
  QLabel *mFileLabel;

  void updateUI();
  QWizardPage *createIntroPage();
  QWizardPage *createFilePage();
  QWizardPage *createWorldPage();
  QWizardPage *createConclusionPage();
};

#endif
