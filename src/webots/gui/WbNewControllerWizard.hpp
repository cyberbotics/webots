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

#ifndef WB_NEW_CONTROLLER_WIZARD_HPP
#define WB_NEW_CONTROLLER_WIZARD_HPP

//
// Description: Wizard for creating a new controller
//

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QWizard>

class WbLanguage;
class WbLineEdit;

class QLabel;

class WbNewControllerWizard : public QWizard {
  Q_OBJECT

public:
  enum {
    INTRO,
    LANGUAGE,
#ifdef _WIN32
    IDE,
#endif
    NAME,
    CONCLUSION
  };

  explicit WbNewControllerWizard(QWidget *parent = NULL);

  // check user inputs
  void accept() override;
  bool validateCurrentPage() override;

  // user wants to edit the new controller file
  bool needsEdit() const;

  // name of the new controller file
  const QString &controllerName() const;

private:
  bool mNeedsEdit;

  WbLanguage *mLanguage;
  QString mControllerDir;
  QString mControllerFullPath;
  QStringList mIdeProjectFullPath;
  QLabel *mFilesLabel;
  WbLineEdit *mNameEdit;
  QButtonGroup *mLanguageButtonGroup;
#ifdef _WIN32
  QButtonGroup *mIdeButtonGroup;
  QString mSlnFile;
#endif
  QCheckBox *mEditCheckBox;

  void updateUI();
  QWizardPage *createIntroPage();
  QWizardPage *createLanguagePage();
#ifdef _WIN32
  QWizardPage *createIdePage();
#endif
  QWizardPage *createNamePage();
  QWizardPage *createConclusionPage();
};

class WbLanguageWizardPage : public QWizardPage {
  Q_OBJECT
public:
  explicit WbLanguageWizardPage(QWidget *parent = NULL) : QWizardPage(parent), mButtonGroup(NULL) {}
  int nextId() const override {
#ifdef _WIN32
    if (mButtonGroup->checkedId() < 2)
      return WbNewControllerWizard::IDE;
    else
#endif
      return WbNewControllerWizard::NAME;
  }
  void setButtonGroup(const QButtonGroup *buttonGroup) {
    mButtonGroup = buttonGroup;
  }

private:
  const QButtonGroup *mButtonGroup;
};

#endif
