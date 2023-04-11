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

#include "WbNewControllerWizard.hpp"

#include "WbDesktopServices.hpp"
#include "WbFileUtil.hpp"
#include "WbLanguage.hpp"
#include "WbLineEdit.hpp"
#include "WbMessageBox.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QUrl>

#include <QtGui/QRegularExpressionValidator>

#include <QtWidgets/QLabel>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizardPage>

static WbLanguage *gLanguages[] = {WbLanguage::findByCode(WbLanguage::C), WbLanguage::findByCode(WbLanguage::CPP),
                                   WbLanguage::findByCode(WbLanguage::JAVA), WbLanguage::findByCode(WbLanguage::PYTHON),
                                   WbLanguage::findByCode(WbLanguage::MATLAB)};

static const int NUM_LANGUAGES = sizeof(gLanguages) / sizeof(gLanguages[0]);

WbNewControllerWizard::WbNewControllerWizard(QWidget *parent) : QWizard(parent) {
  mNeedsEdit = false;
  mLanguage = gLanguages[0];
  setPage(INTRO, createIntroPage());
  setPage(LANGUAGE, createLanguagePage());
#ifdef _WIN32
  setPage(IDE, createIdePage());
#endif
  setPage(NAME, createNamePage());
  setPage(CONCLUSION, createConclusionPage());
  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Create a new robot controller."));
}

void WbNewControllerWizard::updateUI() {
  mLanguage = gLanguages[mLanguageButtonGroup->checkedId()];
  // update paths
  mControllerDir = WbProject::current()->controllersPath() + mNameEdit->text() + "/";
  mControllerFullPath = mControllerDir + mNameEdit->text() + "." + mLanguage->defaultFileSuffix();
  mIdeProjectFullPath.clear();
  if (mLanguage->isCompilable() && mLanguage->code() != WbLanguage::JAVA) {
#ifdef _WIN32
    if (mIdeButtonGroup->checkedId() == 1) {  // Microsoft Visual Studio
      mIdeProjectFullPath << mControllerDir + mNameEdit->text() + ".sln";
      mIdeProjectFullPath << mControllerDir + mNameEdit->text() + ".vcxproj";
      mIdeProjectFullPath << mControllerDir + mNameEdit->text() + ".vcxproj.filter";
      mIdeProjectFullPath << mControllerDir + mNameEdit->text() + ".vcxproj.user";
    } else
#endif
      mIdeProjectFullPath << mControllerDir + "Makefile";
  }

#ifdef _WIN32
  // update check box message
  if (mIdeButtonGroup->checkedId() == 1)  // Microsoft Visual Studio
    mEditCheckBox->setText(tr("Open '%1.sln' in Microsoft Visual Studio (the controller need to be set to <extern> to be "
                              "able to launch the controller from Microsoft Visual Studio.")
                             .arg(mNameEdit->text()));
  else
#endif
    mEditCheckBox->setText(tr("Open '%1.%2' in Text Editor.").arg(mNameEdit->text()).arg(mLanguage->defaultFileSuffix()));
  // show what files will be created
  QString newFiles = mControllerDir + "\n" + mControllerFullPath;
  foreach (const QString &ideProjectFullPath, mIdeProjectFullPath)
    newFiles += "\n" + ideProjectFullPath;
  mFilesLabel->setText(QDir::toNativeSeparators(newFiles));
}

bool WbNewControllerWizard::validateCurrentPage() {
  updateUI();
  // propose a default controller name
  if (currentId() == LANGUAGE) {
    if (mLanguage->code() != WbLanguage::JAVA) {
      if (mNameEdit->text() == "MyController")
        mNameEdit->setText("my_controller");
    } else if (mNameEdit->text() == "my_controller")
      mNameEdit->setText("MyController");
  }
  if (currentId() == NAME) {
    if (mNameEdit->text().isEmpty()) {
      WbMessageBox::warning(tr("Please specify a controller name."), this, tr("Invalid controller name"));
      return false;
    }
    if (QDir(WbProject::current()->controllersPath() + mNameEdit->text()).exists()) {
      WbMessageBox::warning(tr("A controller with this name already exists, please choose a different name."), this,
                            tr("Invalid controller name"));
      return false;
    }
  }
  return true;
}

void WbNewControllerWizard::accept() {
  // create controller directory
  bool success = QDir::root().mkpath(mControllerDir);
  // copy controller from template (and replace "template")
  const QString &src = WbStandardPaths::templatesPath() + "controllers/";
  success = WbFileUtil::copyAndReplaceString(src + "template." + mLanguage->defaultFileSuffix(), mControllerFullPath,
                                             "template", mNameEdit->text()) &&
            success;
#ifdef _WIN32
  mSlnFile = "";
#endif
  if (mLanguage->isCompilable() && mLanguage->code() != WbLanguage::JAVA) {  // copy Makefile from template
    foreach (const QString &ideProjectFullPath, mIdeProjectFullPath) {
      if (ideProjectFullPath.endsWith("/Makefile"))
        success = QFile::copy(src + "Makefile", ideProjectFullPath) && success;
#ifdef _WIN32
      else if (ideProjectFullPath.endsWith(".sln")) {
        mSlnFile = ideProjectFullPath;
        success = WbFileUtil::copyAndReplaceString(src + mLanguage->defaultFileSuffix() + ".sln", ideProjectFullPath,
                                                   "template", mNameEdit->text()) &&
                  success;
      } else if (ideProjectFullPath.endsWith(".vcxproj"))
        success = WbFileUtil::copyAndReplaceString(src + mLanguage->defaultFileSuffix() + ".vcxproj", ideProjectFullPath,
                                                   "template", mNameEdit->text()) &&
                  success;
      else if (ideProjectFullPath.endsWith(".vcxproj.filter"))
        success = WbFileUtil::copyAndReplaceString(src + mLanguage->defaultFileSuffix() + ".vcxproj.filter", ideProjectFullPath,
                                                   "template", mNameEdit->text()) &&
                  success;
      else if (ideProjectFullPath.endsWith(".vcxproj.user"))
        success = WbFileUtil::copyAndReplaceString(src + mLanguage->defaultFileSuffix() + ".vcxproj.user", ideProjectFullPath,
                                                   "template", mNameEdit->text()) &&
                  success;
#endif
    }
  }
  if (!success)
    WbMessageBox::warning(tr("Some directories or files could not be created."), this, tr("Controller creation failed"));
#ifdef _WIN32
  if (mIdeButtonGroup->checkedId() == 1 && mEditCheckBox->isChecked()) {  // Microsoft Visual Studio
    WbDesktopServices::openUrl(QUrl::fromLocalFile(mSlnFile).toString());
    mNeedsEdit = false;
  } else
#endif
    mNeedsEdit = mEditCheckBox->isChecked();
  QDialog::accept();
}

bool WbNewControllerWizard::needsEdit() const {
  return mNeedsEdit;
}

const QString &WbNewControllerWizard::controllerName() const {
  return mControllerFullPath;
}

QWizardPage *WbNewControllerWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("New controller creation"));
  QLabel *label = new QLabel(tr("This wizard will help you creating a new controller."), page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);
  return page;
}

QWizardPage *WbNewControllerWizard::createLanguagePage() {
  WbLanguageWizardPage *page = new WbLanguageWizardPage(this);
  page->setTitle(tr("Language selection"));
  page->setSubTitle(tr("Please choose the language for your controller program."));
  QVBoxLayout *layout = new QVBoxLayout(page);
  mLanguageButtonGroup = new QButtonGroup(page);
  page->setButtonGroup(mLanguageButtonGroup);
  QRadioButton *buttons[NUM_LANGUAGES];
  for (int i = 0; i < NUM_LANGUAGES; i++) {
    buttons[i] = new QRadioButton(gLanguages[i]->name(), page);
    layout->addWidget(buttons[i]);
    mLanguageButtonGroup->addButton(buttons[i], i);
  }
  buttons[0]->setChecked(true);
  return page;
}

#ifdef _WIN32
QWizardPage *WbNewControllerWizard::createIdePage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("IDE selection"));
  page->setSubTitle(tr("Please choose the Integrated Development Environment (IDE) for your controller program."));
  QVBoxLayout *layout = new QVBoxLayout(page);
  mIdeButtonGroup = new QButtonGroup(page);
  QRadioButton *buttons[2];
  buttons[0] = new QRadioButton(tr("Webots (gcc / Makefile)"), page);
  layout->addWidget(buttons[0]);
  mIdeButtonGroup->addButton(buttons[0], 0);
  buttons[1] = new QRadioButton(tr("Microsoft Visual Studio"), page);
  layout->addWidget(buttons[1]);
  mIdeButtonGroup->addButton(buttons[1], 1);
  buttons[0]->setChecked(true);
  return page;
}
#endif

QWizardPage *WbNewControllerWizard::createNamePage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("Name selection"));
  page->setSubTitle(tr("Please choose a name for your controller program."));
  QLabel *nameLabel = new QLabel(tr("Controller name:"), page);
  mNameEdit = new WbLineEdit("my_controller", page);
  mNameEdit->setValidator(new QRegularExpressionValidator(QRegularExpression("[a-zA-Z0-9_-]*"), page));
  nameLabel->setBuddy(mNameEdit);
  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(nameLabel);
  layout->addWidget(mNameEdit);
  return page;
}

QWizardPage *WbNewControllerWizard::createConclusionPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("Conclusion"));
  page->setSubTitle(tr("The following directory and files will be created:"));
  // add files path to be created
  mFilesLabel = new QLabel(page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mFilesLabel);
  // add space
  layout->addSpacing(30);
  // add check box
  mEditCheckBox = new QCheckBox(page);
  mEditCheckBox->setChecked(true);
  layout->addWidget(mEditCheckBox);
  return page;
}
