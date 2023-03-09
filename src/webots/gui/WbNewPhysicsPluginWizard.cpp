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

#include "WbNewPhysicsPluginWizard.hpp"

#include "WbFileUtil.hpp"
#include "WbLineEdit.hpp"
#include "WbMessageBox.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"

#include <QtGui/QRegularExpressionValidator>

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QLabel>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizardPage>

static const QString LANG[] = {"C", "C++"};
static const QString EXT[] = {".c", ".cpp"};
static const int NUM = sizeof(LANG) / sizeof(LANG[0]);

enum { INTRO, LANGUAGE, NAME, CONCLUSION };

WbNewPhysicsPluginWizard::WbNewPhysicsPluginWizard(QWidget *parent) : QWizard(parent) {
  mNeedsEdit = false;

  addPage(createIntroPage());
  addPage(createLanguagePage());
  addPage(createNamePage());
  addPage(createConclusionPage());

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Create a new physics plugin"));
}

void WbNewPhysicsPluginWizard::updateUI() {
  mLanguage = mButtonGroup->checkedId();

  // update paths
  mPhysicsDir = WbProject::current()->physicsPluginsPath() + mNameEdit->text() + "/";
  mPhysicsFullPath = mPhysicsDir + mNameEdit->text() + EXT[mLanguage];
  mMakefileFullPath = mPhysicsDir + "Makefile";

  // update check box message
  mEditCheckBox->setText(tr("Open '%1%2' in Text Editor.").arg(mNameEdit->text()).arg(EXT[mLanguage]));

  QString files = mPhysicsDir + "\n" + mPhysicsFullPath + "\n" + mMakefileFullPath;
  mFilesLabel->setText(QDir::toNativeSeparators(files));
}

bool WbNewPhysicsPluginWizard::validateCurrentPage() {
  updateUI();

  if (currentId() == NAME) {
    if (mNameEdit->text().isEmpty()) {
      WbMessageBox::warning(tr("Please specify a physics plugin name."), this, tr("Invalid physics plugin name"));
      return false;
    }
    if (QDir(WbProject::current()->physicsPluginsPath() + mNameEdit->text()).exists()) {
      WbMessageBox::warning(tr("A physics plugin with this name already exists, please choose a different name."), this,
                            tr("Invalid physics plugin name"));
      return false;
    }
  }

  return true;
}

void WbNewPhysicsPluginWizard::accept() {
  // create physics directory
  bool success = QDir::root().mkpath(mPhysicsDir);

  // copy controller from template (and replace "template")
  QString src = WbStandardPaths::templatesPath() + "plugins/physics/template.c";
  success = WbFileUtil::copyAndReplaceString(src, mPhysicsFullPath, "template", mNameEdit->text()) && success;

  // copy Makefile from template
  src = WbStandardPaths::templatesPath() + "plugins/physics/Makefile";
  success = QFile::copy(src, mMakefileFullPath) && success;

  if (!success)
    WbMessageBox::warning(tr("Some directories or files could not be created."), this, tr("Physics creation failed"));

  mNeedsEdit = mEditCheckBox->isChecked();

  QDialog::accept();
}

bool WbNewPhysicsPluginWizard::needsEdit() const {
  return mNeedsEdit;
}

const QString &WbNewPhysicsPluginWizard::physicsPluginName() const {
  return mPhysicsFullPath;
}

QWizardPage *WbNewPhysicsPluginWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("New physics plugin creation"));

  QLabel *label = new QLabel(tr("This wizard will help you creating a new physics plugin."), page);

  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);

  return page;
}

QWizardPage *WbNewPhysicsPluginWizard::createLanguagePage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Language selection"));
  page->setSubTitle(tr("Please choose the language for your physics plugin."));

  QVBoxLayout *layout = new QVBoxLayout(page);
  mButtonGroup = new QButtonGroup(page);
  QRadioButton *buttons[NUM];
  for (int i = 0; i < NUM; i++) {
    buttons[i] = new QRadioButton(LANG[i], page);
    layout->addWidget(buttons[i]);
    mButtonGroup->addButton(buttons[i], i);
  }

  buttons[0]->setChecked(true);

  return page;
}

QWizardPage *WbNewPhysicsPluginWizard::createNamePage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Name selection"));
  page->setSubTitle(tr("Please choose a name for your physics plugin."));

  QLabel *nameLabel = new QLabel(tr("Plugin name:"), page);
  mNameEdit = new WbLineEdit("my_physics", page);
  mNameEdit->setValidator(new QRegularExpressionValidator(QRegularExpression("[a-zA-Z0-9_-]*"), page));
  nameLabel->setBuddy(mNameEdit);

  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(nameLabel);
  layout->addWidget(mNameEdit);

  return page;
}

QWizardPage *WbNewPhysicsPluginWizard::createConclusionPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Conclusion"));
  page->setSubTitle(tr("The following directory and files will be generated."));

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
