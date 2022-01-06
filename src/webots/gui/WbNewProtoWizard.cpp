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

#include "WbNewProtoWizard.hpp"

#include "WbFileUtil.hpp"
#include "WbLineEdit.hpp"
#include "WbMessageBox.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"

#include <QtGui/QRegExpValidator>

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizardPage>

enum { INTRO, LANGUAGE, NAME, CONCLUSION };

WbNewProtoWizard::WbNewProtoWizard(QWidget *parent) : QWizard(parent) {
  mNeedsEdit = false;

  addPage(createIntroPage());
  addPage(createNamePage());
  addPage(createTagsPage());
  addPage(createConclusionPage());

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Create a new PROTO"));
}

void WbNewProtoWizard::updateUI() {
  mLanguage = mButtonGroup->checkedId();

  // update paths
  mProtoDir = WbProject::current()->protosPath() + mNameEdit->text() + "/";
  mProtoFullPath = mProtoDir + mNameEdit->text() + ".proto"];

  // update check box message
  mEditCheckBox->setText(tr("Open '%1%2' in Text Editor.").arg(mNameEdit->text()).arg(EXT[mLanguage]));

  QString files = mPhysicsDir + "\n" + mPhysicsFullPath + "\n" + mMakefileFullPath;
  mFilesLabel->setText(QDir::toNativeSeparators(files));
}

bool WbNewProtoWizard::validateCurrentPage() {
  updateUI();

  if (currentId() == NAME)
    return !mNameEdit->text().isEmpty();

  return true;
}

void WbNewProtoWizard::accept() {
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

bool WbNewProtoWizard::needsEdit() const {
  return mNeedsEdit;
}

const QString &WbNewProtoWizard::physicsPluginName() const {
  return mPhysicsFullPath;
}

QWizardPage *WbNewProtoWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("New PROTO creation"));

  QLabel *label = new QLabel(tr("This wizard will help you creating a new PROTO."), page);

  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);

  return page;
}

QWizardPage *WbNewProtoWizard::createNamePage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Name selection"));
  page->setSubTitle(tr("Please choose a name for your PROTO node."));

  QLabel *nameLabel = new QLabel(tr("PROTO name:"), page);
  mNameEdit = new WbLineEdit("my_proto", page);
  mNameEdit->setValidator(new QRegExpValidator(QRegExp("[a-zA-Z0-9_-]*"), page));
  nameLabel->setBuddy(mNameEdit);

  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(nameLabel);
  layout->addWidget(mNameEdit);

  return page;
}

QWizardPage *WbNewProtoWizard::createTagsPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Tags selection"));
  page->setSubTitle(tr("Please choose the tags of your PROTO."));

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

QWizardPage *WbNewProtoWizard::createConclusionPage() {
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
