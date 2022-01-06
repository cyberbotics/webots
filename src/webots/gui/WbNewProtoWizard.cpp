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

#include "WbApplicationInfo.hpp"
#include "WbFileUtil.hpp"
#include "WbLineEdit.hpp"
#include "WbMessageBox.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"
#include "WbVersion.hpp"

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

  mProceduralCheckBox->setChecked(true);
  mProceduralCheckBox->setText("Allow template scripting");
  mStaticCheckBox->setChecked(true);
  mStaticCheckBox->setText("Make static");
  mNonDeterministic->setChecked(false);
  mNonDeterministic->setText("The result of the PROTO may vary in a non-deterministic way (time-based seed, ...)");
  mHiddenCheckBox->setChecked(false);
  mHiddenCheckBox->setText("Hide from new node menu");

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Create a new PROTO"));
}

void WbNewProtoWizard::updateUI() {
  // update paths
  mProtoDir = WbProject::current()->protosPath();
  mProtoFullPath = mProtoDir + mNameEdit->text() + ".proto";

  // update check box message
  mEditCheckBox->setText(tr("Open '%1.proto' in Text Editor.").arg(mNameEdit->text()));

  QString files = mProtoDir + "\n" + mProtoFullPath;
  mFilesLabel->setText(QDir::toNativeSeparators(files));
}

bool WbNewProtoWizard::validateCurrentPage() {
  updateUI();

  if (currentId() == NAME)
    return !mNameEdit->text().isEmpty();

  return true;
}

void WbNewProtoWizard::accept() {
  // create protos directory
  bool success = QDir::root().mkpath(mProtoDir);

  // copy PROTO from template and rename
  QString src = WbStandardPaths::templatesPath() + "protos/template.proto";
  success = WbFileUtil::copyAndReplaceString(src, mProtoFullPath, "template", mNameEdit->text()) && success;

  if (success) {
    QFile file(protoName());
    if (!file.open(QIODevice::ReadWrite)) {
      WbMessageBox::warning(tr("PROTO template not found."), this, tr("PROTO creation failed"));
      return;
    }

    QByteArray protoContent = file.readAll();

    QString tags;
    if (mProceduralCheckBox->isChecked())
      tags = "# template language: javascript\n# tags: ";
    else
      tags = "# tags: ";

    if (mStaticCheckBox->isChecked())
      tags += "static, ";
    if (mNonDeterministic->isChecked())
      tags += "nonDeterministic, ";
    if (mHiddenCheckBox->isChecked())
      tags += "hidden, ";

    if (mStaticCheckBox->isChecked() || mNonDeterministic->isChecked() || mHiddenCheckBox->isChecked())
      tags.chop(2);
    else
      tags.chop(QString("# tags: ").length());

    QString version = WbApplicationInfo::version().toString(false);
    protoContent.replace(QByteArray("%tags%"), tags.toUtf8());
    protoContent.replace(QByteArray("%name%"), mNameEdit->text().toUtf8());
    protoContent.replace(QByteArray("%release%"), version.toUtf8());

    file.seek(0);
    file.write(protoContent);
    file.close();
  }

  if (!success)
    WbMessageBox::warning(tr("Some directories or files could not be created."), this, tr("PROTO creation failed"));

  mNeedsEdit = mEditCheckBox->isChecked();

  QDialog::accept();
}

bool WbNewProtoWizard::needsEdit() const {
  return mNeedsEdit;
}

const QString &WbNewProtoWizard::protoName() const {
  return mProtoFullPath;
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

  mHiddenCheckBox = new QCheckBox(page);
  mStaticCheckBox = new QCheckBox(page);
  mNonDeterministic = new QCheckBox(page);
  mProceduralCheckBox = new QCheckBox(page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mProceduralCheckBox);
  layout->addWidget(mStaticCheckBox);
  layout->addWidget(mNonDeterministic);
  layout->addWidget(mHiddenCheckBox);

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
