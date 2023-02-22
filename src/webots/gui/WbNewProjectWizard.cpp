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

#include "WbNewProjectWizard.hpp"

#include "WbApplicationInfo.hpp"
#include "WbFileUtil.hpp"
#include "WbLineEdit.hpp"
#include "WbMessageBox.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoManager.hpp"
#include "WbStandardPaths.hpp"

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizard>

WbNewProjectWizard::WbNewProjectWizard(QWidget *parent) : WbNewWorldWizard(parent) {
  setPage(directoryId(), createDirectoryPage());
  const QString path = proposeNewProjectPath();
  mDirEdit->setText(QDir::toNativeSeparators(path));
  mProject = new WbProject(path);
}

WbNewProjectWizard::~WbNewProjectWizard() {
}

QString WbNewProjectWizard::proposeNewProjectPath() const {
  QString path;
  // if current project is in Webots installation dir
  if (WbProject::current()->isReadOnly()) {  // propose a new project in user's home dir
    path = WbPreferences::instance()->value("Directories/projects").toString();
    if (!path.isEmpty()) {
      if (WbFileUtil::isDirectoryWritable(path))
        path += "my_project";
      else
        path = "";  // no valid default path found
    }
  } else {  // otherwise propose new project dir as sibling of current project
    QDir dir(WbProject::current()->path());
    dir.cdUp();
    path = dir.absolutePath() + "/my_project";
  }
  // propose only if this directory does not yet exist or is empty
  if (!QFile::exists(path))
    return path;
  // test "my_project2", "my_project3", etc.
  QString pathi;
  int i = 2;
  do
    pathi = path + QString::number(i++);
  while (QFile::exists(pathi));
  return pathi;
}

void WbNewProjectWizard::accept() {
  if (!mProject->createNewProjectFolders()) {
    WbMessageBox::warning(tr("Some directories could not be created."), this, tr("Directories creation failed"));
    mWorldEdit->setText("");
    QDialog::accept();
    return;
  }
  WbProject::setCurrent(mProject);
  createWorldFile();
  // store the accepted project directory in the preferences
  QDir dir(mProject->path());
  dir.cdUp();  // store the upper level, probably the path where the directories are stored
  WbPreferences::instance()->setValue("Directories/projects", dir.absolutePath() + "/");
  QDialog::accept();
}

void WbNewProjectWizard::updateUI() {
  updateWorldUI();
  mProject->setPath(mDirEdit->text());
  mFilesLabel->setText(
    QDir::toNativeSeparators(mProject->newProjectFiles().join("\n").replace("empty.wbt", mWorldEdit->text())));
}

bool WbNewProjectWizard::validateCurrentPage() {
  if (currentId() == worldId() && !validateWorldPage())
    return false;
  updateUI();
  if (currentId() == directoryId()) {
    if (mDirEdit->text().isEmpty())
      return false;
    if (WbFileUtil::isLocatedInInstallationDirectory(mDirEdit->text())) {
      WbMessageBox::warning(tr("It is not allowed to create a new project inside the Webots installation directory.") + "\n" +
                              tr("Please select another directory."),
                            this, tr("Invalid new project directory"));
      return false;
    }
  }
  return true;
}

void WbNewProjectWizard::chooseDirectory() {
  QString dir = QFileDialog::getExistingDirectory(this, "Choose a directory", mDirEdit->text());
  if (dir.isEmpty())
    return;
  mDirEdit->setText(dir);
}

QWizardPage *WbNewProjectWizard::createDirectoryPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("Directory selection"));
  page->setSubTitle(tr("Please choose a directory for your new project:"));
  mDirEdit = new WbLineEdit(page);
  QPushButton *chooseButton = new QPushButton(tr("Choose"), page);
  connect(chooseButton, &QPushButton::pressed, this, &WbNewProjectWizard::chooseDirectory);
  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(mDirEdit);
  layout->addWidget(chooseButton);
  return page;
}
