// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "WbNewProjectWizard.hpp"

#include "WbFileUtil.hpp"
#include "WbLineEdit.hpp"
#include "WbMessageBox.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizard>

enum { INTRO, DIRECTORY, WORLD, CONCLUSION };

QString WbNewProjectWizard::proposeNewProjectPath() const {
  QString path;

  // if current project is in Webots installation dir
  if (WbProject::current()->isReadOnly()) {
    // propose a new project in user's home dir
    path = WbPreferences::instance()->value("Directories/projects").toString() + "my_project";
  } else {
    // otherwisepropose new project dir as sibling of current project
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

WbNewProjectWizard::WbNewProjectWizard(QWidget *parent) : QWizard(parent) {
  addPage(createIntroPage());
  addPage(createDirectoryPage());
  addPage(createWorldPage());
  addPage(createConclusionPage());

  QString path = proposeNewProjectPath();
  mDirEdit->setText(QDir::toNativeSeparators(path));
  mWorldEdit->setText(WbProject::newWorldFileName());
  mBackgroundCheckBox->setChecked(true);
  mBackgroundCheckBox->setText("Add a textured background");
  mViewPointCheckBox->setChecked(true);
  mViewPointCheckBox->setText("Center view point");
  mDirectionalLightCheckBox->setChecked(true);
  mDirectionalLightCheckBox->setText("Add a directional light");
  mArenaCheckBox->setChecked(false);
  mArenaCheckBox->setText("Add a rectangle arena");

  mProject = new WbProject(path);
  mIsValidProject = false;

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Create a Webots project directory"));
}

WbNewProjectWizard::~WbNewProjectWizard() {
}

void WbNewProjectWizard::accept() {
  bool success = mProject->createNewProjectFiles(mWorldEdit->text());

  if (success) {
    QFile file(newWorldFile());
    file.open(QIODevice::ReadWrite);
    QByteArray worldContent = file.readAll();

    if (mBackgroundCheckBox->isChecked())
      worldContent.append(QByteArray("TexturedBackground {\n"
                                     "}\n"));
    else
      worldContent.append(QByteArray("Background {\n"
                                     "  skyColor [\n"
                                     "    0.4 0.7 1\n"
                                     "  ]\n"
                                     "}\n"));

    if (mViewPointCheckBox->isChecked())
      worldContent.replace(QByteArray("Viewpoint {"), QByteArray("Viewpoint {\n"
                                                                 "  orientation -0.5773 0.5773 0.5773 2.0944\n"
                                                                 "  position 0 0 10\n"));

    if (mDirectionalLightCheckBox->isChecked()) {
      if (mBackgroundCheckBox->isChecked())
        worldContent.append(QByteArray("TexturedBackgroundLight {\n"
                                       "}\n"));
      else
        worldContent.append(QByteArray("DirectionalLight {\n"
                                       "  ambientIntensity 1\n"
                                       "  direction 0.1 -0.5 0.3\n"
                                       "}\n"));
    }

    if (mArenaCheckBox->isChecked())
      worldContent.append(QByteArray("RectangleArena {\n"
                                     "}\n"));

    file.seek(0);
    file.write(worldContent);
    file.close();
  }

  mIsValidProject = true;
  if (success) {
    // store the accepted project directory in the preferences
    QDir dir(mProject->path());
    dir.cdUp();  // store the upper level, probably the path where the directories are stored
    WbPreferences::instance()->setValue("Directories/projects", dir.absolutePath() + "/");
  } else {
    WbMessageBox::warning(tr("Some directories or files could not be created."), this, tr("File creation failed"));
    mIsValidProject = false;
  }

  QDialog::accept();
}

void WbNewProjectWizard::updateUI() {
  mProject->setPath(mDirEdit->text());
  if (!mWorldEdit->text().isEmpty() && !mWorldEdit->text().endsWith(".wbt"))
    mWorldEdit->setText(mWorldEdit->text().append(".wbt"));
  mFilesLabel->setText(QDir::toNativeSeparators(
    mProject->newProjectFiles().join("\n").replace(WbProject::newWorldFileName(), mWorldEdit->text())));
}

bool WbNewProjectWizard::validateCurrentPage() {
  if (currentId() == WORLD && mWorldEdit->text().isEmpty()) {
    WbMessageBox::warning(tr("Please specify a world name."), this, tr("Invalid new world name"));
    return false;
  }

  updateUI();

  if (currentId() == DIRECTORY) {
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

bool WbNewProjectWizard::isValidProject() const {
  return mIsValidProject;
}

QString WbNewProjectWizard::newWorldFile() const {
  return mProject->worldsPath() + mWorldEdit->text();
}

QWizardPage *WbNewProjectWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("New project creation"));

  QLabel *label = new QLabel(tr("This wizard will help you creating a new project."), page);

  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);

  return page;
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

QWizardPage *WbNewProjectWizard::createWorldPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("World settings"));
  page->setSubTitle(tr("Please choose a name for the new world and select the features you want:"));

  mWorldEdit = new WbLineEdit(page);
  mBackgroundCheckBox = new QCheckBox(page);
  mViewPointCheckBox = new QCheckBox(page);
  mDirectionalLightCheckBox = new QCheckBox(page);
  mArenaCheckBox = new QCheckBox(page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mWorldEdit);
  layout->addWidget(mViewPointCheckBox);
  layout->addWidget(mBackgroundCheckBox);
  layout->addWidget(mDirectionalLightCheckBox);
  layout->addWidget(mArenaCheckBox);

  return page;
}

QWizardPage *WbNewProjectWizard::createConclusionPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Conclusion"));
  page->setSubTitle(tr("The following directories and files will be created:"));

  mFilesLabel = new QLabel(page);
  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(mFilesLabel);

  return page;
}
