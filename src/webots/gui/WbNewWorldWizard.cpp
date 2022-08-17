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

#include "WbNewWorldWizard.hpp"

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

enum { INTRO, WORLD, CONCLUSION };

WbNewWorldWizard::WbNewWorldWizard(QWidget *parent) : QWizard(parent) {
  addPage(createIntroPage());
  addPage(createFilePage());
  addPage(createWorldPage());
  addPage(createConclusionPage());

  mWorldFileNameEdit->setText("");
  mBackgroundCheckBox->setChecked(true);
  mBackgroundCheckBox->setText(tr("Add a textured background"));
  mViewPointCheckBox->setChecked(true);
  mViewPointCheckBox->setText("Center view point");
  mDirectionalLightCheckBox->setChecked(true);
  mDirectionalLightCheckBox->setText("Add a directional light");
  mArenaCheckBox->setChecked(false);
  mArenaCheckBox->setText("Add a rectangle arena");

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Create a Webots project directory"));
}

WbNewWorldWizard::~WbNewWorldWizard() {
}

void WbNewWorldWizard::accept() {
  QFile file(WbProject::current()->worldsPath() + fileName());
  file.open(QIODevice::WriteOnly);
  QByteArray worldContent;
  worldContent.append(QString("#VRML_SIM %1 utf8\n").arg(WbApplicationInfo::version().toString(false)).toUtf8());

  QStringList externProtoList;
  if (mBackgroundCheckBox->isChecked()) {
    externProtoList << "TexturedBackground";
    if (mDirectionalLightCheckBox->isChecked())
      externProtoList << "TexturedBackgroundLight";
  }
  if (mArenaCheckBox->isChecked())
    externProtoList << "RectangleArena";
  foreach (const QString &protoModel, externProtoList) {
    const QString &modelPath = WbProtoManager::instance()->protoUrl(protoModel, WbProtoManager::PROTO_WEBOTS);
    worldContent.append(QByteArray(QString("EXTERNPROTO \"%1\"\n").arg(modelPath).toUtf8()));
  }

  worldContent.append(QByteArray("WorldInfo {\n}\n"));
  worldContent.append(QByteArray("Viewpoint {\n"));
  if (mViewPointCheckBox->isChecked())
    worldContent.append(QByteArray("  orientation -0.5773 0.5773 0.5773 2.0944\n"
                                   "  position 0 0 10\n"));
  worldContent.append(QByteArray("}\n"));

  if (mBackgroundCheckBox->isChecked())
    worldContent.append(QByteArray("TexturedBackground {\n}\n"));
  else
    worldContent.append(QByteArray("Background {\n"
                                   "  skyColor [\n"
                                   "    0.4 0.7 1\n"
                                   "  ]\n"
                                   "}\n"));

  if (mDirectionalLightCheckBox->isChecked()) {
    if (mBackgroundCheckBox->isChecked())
      worldContent.append(QByteArray("TexturedBackgroundLight {\n}\n"));
    else
      worldContent.append(QByteArray("DirectionalLight {\n"
                                     "  ambientIntensity 1\n"
                                     "  direction 0.1 -0.5 0.3\n"
                                     "}\n"));
  }

  if (mArenaCheckBox->isChecked())
    worldContent.append(QByteArray("RectangleArena {\n}\n"));

  file.seek(0);
  file.write(worldContent);
  file.close();

  QDialog::accept();
}

void WbNewWorldWizard::updateUI() {
  if (!mWorldFileNameEdit->text().isEmpty() && !mWorldFileNameEdit->text().endsWith(".wbt", Qt::CaseInsensitive))
    mWorldFileNameEdit->setText(mWorldFileNameEdit->text().append(".wbt"));
  mFileLabel->setText(QDir::toNativeSeparators(WbProject::current()->worldsPath() + mWorldFileNameEdit->text()));
}

bool WbNewWorldWizard::validateCurrentPage() {
  if (currentId() == WORLD && mWorldFileNameEdit->text().isEmpty()) {
    WbMessageBox::warning(tr("Please specify a world name."), this, tr("Invalid new world name"));
    return false;
  }
  updateUI();
  return true;
}

QString WbNewWorldWizard::fileName() const {
  return mWorldFileNameEdit->text();
}

QWizardPage *WbNewWorldWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("New world creation"));
  QLabel *label = new QLabel(tr("This wizard will help you creating a new world file."), page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);
  return page;
}

void WbNewWorldWizard::chooseFileName() {
  const QString path = WbProject::current()->worldsPath() + "my_world_file.wbt";
  const QString fileName =
    QFileDialog::getSaveFileName(this, tr("Select World File Name"), path, tr("Webots World File (*.wbt)"));
  if (fileName.isEmpty()) {
    mWorldFileNameEdit->setText("");
    return;
  }
  QFileInfo fileInfo(fileName);
  if (fileInfo.dir().absolutePath() + '/' != WbProject::current()->worldsPath()) {
    WbMessageBox::warning(tr("The world file should be saved in the \"worlds\" directory of the current project."), this,
                          tr("Invalid world path"));
    mWorldFileNameEdit->setText("");
    return;
  }
  mWorldFileNameEdit->setText(fileInfo.fileName());
}

QWizardPage *WbNewWorldWizard::createFilePage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("World file name"));
  page->setSubTitle(tr("Please choose a file name for your new world file:"));
  mWorldFileNameEdit = new WbLineEdit(page);
  mWorldFileNameEdit->setReadOnly(true);
  QPushButton *chooseButton = new QPushButton(tr("Choose"), page);
  connect(chooseButton, &QPushButton::pressed, this, &WbNewWorldWizard::chooseFileName);
  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(mWorldFileNameEdit);
  layout->addWidget(chooseButton);
  return page;
}

QWizardPage *WbNewWorldWizard::createWorldPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("World settings"));
  page->setSubTitle(tr("Please select the features you want:"));
  mBackgroundCheckBox = new QCheckBox(page);
  mViewPointCheckBox = new QCheckBox(page);
  mDirectionalLightCheckBox = new QCheckBox(page);
  mArenaCheckBox = new QCheckBox(page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mViewPointCheckBox);
  layout->addWidget(mBackgroundCheckBox);
  layout->addWidget(mDirectionalLightCheckBox);
  layout->addWidget(mArenaCheckBox);
  return page;
}

QWizardPage *WbNewWorldWizard::createConclusionPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("Conclusion"));
  page->setSubTitle(tr("The following file will be created:"));
  mFileLabel = new QLabel(page);
  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(mFileLabel);
  return page;
}
