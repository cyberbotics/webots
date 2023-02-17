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

WbNewWorldWizard::WbNewWorldWizard(QWidget *parent) : QWizard(parent) {
  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
}

WbNewWorldWizard::~WbNewWorldWizard() {
}

int WbNewWorldWizard::exec() {
  setWindowTitle(title());
  setPage(introId(), createIntroPage());
  setPage(worldId(), createWorldPage());
  setPage(conclusionId(), createConclusionPage());
  return QWizard::exec();
}

void WbNewWorldWizard::createWorldFile() {
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
}

void WbNewWorldWizard::accept() {
  createWorldFile();
  QDialog::accept();
}

void WbNewWorldWizard::updateWorldUI() {
  if (!mWorldEdit->text().isEmpty() && !mWorldEdit->text().endsWith(".wbt", Qt::CaseInsensitive))
    mWorldEdit->setText(mWorldEdit->text().append(".wbt"));
}

void WbNewWorldWizard::updateUI() {
  updateWorldUI();
  mFilesLabel->setText(QDir::toNativeSeparators(WbProject::current()->worldsPath() + mWorldEdit->text()));
}

bool WbNewWorldWizard::validateWorldPage() {
  assert(currentId() == worldId());
  if (mWorldEdit->text().isEmpty()) {
    WbMessageBox::warning(tr("Please specify a world name."), this, tr("Invalid new world name"));
    return false;
  }
  if (QFileInfo(mWorldEdit->text()).dir() != QDir()) {
    // mWorldEdit->text() should not contain any directory
    WbMessageBox::warning(tr("Please specify a world name and not a path."), this, tr("Invalid new world name"));
    return false;
  }
  return true;
}

bool WbNewWorldWizard::validateCurrentPage() {
  if (currentId() == worldId()) {
    if (!validateWorldPage())
      return false;
    QString path = WbProject::current()->worldsPath() + mWorldEdit->text();
    if (!path.endsWith(".wbt"))
      path += ".wbt";
    if (QFile::exists(path)) {
      WbMessageBox::warning(tr("A world file with this name already exists, please choose a different name."), this,
                            tr("Invalid new world name"));
      return false;
    }
  }
  updateUI();
  return true;
}

QString WbNewWorldWizard::fileName() const {
  return mWorldEdit->text();
}

QWizardPage *WbNewWorldWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(introTitle());
  QLabel *label = new QLabel(introText(), page);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);
  return page;
}

QWizardPage *WbNewWorldWizard::createWorldPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("World settings"));
  page->setSubTitle(tr("Please choose a name for the new world and select the features you want:"));
  mWorldEdit = new WbLineEdit(page);
  mBackgroundCheckBox = new QCheckBox(page);
  mBackgroundCheckBox->setChecked(true);
  mBackgroundCheckBox->setText(tr("Add a textured background"));
  mViewPointCheckBox = new QCheckBox(page);
  mViewPointCheckBox->setChecked(true);
  mViewPointCheckBox->setText(tr("Center view point"));
  mDirectionalLightCheckBox = new QCheckBox(page);
  mDirectionalLightCheckBox->setChecked(true);
  mDirectionalLightCheckBox->setText(tr("Add a directional light"));
  mArenaCheckBox = new QCheckBox(page);
  mArenaCheckBox->setChecked(false);
  mArenaCheckBox->setText(tr("Add a rectangle arena"));
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mWorldEdit);
  layout->addWidget(mViewPointCheckBox);
  layout->addWidget(mBackgroundCheckBox);
  layout->addWidget(mDirectionalLightCheckBox);
  layout->addWidget(mArenaCheckBox);
  return page;
}

QWizardPage *WbNewWorldWizard::createConclusionPage() {
  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("Conclusion"));
  page->setSubTitle(conclusionText());
  mFilesLabel = new QLabel(page);
  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(mFilesLabel);
  return page;
}
