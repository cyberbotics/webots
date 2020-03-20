// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "WbImportWizard.hpp"

#include "WbLineEdit.hpp"

#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizard>

enum { INTRO, FILE_SELECTION, CONCLUSION };

WbImportWizard::WbImportWizard(const QString &suggestedPath, QWidget *parent) : QWizard(parent) {
  addPage(createIntroPage());
  addPage(createFileSelectionPage());
  addPage(createConclusionPage());

  mFileEdit->setText(suggestedPath);

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Import a 3D model in the scene."));
}

WbImportWizard::~WbImportWizard() {
}

const QString WbImportWizard::fileName() {
  return mFileEdit->text();
}

bool WbImportWizard::validateCurrentPage() {
  if (currentId() == FILE_SELECTION) {
    // check that file exists
    if (!QFile::exists(fileName()))
      return false;
    // check file extension
    const QStringList supportedExtension = QStringList() << ".blend" << ".dae" << ".fbx" << ".obj" << ".wrl";
    for (int i = 0; i < supportedExtension.size(); ++i) {
      if (fileName().endsWith(supportedExtension[i], Qt::CaseInsensitive))
        return true;
    }
    return false;
  }
  return true;
}

QWizardPage *WbImportWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("3D model importation"));

  QLabel *label = new QLabel(tr("This wizard will help you importing a 3D model in Webots.\n\nThe following file formats are supported:\n\t- Blender (*.blend)\n\t- Collada (*.dae)\n\t- Filmbox (*.fbx)\n\t- STL (*.stl)\n\t- VRML (*.wrl)\n\t- Wavefront (*.obj)"), page);

  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);

  return page;
}

void WbImportWizard::chooseFile() {
  const QString fileName = QFileDialog::getOpenFileName(this, tr("Choose a File"), mFileEdit->text(),
                                                        tr("3D Files (*.blend *.dae *.fbx *.obj *.wrl *.WRL);;Blender (*.blend);;Collada (*.dae);;Filmbox (*.fbx);;STL (*.stl);;VRML (*.wrl *.WRL);;Wavefront (*.obj)"));
  if (fileName.isEmpty())
    return;
  mFileEdit->setText(fileName);
  mConclusionLabel->setText(tr("The '%1' file will now be imported at the end of the scene-tree.").arg(fileName));
}

QWizardPage *WbImportWizard::createFileSelectionPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("File Selection"));
  page->setSubTitle(tr("Please choose a 3D file to import:"));

  mFileEdit = new WbLineEdit(page);
  QPushButton *chooseButton = new QPushButton(tr("Choose"), page);

  connect(chooseButton, &QPushButton::pressed, this, &WbImportWizard::chooseFile);

  QHBoxLayout *layout = new QHBoxLayout(page);
  layout->addWidget(mFileEdit);
  layout->addWidget(chooseButton);

  return page;
}

QWizardPage *WbImportWizard::createConclusionPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Summary"));

  mConclusionLabel = new QLabel(page);

  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mConclusionLabel);

  return page;
}
