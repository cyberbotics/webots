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

#include "WbImportWizard.hpp"

#include "WbLineEdit.hpp"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizard>

enum { INTRO, FILE_SELECTION, SETTINGS, CONCLUSION };

WbImportWizard::WbImportWizard(const QString &suggestedPath, QWidget *parent) : QWizard(parent) {
  addPage(createIntroPage());
  addPage(createFileSelectionPage());
  addPage(createOptionPage());
  addPage(createConclusionPage());

  mFileEdit->setText(suggestedPath);

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Import a 3D model in the scene."));
}

WbImportWizard::~WbImportWizard() {
}

const QString WbImportWizard::fileName() const {
  return mFileEdit->text();
}

bool WbImportWizard::importTextureCoordinates() const {
  return mTextureCoordinateCheckBox->isChecked();
}

bool WbImportWizard::importNormals() const {
  return mNormalCheckBox->isChecked();
}

bool WbImportWizard::importAppearances() const {
  return mAppearancesCheckBox->isChecked();
}

bool WbImportWizard::importAsSolid() const {
  return mSolidCheckBox->isChecked();
}

bool WbImportWizard::importBoundingObjects() const {
  return mBoundingObjectCheckBox->isChecked();
}

bool WbImportWizard::validateCurrentPage() {
  if (currentId() == FILE_SELECTION) {
    // check that file exists
    if (!QFile::exists(fileName()))
      return false;
    // check file extension
    const QStringList supportedExtension = QStringList() << ".dae"
                                                         << ".stl"
                                                         << ".obj";
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

  QLabel *label =
    new QLabel(tr("This wizard will help you importing a 3D model in Webots.\n\nThe following file formats are supported:"
                  "\n\t- Collada (*.dae)"
                  "\n\t- STL (*.stl)"
                  "\n\t- Wavefront (*.obj)"),
               page);

  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);

  return page;
}

void WbImportWizard::chooseFile() {
  const QString fileName = QFileDialog::getOpenFileName(this, tr("Choose a File"), mFileEdit->text(),
                                                        tr("3D Files (*.dae *.DAE *.stl *.STL *.obj *.OBJ);;"
                                                           "Collada (*.dae *.DAE);;"
                                                           "STL (*.stl *.STL);;"
                                                           "Wavefront (*.obj *.OBJ)"));
  if (fileName.isEmpty())
    return;
  mFileEdit->setText(fileName);
  mConclusionLabel->setText(tr("The '%1' file will now be imported at the end of the scene tree.").arg(fileName));
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

QWizardPage *WbImportWizard::createOptionPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("Importation Settings"));
  page->setSubTitle(tr("Please choose how do you want to import the model:"));

  mTextureCoordinateCheckBox = new QCheckBox(tr("Import texture coordinates (if available)."), page);
  mTextureCoordinateCheckBox->setChecked(true);
  mNormalCheckBox = new QCheckBox(tr("Import normals (if available)."), page);
  mNormalCheckBox->setChecked(true);
  mAppearancesCheckBox = new QCheckBox(tr("Import appearances (if available)."), page);
  mAppearancesCheckBox->setChecked(true);
  mSolidCheckBox = new QCheckBox(tr("Import nodes as Solids."), page);
  mSolidCheckBox->setChecked(true);
  mBoundingObjectCheckBox = new QCheckBox(tr("Use meshes for bounding objects."), page);
  mBoundingObjectCheckBox->setChecked(false);
  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(mTextureCoordinateCheckBox);
  layout->addWidget(mNormalCheckBox);
  layout->addWidget(mAppearancesCheckBox);
  layout->addWidget(mSolidCheckBox);
  layout->addWidget(mBoundingObjectCheckBox);

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
