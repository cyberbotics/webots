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

#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWizard>

enum { INTRO };

WbImportWizard::WbImportWizard(QWidget *parent) : QWizard(parent) {
  addPage(createIntroPage());

  setOption(QWizard::NoCancelButton, false);
  setOption(QWizard::CancelButtonOnLeft, true);
  setWindowTitle(tr("Import a 3D model in the scene."));
}

WbImportWizard::~WbImportWizard() {
}

bool WbImportWizard::validateCurrentPage() {
  if (currentId() == INTRO) {
    //TODO
    return true;
  }

  return true;
}

QWizardPage *WbImportWizard::createIntroPage() {
  QWizardPage *page = new QWizardPage(this);

  page->setTitle(tr("3D model importation"));

  QLabel *label = new QLabel(tr("This wizard will help you importing a 3D model in Webots.\n\nThe following file formats are supported:\n\t- Blender (*.blend)\n\t- Collada (*.dae)\n\t- STL (*.stl)\n\t- VRML (*.wrl)\n\t- Wavefront (*.obj)"), page);

  QVBoxLayout *layout = new QVBoxLayout(page);
  layout->addWidget(label);

  return page;
}
