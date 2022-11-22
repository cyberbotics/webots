// Copyright 1996-2023 Cyberbotics Ltd.
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

#ifndef WB_IMPORT_WIZARD_HPP
#define WB_IMPORT_WIZARD_HPP

//
// Description: Wizard for importing a 3D model in Webots
//

#include <QtWidgets/QWizard>

class QCheckBox;
class QLabel;
class WbLineEdit;

class WbImportWizard : public QWizard {
  Q_OBJECT

public:
  explicit WbImportWizard(const QString &suggestedPath, QWidget *parent = NULL);
  virtual ~WbImportWizard();

  const QString fileName() const;
  bool importTextureCoordinates() const;
  bool importNormals() const;
  bool importAppearances() const;
  bool importAsSolid() const;
  bool importBoundingObjects() const;
  bool validateCurrentPage() override;

private slots:
  void chooseFile();

private:
  WbLineEdit *mFileEdit;
  QLabel *mConclusionLabel;
  QCheckBox *mTextureCoordinateCheckBox;
  QCheckBox *mNormalCheckBox;
  QCheckBox *mAppearancesCheckBox;
  QCheckBox *mSolidCheckBox;
  QCheckBox *mBoundingObjectCheckBox;

  QWizardPage *createIntroPage();
  QWizardPage *createFileSelectionPage();
  QWizardPage *createOptionPage();
  QWizardPage *createConclusionPage();
};

#endif
