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

#include "WbHtmlExportDialog.hpp"
#include "WbLight.hpp"
#include "WbMathsUtilities.hpp"
#include "WbPerspective.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbWorld.hpp"

#include <QtCore/QFileInfo>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

#include <cassert>
#include <cmath>

WbHtmlExportDialog::WbHtmlExportDialog(const QString &title, const QString &worldFilePath, QWidget *parent) :
  QDialog(parent),
  mTitle(title),
  mWorldFilePath(worldFilePath) {
  setWindowTitle(title);
  setModal(true);

  QGroupBox *fileGroup = new QGroupBox(this);
  mFileLineEdit = new QLineEdit(this);
  QString proposedFilename;
  for (int i = 0; i < 100; ++i) {
    QString suffix = i == 0 ? "" : QString("_%1").arg(i);
    proposedFilename =
      WbPreferences::instance()->value("Directories/www").toString() + QFileInfo(mWorldFilePath).baseName() + suffix + ".html";
    if (!QFileInfo::exists(proposedFilename))
      break;
  }
  mFileLineEdit->setText(WbProject::computeBestPathForSaveAs(proposedFilename));
  mFileLineEdit->setReadOnly(true);
  QPushButton *browseButton = new QPushButton("...", this);
  browseButton->setMaximumWidth(30);
  connect(browseButton, &QPushButton::clicked, this, &WbHtmlExportDialog::browse);

  QHBoxLayout *fileLayout = new QHBoxLayout(fileGroup);
  fileLayout->addWidget(new QLabel(tr("File name:"), this), 0, Qt::AlignLeft);
  fileLayout->addWidget(mFileLineEdit, 1);
  fileLayout->addWidget(browseButton, 0, Qt::AlignLeft);

  mButtonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
  connect(mButtonBox, &QDialogButtonBox::accepted, this, &WbHtmlExportDialog::accept);
  connect(mButtonBox, &QDialogButtonBox::rejected, this, &WbHtmlExportDialog::reject);

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(fileGroup);
  mainLayout->addWidget(mButtonBox);
}

WbHtmlExportDialog::~WbHtmlExportDialog() {
}

void WbHtmlExportDialog::accept() {
  QDialog::accept();
}

void WbHtmlExportDialog::browse() {
  const QString &fileName = QFileDialog::getSaveFileName(this, mTitle, mFileLineEdit->text(), tr("HTML Files (*.html *.HTML)"));
  if (!fileName.isEmpty())
    mFileLineEdit->setText(fileName);
}

QString WbHtmlExportDialog::fileName() {
  return mFileLineEdit->text();
}
