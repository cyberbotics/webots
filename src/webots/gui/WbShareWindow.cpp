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

#include "WbShareWindow.hpp"

#include "WbMainWindow.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"

#include <QtWidgets/QVBoxLayout>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtWidgets/QGroupBox>
#include <QtCore/qdebug.h>



WbShareWindow::WbShareWindow(QWidget *parent) : QWizard(parent) {

  

  setPage(SHARE, createSharePage());
  setPage(LINK, createLinkPage());
  setButtonText(QWizard::CustomButton1, tr("upload &scene"));
  setButtonText(QWizard::CustomButton2, tr("record and upload your &animation"));
  setOption(QWizard::NoDefaultButton, false);

  QList<QWizard::WizardButton> layout;
  layout << QWizard::CancelButton << QWizard::CustomButton1 << QWizard::CustomButton2;
  setButtonLayout(layout);

  connect(this, &QWizard::customButtonClicked, this, &WbShareWindow::ButtonClicked);
  connect(this, &QWizard::customButtonClicked, this, &QWizard::next);

  connect(this, &QWizard::customButtonClicked, this, &WbShareWindow::ButtonClicked);
  connect(this, &QWizard::customButtonClicked, this, &QWizard::next);

  setWindowTitle(tr("Share your simulation online"));
}

QWizardPage *WbShareWindow::createSharePage() {

  QString uploadUrl = WbPreferences::instance()->value("Network/uploadUrl").toString();
  if (uploadUrl.contains("//"))
    uploadUrl = uploadUrl.split("//")[1];

  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("Publish your simulation on %1").arg(uploadUrl));
  return page;
}

QWizardPage *WbShareWindow::createLinkPage() {
  QString groupBoxStyleSheet = "QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } QGroupBox::title "
                      "{subcontrol-origin:  margin; subcontrol-position: top center; }";

  QWizardPage *page = new QWizardPage(this);
  page->setTitle(tr("Upload successful"));


  QGroupBox *mGroupBoxLink = new QGroupBox(this);
  mGroupBoxLink->setTitle(tr(""));
  mGroupBoxLink->setStyleSheet(groupBoxStyleSheet);
  mGroupBoxLink->setGeometry(QRect(10, 20, 291, 61));

  mLabelLink = new QLabel(mGroupBoxLink);
  mLabelLink->setGeometry(QRect(10, 30, 271, 21));
  mLabelLink->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelLink->setStyleSheet("border: none;");
  mLabelLink->setOpenExternalLinks(true);
  mLabelLink->setWordWrap(true);

  mPushButtonSave = new QPushButton(this);
  mPushButtonSave->setGeometry(QRect(10, 90, 181, 25));
  mPushButtonSave->setFocusPolicy(Qt::NoFocus);
  mPushButtonSave->setText(tr("Also save a local copy..."));

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());

  connect(mPushButtonSave, &QPushButton::pressed, mainWindow, &WbMainWindow::exportHtmlFiles);
  connect(mPushButtonSave, &QPushButton::pressed, this, [this]() {
  mPushButtonSave->setEnabled(false);
  mPushButtonSave->setText(tr("local copy saved"));
  mPushButtonSave->setStyleSheet("color: gray;");
  });
  QVBoxLayout *layout = new QVBoxLayout(page);

  layout->addWidget(mGroupBoxLink);
  layout->addWidget(mPushButtonSave);

  return page;
}

void WbShareWindow::reject() {
  QDir dir(WbStandardPaths::webotsTmpPath() + "textures/");  // remove tmp files
  dir.removeRecursively();
  const QStringList extensions = {".html", ".x3d", ".json"};
  foreach (QString extension, extensions)
    QFile::remove(WbStandardPaths::webotsTmpPath() + "export_cloud" + extension);

  QDialog::reject();
}

void WbShareWindow::ButtonClicked(int which){
  WbMainWindow *mMainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
  if(which==6)
    mMainWindow->uploadScene();
  else if(which==7)
    mMainWindow->startAnimationRecording();
}