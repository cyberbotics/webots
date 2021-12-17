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

#include <QtWidgets/QGridLayout>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QSpacerItem>

QString groupBoxStyleSheet;

WbShareWindow::WbShareWindow(QWidget *parent) : QDialog(parent) {
  QString uploadUrl = WbPreferences::instance()->value("Network/uploadUrl").toString();
  if (uploadUrl.contains("//"))
    uploadUrl = uploadUrl.split("//")[1];
  groupBoxStyleSheet = "QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } QGroupBox::title "
                       "{subcontrol-origin:  margin; subcontrol-position: top center; }";
  this->setWindowTitle(tr("Share your simulation online"));

  QGridLayout *layout = new QGridLayout(this);

  QLabel *mLabelIntro = new QLabel(this);
  mLabelIntro->setWordWrap(true);
  mLabelIntro->setOpenExternalLinks(true);
  mLabelIntro->setText(
    tr("<html><head/><body><p>Publish your simulation on <a href=\"https://%1/\"><span "
       "style=\" text-decoration: underline; color:#5dade2;\">%1</span></a>.</p></body></html>")
      .arg(uploadUrl));
  layout->addWidget(mLabelIntro, 3, 0, 1, 2);

  QSpacerItem *verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
  layout->addItem(verticalSpacer, 4, 0, 1, 2);

  QPushButton *mPushButtonScene = new QPushButton(this);
  //mPushButtonScene->setGeometry(QRect(12, 72, 150, 42));
  mPushButtonScene->setFocusPolicy(Qt::NoFocus);
  mPushButtonScene->setText(tr("Upload your scene"));
  layout->addWidget(mPushButtonScene, 5, 0, 1, 1);

  QPushButton *mPushButtonAnimation = new QPushButton(this);
  //mPushButtonAnimation->setGeometry(QRect(12, 72, 150, 42));
  mPushButtonAnimation->setFocusPolicy(Qt::NoFocus);
  mPushButtonAnimation->setText(tr("Record and\n"
                                   "upload &animation"));
  layout->addWidget(mPushButtonAnimation, 5, 1, 1, 1);

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());

  connect(mPushButtonScene, &QPushButton::pressed, mainWindow, &WbMainWindow::uploadScene);
  connect(mPushButtonScene, &QPushButton::pressed, this, &WbShareWindow::close);

  connect(mPushButtonAnimation, &QPushButton::pressed, mainWindow, &WbMainWindow::startAnimationRecording);
  connect(mPushButtonAnimation, &QPushButton::pressed, this, &WbShareWindow::close);
}

WbLinkWindow::WbLinkWindow(QWidget *parent) : QDialog(parent) {
  this->setWindowTitle(tr("Share your simulation"));

  QGroupBox *mGroupBoxLink = new QGroupBox(this);
  mGroupBoxLink->setTitle(tr("Upload successful"));
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
}

void WbLinkWindow::reject() {
  QDir dir(WbStandardPaths::webotsTmpPath() + "textures/");  // remove tmp files
  dir.removeRecursively();
  const QStringList extensions = {".html", ".x3d", ".json"};
  foreach (QString extension, extensions)
    QFile::remove(WbStandardPaths::webotsTmpPath() + "export_cloud" + extension);

  QDialog::reject();
}
