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
QString groupBoxStyleSheet;

WbShareWindow::WbShareWindow(QWidget *parent) : QDialog(parent) {
  QString uploadUrl = WbPreferences::instance()->value("Network/uploadUrl").toString();
  if (uploadUrl.contains("//"))
    uploadUrl = uploadUrl.split("//")[1];
  groupBoxStyleSheet = "QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } QGroupBox::title "
                       "{subcontrol-origin:  margin; subcontrol-position: top center; }";
  this->setWindowTitle(tr("Share your simulation on %1").arg(uploadUrl));

  QGroupBox *mGroupBoxScene = new QGroupBox(this);
  mGroupBoxScene->setGeometry(QRect(10, 80, 169, 126));
  mGroupBoxScene->setStyleSheet(groupBoxStyleSheet);
  mGroupBoxScene->setTitle(tr("Upload your scene"));

  QGroupBox *mGroupBoxAnimation = new QGroupBox(this);
  mGroupBoxAnimation->setGeometry(QRect(210, 80, 169, 126));
  mGroupBoxAnimation->setStyleSheet(groupBoxStyleSheet);
  mGroupBoxAnimation->setTitle(tr("Share your animation"));

  QPushButton *mPushButtonScene = new QPushButton(mGroupBoxScene);
  mPushButtonScene->setGeometry(QRect(12, 72, 150, 42));
  mPushButtonScene->setFocusPolicy(Qt::NoFocus);
  mPushButtonScene->setText(tr("Get a link"));

  QPushButton *mPushButtonAnimation = new QPushButton(mGroupBoxAnimation);
  mPushButtonAnimation->setGeometry(QRect(12, 72, 150, 42));
  mPushButtonAnimation->setFocusPolicy(Qt::NoFocus);
  mPushButtonAnimation->setText(tr("Start your animation \n"
                                   " and get a link"));

  QLabel *mLabelIntro = new QLabel(this);
  mLabelIntro->setGeometry(QRect(9, 9, 380, 68));
  mLabelIntro->setWordWrap(true);
  mLabelIntro->setOpenExternalLinks(true);
  mLabelIntro->setText(
    tr("<html><head/><body><p>You can now upload your scenes and animations on <a href=\"https://%1/\"><span "
       "style=\" text-decoration: underline; color:#5dade2;\">%1</span></a>.\nClick on one of the buttons to "
       "generate a sharing link that you can send to others.</p></body></html>")
      .arg(uploadUrl));

  QLabel *mLabelScene = new QLabel(mGroupBoxScene);
  mLabelScene->setGeometry(QRect(12, 32, 155, 34));
  mLabelScene->setStyleSheet("border: none;");
  mLabelScene->setWordWrap(true);
  mLabelScene->setText(tr("upload your scene on %1.").arg(uploadUrl));

  QLabel *mLabelAnimation = new QLabel(mGroupBoxAnimation);
  mLabelAnimation->setGeometry(QRect(12, 32, 155, 34));
  mLabelAnimation->setStyleSheet("border: none;");
  mLabelAnimation->setWordWrap(true);
  mLabelAnimation->setText(tr("Start recording your animation and upload it."));

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
