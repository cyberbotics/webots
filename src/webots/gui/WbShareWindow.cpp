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

WbShareWindow::WbShareWindow(QWidget *parent) : QDialog(parent) {
  this->setWindowTitle(tr("Share your project on Webots.cloud"));

  mGroupBoxScene = new QGroupBox(this);
  mGroupBoxScene->setGeometry(QRect(10, 80, 169, 126));
  mGroupBoxScene->setStyleSheet("border: 1px solid gray;");
  mGroupBoxScene->setTitle(tr("Upload your scene"));

  mGroupBoxAnimation = new QGroupBox(this);
  mGroupBoxAnimation->setGeometry(QRect(210, 80, 169, 126));
  mGroupBoxAnimation->setStyleSheet("border: 1px solid gray;");
  mGroupBoxAnimation->setTitle(tr("Share your animation"));

  mPushButtonAnimation = new QPushButton(mGroupBoxAnimation);
  mPushButtonAnimation->setGeometry(QRect(12, 72, 150, 42));
  mPushButtonAnimation->setStyleSheet("border: 0px;");
  mPushButtonAnimation->setText(tr("Start your animation \n"
                                   " and get a link"));

  mPushButtonScene = new QPushButton(mGroupBoxScene);
  mPushButtonScene->setGeometry(QRect(12, 72, 150, 42));
  mPushButtonScene->setStyleSheet("border: 0px;");
  mPushButtonScene->setText(tr("Get a link"));

  mLabelIntro = new QLabel(this);
  mLabelIntro->setGeometry(QRect(9, 9, 380, 68));
  mLabelIntro->setWordWrap(true);
  mLabelIntro->setOpenExternalLinks(true);
  mLabelIntro->setText(
    tr("<html><head/><body><p>You can now upload your scenes and animations on <a href=\"https://beta.webots.cloud/\"><span "
       "style=\" text-decoration: underline; color:#5dade2;\">beta.webots.cloud</span></a>.\nClick on one of the buttons to "
       "generate a sharing link that you can send to others.</p></body></html>"));

  mLabelScene = new QLabel(mGroupBoxScene);
  mLabelScene->setGeometry(QRect(12, 32, 155, 34));
  mLabelScene->setStyleSheet("border: none;");
  mLabelScene->setWordWrap(true);
  mLabelScene->setText(tr("upload your scene on Webots.cloud."));

  mLabelAnimation = new QLabel(mGroupBoxAnimation);
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
  this->setWindowTitle(tr("Share your project on Webots.cloud"));

  mGroupBoxLink = new QGroupBox(this);
  mGroupBoxLink->setTitle(tr("Upload successful"));
  mGroupBoxLink->setStyleSheet("border: 1px solid gray;");
  mGroupBoxLink->setGeometry(QRect(10, 20, 291, 61));

  mLabelLink = new QLabel(mGroupBoxLink);
  mLabelLink->setGeometry(QRect(10, 30, 271, 21));
  mLabelLink->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelLink->setStyleSheet("border: none;");
  mLabelLink->setOpenExternalLinks(true);
  mLabelLink->setWordWrap(true);

  mPushButtonSave = new QPushButton(this);
  mPushButtonSave->setGeometry(QRect(10, 90, 181, 25));
  mPushButtonSave->setStyleSheet("border: 1px solid gray;;\n"
                                 "color: #5DADE2;\n"
                                 "background: transparent;");
  mPushButtonSave->setCursor(QCursor(Qt::PointingHandCursor));
  mPushButtonSave->setText(tr("Also save a local copy..."));

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
  connect(mPushButtonSave, &QPushButton::pressed, mainWindow, &WbMainWindow::exportHtmlFiles);
  connect(mPushButtonSave, &QPushButton::pressed, this, [this]() {
    mPushButtonSave->setEnabled(false);
    mPushButtonSave->setText(tr("local copy saved"));
    mPushButtonSave->setStyleSheet("border: 1px solid gray;;\n"
                                   "color: gray;\n"
                                   "background: transparent;");
  });
}

void WbLinkWindow::reject() {
  QDir dir(WbStandardPaths::webotsTmpPath() + "textures/");  // remove tmp files
  dir.removeRecursively();
  QStringList extensions = {".html", ".x3d", ".json"};
  foreach (QString extension, extensions)
    QFile::remove(WbStandardPaths::webotsTmpPath() + "export_cloud" + extension);

  QDialog::reject();
}
