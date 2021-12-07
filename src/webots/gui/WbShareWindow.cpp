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

  groupBoxScene = new QGroupBox(this);
  groupBoxScene->setGeometry(QRect(10, 80, 169, 126));
  groupBoxScene->setStyleSheet("border: 1px solid gray;");
  groupBoxScene->setTitle(tr("Upload your scene"));

  groupBoxAnimation = new QGroupBox(this);
  groupBoxAnimation->setGeometry(QRect(210, 80, 169, 126));
  groupBoxAnimation->setStyleSheet("border: 1px solid gray;");
  groupBoxAnimation->setTitle(tr("Share your animation"));

  pushButtonAnimation = new QPushButton(groupBoxAnimation);
  pushButtonAnimation->setGeometry(QRect(12, 72, 150, 42));
  pushButtonAnimation->setStyleSheet("border: 0px;");
  pushButtonAnimation->setText(tr("Start your animation \n"
                               " and get a link"));

  pushButtonScene = new QPushButton(groupBoxScene);
  pushButtonScene->setGeometry(QRect(12, 72, 150, 42));
  pushButtonScene->setStyleSheet("border: 0px;");
  pushButtonScene->setText(tr("Get a link"));

  labelIntro = new QLabel(this);
  labelIntro->setGeometry(QRect(9, 9, 380, 68));
  labelIntro->setWordWrap(true);
  labelIntro->setOpenExternalLinks(true);
  labelIntro->setText(tr(
    "<html><head/><body><p>You can now upload your scenes and animations on <a href=\"https://beta.webots.cloud/\"><span "
    "style=\" text-decoration: underline; color:#5dade2;\">beta.webots.cloud</span></a>.\nClick on one of the buttons to "
    "generate a sharing link that you can send to others.</p></body></html>"));

  labelScene = new QLabel(groupBoxScene);
  labelScene->setGeometry(QRect(12, 32, 155, 34));
  labelScene->setStyleSheet("border: none;");
  labelScene->setWordWrap(true);
  labelScene->setText(tr("upload your scene on Webots.cloud."));

  labelAnimation = new QLabel(groupBoxAnimation);
  labelAnimation->setGeometry(QRect(12, 32, 155, 34));
  labelAnimation->setStyleSheet("border: none;");
  labelAnimation->setWordWrap(true);
  labelAnimation->setText(tr("Start recording your animation and upload it."));

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());

  connect(pushButtonScene, &QPushButton::pressed, mainWindow, &WbMainWindow::uploadScene);
  connect(pushButtonScene, &QPushButton::pressed, this, &WbShareWindow::close);

  connect(pushButtonAnimation, &QPushButton::pressed, mainWindow, &WbMainWindow::startAnimationRecording);
  connect(pushButtonAnimation, &QPushButton::pressed, this, &WbShareWindow::close);
}

WbLinkWindow::WbLinkWindow(QWidget *parent) : QDialog(parent) {
  this->setWindowTitle(tr("Share your project on Webots.cloud"));

  groupBoxLink = new QGroupBox(this);
  groupBoxLink->setTitle(tr("Upload successful"));
  groupBoxLink->setStyleSheet("border: 1px solid gray;");
  groupBoxLink->setGeometry(QRect(10, 20, 291, 61));

  labelLink = new QLabel(groupBoxLink);
  labelLink->setGeometry(QRect(10, 30, 271, 21));
  labelLink->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
  labelLink->setStyleSheet("border: none;");
  labelLink->setOpenExternalLinks(true);
  labelLink->setWordWrap(true);


  pushButtonSave = new QPushButton(this);
  pushButtonSave->setGeometry(QRect(10, 90, 181, 25));
  pushButtonSave->setStyleSheet(QString::fromUtf8("border: 1px solid gray;;\n""color: #5DADE2;\n""background: transparent;"));
  pushButtonSave->setCursor(QCursor(Qt::PointingHandCursor));
  pushButtonSave->setText(tr("Also save a local copy..."));

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
  connect(pushButtonSave, &QPushButton::pressed, mainWindow, &WbMainWindow::exportHtmlFiles);
  connect(pushButtonSave, &QPushButton::pressed, this, &WbLinkWindow::close);

}