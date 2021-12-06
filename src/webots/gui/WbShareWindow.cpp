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
  this->setWindowTitle("Share your project on Webots.cloud");

  groupBoxScene = new QGroupBox(this);
  groupBoxScene->setGeometry(QRect(10, 80, 169, 126));
  groupBoxScene->setStyleSheet("border: 1px solid gray;");
  groupBoxScene->setTitle("Upload your scene");

  groupBoxAnimation = new QGroupBox(this);
  groupBoxAnimation->setGeometry(QRect(210, 80, 169, 126));
  groupBoxAnimation->setStyleSheet("border: 1px solid gray;");
  groupBoxAnimation->setTitle("Share your animation");

  pushButtonAnimation = new QPushButton(groupBoxAnimation);
  pushButtonAnimation->setGeometry(QRect(12, 72, 150, 42));
  pushButtonAnimation->setStyleSheet("border: 0px;");
  pushButtonAnimation->setText("Start your animation \n"
                               " and get a link");

  pushButtonScene = new QPushButton(groupBoxScene);
  pushButtonScene->setGeometry(QRect(12, 72, 150, 42));
  pushButtonScene->setStyleSheet("border: 0px;");
  pushButtonScene->setText("Get a link");

  labelIntro = new QLabel(this);
  labelIntro->setGeometry(QRect(9, 9, 380, 68));
  labelIntro->setWordWrap(true);
  labelIntro->setOpenExternalLinks(true);
  labelIntro->setText(
    "<html><head/><body><p>You can now upload your scenes and animations on <a href=\"https://beta.webots.cloud/\"><span "
    "style=\" text-decoration: underline; color:#5dade2;\">beta.webots.cloud</span></a>.\nClick on one of the buttons to "
    "generate a sharing link that you can send to others.</p></body></html>");

  labelScene = new QLabel(groupBoxScene);
  labelScene->setGeometry(QRect(12, 32, 155, 34));
  labelScene->setStyleSheet("border: 0px;");
  labelScene->setWordWrap(true);
  labelScene->setText("upload your scene on Webots.cloud.");

  labelAnimation = new QLabel(groupBoxAnimation);
  labelAnimation->setGeometry(QRect(12, 32, 155, 34));
  labelAnimation->setStyleSheet("border: 0px;");
  labelAnimation->setWordWrap(true);
  labelAnimation->setText("Start recording your animation and upload it.");

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());

  connect(pushButtonScene, &QPushButton::pressed, mainWindow, &WbMainWindow::uploadScene);
  connect(pushButtonScene, SIGNAL(pressed()), this, SLOT(close()));

  connect(pushButtonAnimation, &QPushButton::pressed, mainWindow, &WbMainWindow::startAnimationRecording);
  connect(pushButtonAnimation, SIGNAL(pressed()), this, SLOT(close()));
}
