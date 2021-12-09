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

#include "WbUpdatedDialog.hpp"

WbUpdatedDialog::WbUpdatedDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(QString("Welcome to Webots 2022a"));

  QPixmap webotsLogo("images:webots.png");
  QLabel *image = new QLabel(this);
  image->setGeometry(QRect(5, 5, 75, 75));
  image->setPixmap(webotsLogo);
  image->setScaledContents(true);

  labelWelcome = new QLabel(this);
  labelWelcome->setGeometry(QRect(85, 20, 340, 17));
  labelWelcome->setText(QString("<p style='font-size: large;'><b>Webots 2022a Introduction</b></p>"));

  labelThanks = new QLabel(this);
  labelThanks->setGeometry(QRect(85, 50, 340, 31));
  labelThanks->setText(QString("<b>Thank you for updating to the last version of Webots.</b>"));
  labelThanks->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  labelThanks->setWordWrap(true);

  groupBox = new QGroupBox(this);
  groupBox->setGeometry(QRect(35, 85, 385, 170));
  QFont font;
  font.setBold(true);
  groupBox->setFont(font);
  groupBox->setTitle(QString("Important Note"));
  groupBox->setStyleSheet("border: 1px solid gray;");

  labelNote = new QLabel(groupBox);
  labelNote->setGeometry(QRect(15, 20, 345, 140));
  labelNote->setText(QString(
    "Webots axis systems and objects orientation conventions respectively <b>changed to ENU and FLU</b>. All the protos and "
    "worlds included in the Webots package have been converted. <br /> <b>If you are using your own world or proto</b>, Webots will "
    "rotate all the elements of your world when you open it in this new version. However, you may have to rotate some "
    "objects, fix bounding objects by yourself or adjust your controller. Refer to the <a style='color: #5DADE2;' "
    "href='https://cyberbotics.com/doc/reference/changelog'>wiki</a> for more information."));
  labelNote->setOpenExternalLinks(true);
  labelNote->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  labelNote->setStyleSheet("border: none");
  labelNote->setWordWrap(true);

  labelChangelog = new QLabel(this);
  labelChangelog->setGeometry(QRect(35, 260, 290, 41));
  labelChangelog->setText(QString("Find out the new features, enhancements and bug fixes of Webots 2022a in the <a style='color: #5DADE2;' "
                                  "href='https://cyberbotics.com/doc/reference/changelog-r2022'>changelog</a>."));
  labelChangelog->setOpenExternalLinks(true);
  labelChangelog->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  labelChangelog->setWordWrap(true);

  QPushButton *closeButton = new QPushButton(tr("Close"), this);
  closeButton->setGeometry(QRect(340, 270, 80, 25));
  connect(closeButton, &QPushButton::pressed, this, &WbUpdatedDialog::close);
}
