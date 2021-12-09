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
  QLabel *mImage = new QLabel(this);
  mImage->setGeometry(QRect(5, 5, 75, 75));
  mImage->setPixmap(webotsLogo);
  mImage->setScaledContents(true);

  mLabelWelcome = new QLabel(this);
  mLabelWelcome->setGeometry(QRect(85, 20, 340, 17));
  mLabelWelcome->setText(QString("<p style='font-size: large;'><b>Webots 2022a Introduction</b></p>"));

  mLabelThanks = new QLabel(this);
  mLabelThanks->setGeometry(QRect(85, 50, 340, 31));
  mLabelThanks->setText(QString("<b>Thank you for updating to the last version of Webots.</b>"));
  mLabelThanks->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelThanks->setWordWrap(true);

  mGroupBox = new QGroupBox(this);
  mGroupBox->setGeometry(QRect(35, 85, 385, 170));
  QFont font;
  font.setBold(true);
  mGroupBox->setFont(font);
  mGroupBox->setTitle(QString("Important Note"));
  mGroupBox->setStyleSheet("border: 1px solid gray;");

  mLabelNote = new QLabel(mGroupBox);
  mLabelNote->setGeometry(QRect(15, 20, 345, 140));
  mLabelNote->setText(QString(
    "Webots axis systems and objects orientation conventions respectively <b>changed to ENU and FLU</b>. All the PROTOs and "
    "worlds included in the Webots package are now in ENU and FLU. <br /> <b>If you are using your own world or PROTO</b>, "
    "Webots will "
    "rotate all the elements of your world when you open it in this new version. However, you may have to rotate some "
    "objects, fix bounding objects by yourself or adjust your controller. Refer to the <a style='color: #5DADE2;' "
    "href='https://cyberbotics.com/doc/reference/changelog'>wiki</a> for more information."));
  mLabelNote->setOpenExternalLinks(true);
  mLabelNote->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelNote->setStyleSheet("border: none");
  mLabelNote->setWordWrap(true);

  mLabelChangelog = new QLabel(this);
  mLabelChangelog->setGeometry(QRect(35, 260, 290, 41));
  mLabelChangelog->setText(
    QString("Find out the new features, enhancements and bug fixes of Webots R2022a in the <a style='color: #5DADE2;' "
            "href='https://cyberbotics.com/doc/reference/changelog-r2022'>changelog</a>."));
  mLabelChangelog->setOpenExternalLinks(true);
  mLabelChangelog->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelChangelog->setWordWrap(true);

  QPushButton *mCloseButton = new QPushButton(tr("Close"), this);
  mCloseButton->setGeometry(QRect(340, 270, 80, 25));
  connect(mCloseButton, &QPushButton::pressed, this, &WbUpdatedDialog::close);
}
