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
  mImage->setGeometry(QRect(16, 15, 50, 50));
  mImage->setPixmap(webotsLogo);
  mImage->setScaledContents(true);

  mLabelThanks = new QLabel(this);
  mLabelThanks->setGeometry(QRect(70, 30, 330, 31));
  mLabelThanks->setText(QString("<b>Thank you for updating to the last version of Webots.</b>"));
  mLabelThanks->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelThanks->setWordWrap(true);

  mGroupBox = new QGroupBox(this);
  mGroupBox->setGeometry(QRect(35, 65, 365, 230));
  mGroupBox->setTitle(QString("Important Note for previous users"));
  mGroupBox->setStyleSheet("QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } "
                           "QGroupBox::title {subcontrol-origin:  margin; subcontrol-position: top center; top; }");

  mLabelNote = new QLabel(mGroupBox);
  mLabelNote->setGeometry(QRect(15, 20, 345, 200));
  mLabelNote->setText(QString(
    "<p style=\"line-height:1.2\"> <br />"
    "Webots axis systems and objects orientation conventions respectively <b>changed to ENU and FLU</b>. All the PROTOs and "
    "worlds included in the Webots package are now in ENU and FLU. <br /> <br /> <b>If you are using your own world or "
    "PROTO</b>, "
    "Webots will "
    "rotate all the elements of your world when you open it in this new version. However, you may have to rotate some "
    "objects, fix bounding objects by yourself or adjust your controller. Refer to the <a style='color: #5DADE2;' "
    "href='https://cyberbotics.com/doc/reference/changelog'>wiki</a> for more information. </p>"));
  mLabelNote->setOpenExternalLinks(true);
  mLabelNote->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelNote->setStyleSheet("border: none");
  mLabelNote->setWordWrap(true);

  mLabelChangelog = new QLabel(this);
  mLabelChangelog->setGeometry(QRect(35, 300, 290, 41));
  mLabelChangelog->setText(
    QString("Find out the new features, enhancements and bug fixes of Webots R2022a in the <a style='color: #5DADE2;' "
            "href='https://cyberbotics.com/doc/reference/changelog-r2022'>changelog</a>."));
  mLabelChangelog->setOpenExternalLinks(true);
  mLabelChangelog->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelChangelog->setWordWrap(true);

  QPushButton *mCloseButton = new QPushButton(tr("Close"), this);
  mCloseButton->setGeometry(QRect(320, 310, 80, 25));
  connect(mCloseButton, &QPushButton::pressed, this, &WbUpdatedDialog::close);
}
