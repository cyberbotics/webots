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
  setWindowTitle(tr("Welcome to Webots 2022a"));

  QPixmap webotsLogo("images:webots.png");
  QLabel *mImage = new QLabel(this);
  mImage->setGeometry(QRect(16, 15, 50, 50));
  mImage->setPixmap(webotsLogo);
  mImage->setScaledContents(true);

  mLabelThanks = new QLabel(this);
  mLabelThanks->setGeometry(QRect(75, 30, 330, 31));
  mLabelThanks->setText(tr("<b>Thank you for updating to Webots R2022a.</b>"));
  mLabelThanks->setStyleSheet("font-size: 15px;");
  mLabelThanks->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelThanks->setWordWrap(true);

  mGroupBox = new QGroupBox(this);
  mGroupBox->setGeometry(QRect(35, 65, 365, 230));
  mGroupBox->setTitle(tr("Important Note for Users of Previous Versions"));
  mGroupBox->setStyleSheet("QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } "
                           "QGroupBox::title {subcontrol-origin:  margin; subcontrol-position: top center; }");

  mLabelNote = new QLabel(mGroupBox);
  mLabelNote->setGeometry(QRect(15, 20, 335, 200));
  mLabelNote->setText(
    tr("<p style=\"line-height:1.2\"> <br />"
       "All Webots worlds are now in the ENU axis system and all Webots devices, geometries, and PROTOs are in the FLU axis "
       "system. <br /> <br /> "
       "If your world or PROTO includes devices, geometries, or Webots PROTOs, "
       "then Webots will try to preserve the old behavior by rotating them. However, you may have to rotate some objects, "
       "fix bounding objects or adjust your controller by yourself. <br />Refer to the < a style = 'color: #5DADE2;' "
       "href='https://github.com/cyberbotics/webots/wiki/"
       "FLU-and-ENU-conversion-guide'>wiki</a> for more information. </p>"));
  mLabelNote->setOpenExternalLinks(true);
  mLabelNote->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelNote->setStyleSheet("border: none");
  mLabelNote->setWordWrap(true);

  mLabelChangelog = new QLabel(this);
  mLabelChangelog->setGeometry(QRect(35, 305, 290, 41));
  mLabelChangelog->setText(
    tr("Find out the new features, enhancements and bug fixes of Webots R2022a in the <a style='color: #5DADE2;' "
       "href='https://cyberbotics.com/doc/reference/changelog-r2022'>changelog</a>."));
  mLabelChangelog->setOpenExternalLinks(true);
  mLabelChangelog->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  mLabelChangelog->setWordWrap(true);

  QPushButton *mCloseButton = new QPushButton(tr("Close"), this);
  mCloseButton->setGeometry(QRect(320, 310, 80, 25));
  connect(mCloseButton, &QPushButton::pressed, this, &WbUpdatedDialog::close);
}
