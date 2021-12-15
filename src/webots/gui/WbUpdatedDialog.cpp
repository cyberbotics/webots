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

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

#include "WbUpdatedDialog.hpp"

WbUpdatedDialog::WbUpdatedDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("Welcome to Webots R2022a"));

  QPixmap webotsLogo("images:webots.png");
  QLabel *image = new QLabel(this);
  image->setGeometry(QRect(16, 15, 50, 50));
  image->setPixmap(webotsLogo);
  image->setScaledContents(true);

  QLabel *labelThanks = new QLabel(this);
  labelThanks->setGeometry(QRect(75, 30, 330, 30));
  labelThanks->setText(tr("<b>Thank you for using Webots R2022a.</b>"));
  labelThanks->setStyleSheet("font-size: 15px;");
  labelThanks->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  labelThanks->setWordWrap(true);

  QGroupBox *groupBox = new QGroupBox(this);
  groupBox->setGeometry(QRect(35, 65, 365, 240));
  groupBox->setTitle(tr("Important Note for Users of Previous Versions"));
  groupBox->setStyleSheet("QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } "
                          "QGroupBox::title {subcontrol-origin:  margin; subcontrol-position: top center; }");

  QLabel *labelNote = new QLabel(groupBox);
  labelNote->setGeometry(QRect(15, 20, 335, 210));
  labelNote->setText(
    tr("<p style=\"line-height:1.2\">"
       "All Webots worlds are now in the ENU axis system and all Webots devices, geometries, and PROTOs are in the FLU axis "
       "system. <br /> <br /> "
       "If your world or PROTO files include devices, geometries, or Webots PROTOs, "
       "then Webots will try to preserve the old behavior by rotating them. However, you may have to rotate some objects, "
       "fix bounding objects or adjust your controller by yourself. <br />Refer to the < a style = 'color: #5DADE2;' "
       "href='https://github.com/cyberbotics/webots/wiki/"
       "FLU-and-ENU-conversion-guide'>wiki</a> for more information. </p>"));
  labelNote->setOpenExternalLinks(true);
  labelNote->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  labelNote->setStyleSheet("border: none");
  labelNote->setWordWrap(true);

  QLabel *labelChangelog = new QLabel(this);
  labelChangelog->setGeometry(QRect(35, 313, 283, 44));
  labelChangelog->setText(
    tr("Find out the new features, enhancements and bug fixes of Webots R2022a in the <a style='color: #5DADE2;' "
       "href='https://cyberbotics.com/doc/reference/changelog-r2022'>changelog</a>."));
  labelChangelog->setOpenExternalLinks(true);
  labelChangelog->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  labelChangelog->setWordWrap(true);

  QPushButton *closeButton = new QPushButton(tr("Close"), this);
  closeButton->setGeometry(QRect(320, 315, 80, 30));
  connect(closeButton, &QPushButton::pressed, this, &WbUpdatedDialog::close);
}
