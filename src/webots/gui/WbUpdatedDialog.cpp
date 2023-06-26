// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
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
  setWindowTitle(tr("Welcome to Webots R2023b"));

  QPixmap webotsLogo("images:webots.png");
  QLabel *image = new QLabel(this);
  image->setGeometry(QRect(16, 15, 50, 50));
  image->setPixmap(webotsLogo);
  image->setScaledContents(true);

  QLabel *label = new QLabel(this);
  label->setGeometry(QRect(75, 30, 330, 30));
  label->setText(tr("<b>Thank you for using Webots R2023b.</b>"));
  label->setStyleSheet("font-size: 15px;");
  label->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  label->setWordWrap(true);

  QGroupBox *groupBox = new QGroupBox(this);
  groupBox->setGeometry(QRect(35, 65, 365, 105));
  groupBox->setTitle(tr("Important Note for Users of Previous Versions"));
  groupBox->setStyleSheet("QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } "
                          "QGroupBox::title {subcontrol-origin:  margin; subcontrol-position: top center; }");

  label = new QLabel(groupBox);
  label->setGeometry(QRect(15, 20, 315, 70));
  label->setText(tr("<p style=\"line-height:1.2\">"
                    "Some features introduced in this version may break backward compatibility with your worlds and PROTO "
                    "models. <br /> <br /> "
                    "Please refer to the <a style='color: #5DADE2;' "
                    "href='https://cyberbotics.com/doc/guide/upgrading-webots'>upgrade guide</a>.</p>"));
  label->setOpenExternalLinks(true);
  label->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  label->setStyleSheet("border: none");
  label->setWordWrap(true);

  label = new QLabel(this);
  label->setGeometry(QRect(35, 200, 283, 44));
  label->setText(tr("Find out the new features, enhancements and bug fixes of Webots R2023b in the <a style='color: #5DADE2;' "
                    "href='https://cyberbotics.com/doc/reference/changelog-r2023'>changelog</a>."));
  label->setOpenExternalLinks(true);
  label->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);
  label->setWordWrap(true);

  QPushButton *closeButton = new QPushButton(tr("Close"), this);
  closeButton->setGeometry(QRect(320, 202, 80, 30));
  connect(closeButton, &QPushButton::pressed, this, &WbUpdatedDialog::close);
}
