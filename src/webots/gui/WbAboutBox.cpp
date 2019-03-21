// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbAboutBox.hpp"

#include <QtCore/QDate>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

WbAboutBox::WbAboutBox(QWidget *parent) : QDialog(parent) {
  setModal(true);
  setWindowTitle(tr("About Webots"));
  QPixmap webotsLogo("images:webots.png");
  QGridLayout *layout = new QGridLayout();
  layout->setColumnMinimumWidth(0, 150);
  layout->setColumnMinimumWidth(1, 250);
  layout->setRowMinimumHeight(1, 30);

  QLabel *image = new QLabel();
  image->setPixmap(webotsLogo);
  layout->addWidget(image, 0, 0, 1, 1, Qt::AlignCenter);

  int year = QDate::currentDate().year();
  QLabel *licenseInfo = new QLabel();
  licenseInfo->setOpenExternalLinks(true);
  licenseInfo->setWordWrap(true);
  licenseInfo->setText(QString("<br> © Cyberbotics 1998-%1. Licensed under the <a style='color: #5DADE2;' "
                               "href='https://www.apache.org/licenses/LICENSE-2.0'>Apache License, Version 2.0</a>.")
                         .arg(year));
  layout->addWidget(licenseInfo, 0, 1, 1, 1, Qt::AlignCenter);

  QLabel *description = new QLabel();
  description->setOpenExternalLinks(true);
  description->setWordWrap(true);
  description->setText("<a style='color: #5DADE2;' href='https://cyberbotics.com'>"
                       "Our Website</a>&nbsp;&nbsp;");  // extra spaces for correct alignment

  layout->addWidget(description, 1, 0, 1, 1, Qt::AlignBottom | Qt::AlignRight);

  description = new QLabel();
  description->setOpenExternalLinks(true);
  description->setWordWrap(true);
  description->setText("<a style='color: #5DADE2;' href='https://cyberbotics.com/change_log'>Changelog</a>");
  layout->addWidget(description, 1, 1, 1, 1, Qt::AlignBottom | Qt::AlignCenter);

  setLayout(layout);
}
