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

#include "WbAboutBox.hpp"

#include "WbApplicationInfo.hpp"
#include "WbDesktopServices.hpp"
#include "WbVersion.hpp"

#include <QtCore/QDate>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

WbAboutBox::WbAboutBox(QWidget *parent) : QDialog(parent) {
  setModal(true);
  setWindowTitle(tr("About Webots"));
  QPixmap webotsLogo("images:webots.png");
  QVBoxLayout *layout = new QVBoxLayout();
  QGridLayout *firstRowLayout = new QGridLayout();
  firstRowLayout->setColumnMinimumWidth(0, 150);
  firstRowLayout->setColumnMinimumWidth(1, 250);

  QLabel *image = new QLabel();
  image->setPixmap(webotsLogo);
  firstRowLayout->addWidget(image, 0, 0, 1, 1, Qt::AlignCenter);

  QLabel *versionInfo = new QLabel();
  connect(versionInfo, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  QDateTime releaseTimestamp;
  releaseTimestamp.setSecsSinceEpoch(WbApplicationInfo::releaseDate());
  versionInfo->setText(QString("<p style='font-size: large;'><b>Webots %1</b></p>"
                               "<p>Date: %2</p><p>Webots is free software<br>Licensed under the <a style='color: #5DADE2;' "
                               "href='https://www.apache.org/licenses/LICENSE-2.0'>Apache License, Version 2.0</a></p>"
                               "<p>© Cyberbotics 1998-%3</p>")
                         .arg(WbApplicationInfo::version().toString())
                         .arg(releaseTimestamp.date().toString("MMMM dd, yyyy"))
                         .arg(QDate::currentDate().year()));
  firstRowLayout->addWidget(versionInfo, 0, 1, 1, 2, Qt::AlignCenter);

  QGridLayout *secondRowLayout = new QGridLayout();
  secondRowLayout->setRowMinimumHeight(0, 20);
  QLabel *description = new QLabel();
  connect(description, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  description->setWordWrap(true);
  description->setText("<a style='color: #5DADE2;' href='https://cyberbotics.com'>Website</a>");
  secondRowLayout->addWidget(description, 1, 0, 1, 1, Qt::AlignBottom | Qt::AlignCenter);

  description = new QLabel();
  connect(description, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  description->setWordWrap(true);
  description->setText("<a style='color: #5DADE2;' href='https://cyberbotics.com/doc/reference/changelog'>Changelog</a>");
  secondRowLayout->addWidget(description, 1, 1, 1, 1, Qt::AlignBottom | Qt::AlignCenter);

  description = new QLabel();
  connect(description, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  description->setWordWrap(true);
  description->setText("<a style='color: #5DADE2;' href='https://cyberbotics.com/doc/guide/privacy-policy'>Privacy Policy</a>");
  secondRowLayout->addWidget(description, 1, 2, 1, 1, Qt::AlignBottom | Qt::AlignCenter);

  layout->addLayout(firstRowLayout);
  layout->addLayout(secondRowLayout);
  setLayout(layout);
}
