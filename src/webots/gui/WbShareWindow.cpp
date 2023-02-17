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

#include "WbShareWindow.hpp"

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtGui/QDesktopServices>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>

#include "WbMainWindow.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"

QString groupBoxStyleSheet;

WbShareWindow::WbShareWindow(QWidget *parent) : QDialog(parent) {
  QString uploadUrl = WbPreferences::instance()->value("Network/uploadUrl").toString();
  if (uploadUrl.contains("//"))
    uploadUrl = uploadUrl.split("//")[1];
  groupBoxStyleSheet = "QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } QGroupBox::title "
                       "{subcontrol-origin:  margin; subcontrol-position: top center; }";
  this->setWindowTitle(tr("Share your simulation"));
  this->setMinimumSize(325, 150);

  QGridLayout *layout = new QGridLayout(this);

  QRadioButton *boxButtons[2];
  boxButtons[0] = new QRadioButton("Upload to " + uploadUrl, this);
  layout->addWidget(boxButtons[0], 1, 0, 2, 2);

  boxButtons[1] = new QRadioButton("Save as local files", this);
  layout->addWidget(boxButtons[1], 3, 0, 2, 2);

  QSpacerItem *verticalSpacer = new QSpacerItem(100, 10);
  layout->addItem(verticalSpacer, 5, 0, 1, 2);

  boxButtons[0]->setChecked(true);

  QPushButton *pushButtonAnimation = new QPushButton(this);
  pushButtonAnimation->setFocusPolicy(Qt::NoFocus);
  pushButtonAnimation->setText(tr("Record and\nexport animation"));
  pushButtonAnimation->setFixedHeight(46);
  layout->addWidget(pushButtonAnimation, 6, 1, 1, 1);

  QPushButton *pushButtonScene = new QPushButton(this);
  pushButtonScene->setFocusPolicy(Qt::NoFocus);
  pushButtonScene->setText(tr("Export scene"));
  pushButtonScene->setFixedHeight(46);
  layout->addWidget(pushButtonScene, 6, 0, 1, 1);

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());
  mainWindow->setSaveLocally(false);

  connect(boxButtons[1], &QRadioButton::toggled, mainWindow, &WbMainWindow::setSaveLocally);
  connect(pushButtonScene, &QPushButton::pressed, mainWindow, &WbMainWindow::uploadScene);
  connect(pushButtonScene, &QPushButton::pressed, this, &WbShareWindow::close);
  connect(pushButtonAnimation, &QPushButton::pressed, mainWindow, &WbMainWindow::startAnimationRecording);
  connect(pushButtonAnimation, &QPushButton::pressed, this, &WbShareWindow::close);
}

WbLinkWindow::WbLinkWindow(QWidget *parent) : QDialog(parent) {
  this->setWindowTitle(tr("Upload Successful"));
  this->setMinimumSize(325, 110);

  QGridLayout *layout = new QGridLayout(this);

  QSpacerItem *verticalSpacer = new QSpacerItem(100, 8);
  layout->addItem(verticalSpacer, 0, 0, 1, 3);

  QPushButton *pushButtonOpenLink = new QPushButton(this);
  pushButtonOpenLink->setFocusPolicy(Qt::NoFocus);
  pushButtonOpenLink->setText(tr("Open in Browser"));
  pushButtonOpenLink->setFixedWidth(pushButtonOpenLink->width() + 50);
  layout->addWidget(pushButtonOpenLink, 1, 1, 1, 1);
  connect(pushButtonOpenLink, &QPushButton::pressed, this, &WbLinkWindow::openUrl);
  connect(pushButtonOpenLink, &QPushButton::pressed, this, &WbShareWindow::close);

  QLabel *labelInfo = new QLabel(this);
  labelInfo->setText(tr("<html><head/><body><p style=\" text-align: center;\">Make sure to click Open in Browser "
                        "to associate<br>the upload with your webots.cloud account.</a></p></body></html>"));
  layout->addWidget(labelInfo, 2, 0, 1, 3);
}

void WbLinkWindow::reject() {
  QDir dir(WbStandardPaths::webotsTmpPath() + "textures/");  // remove tmp files
  dir.removeRecursively();
  const QStringList extensions = {".html", ".x3d", ".json", ".jpg"};
  foreach (QString extension, extensions)
    QFile::remove(WbStandardPaths::webotsTmpPath() + "cloud_export" + extension);

  QDialog::reject();
}

void WbLinkWindow::setUploadUrl(const QString &url) {
  mUrl = url + "?upload=webots";
}

void WbLinkWindow::openUrl() {
  if (QDesktopServices::openUrl(QUrl(mUrl, QUrl::TolerantMode)))
    return;
}
