// Copyright 1996-2022 Cyberbotics Ltd.
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

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QSpacerItem>

#include "WbMainWindow.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"

QString groupBoxStyleSheet;

WbShareWindow::WbShareWindow(QWidget *parent) : QDialog(parent) {
  QString uploadUrl = "https://testing.webots.cloud"; //WbPreferences::instance()->value("Network/uploadUrl").toString();
  if (uploadUrl.contains("//"))
    uploadUrl = uploadUrl.split("//")[1];
  groupBoxStyleSheet = "QGroupBox {border: 1px solid gray;border-radius: 9px;margin-top: 0.5em; } QGroupBox::title "
                       "{subcontrol-origin:  margin; subcontrol-position: top center; }";
  this->setWindowTitle(tr("Share your simulation online"));

  QGridLayout *layout = new QGridLayout(this);

  QLabel *labelIntro = new QLabel(this);
  labelIntro->setOpenExternalLinks(true);
  labelIntro->setText(tr("<html><head/><body><p>Publish your simulation on <a href=\"https://%1/\"><span "
                         "style=\" text-decoration: underline; color:#5dade2;\">%1</span></a>.</p></body></html>")
                        .arg(uploadUrl));
  layout->addWidget(labelIntro, 3, 0, 1, 2);

  QSpacerItem *verticalSpacer = new QSpacerItem(100, 10);
  layout->addItem(verticalSpacer, 4, 0, 1, 2);

  QPushButton *pushButtonAnimation = new QPushButton(this);
  pushButtonAnimation->setFocusPolicy(Qt::NoFocus);
  pushButtonAnimation->setText(tr("Record and\n"
                                  "upload &animation"));
  layout->addWidget(pushButtonAnimation, 5, 1, 1, 1);

  QPushButton *pushButtonScene = new QPushButton(this);
  pushButtonScene->setFocusPolicy(Qt::NoFocus);
  pushButtonScene->setText(tr("Upload your scene"));
  pushButtonScene->setFixedHeight(pushButtonAnimation->height() + 9);
  layout->addWidget(pushButtonScene, 5, 0, 1, 1);

  WbMainWindow *mainWindow = dynamic_cast<WbMainWindow *>(parentWidget());

  QCheckBox *checkBoxSave = new QCheckBox(this);
  checkBoxSave->setFocusPolicy(Qt::NoFocus);
  checkBoxSave->setText(tr("Save as local files"));
  mainWindow->CheckBoxStatus(false);
  layout->addWidget(checkBoxSave, 6, 0, 1, 1);

  connect(checkBoxSave, &QCheckBox::stateChanged, mainWindow, &WbMainWindow::CheckBoxStatus);
  connect(pushButtonScene, &QPushButton::pressed, mainWindow, &WbMainWindow::uploadScene);
  connect(pushButtonScene, &QPushButton::pressed, this, &WbShareWindow::close);
  connect(pushButtonAnimation, &QPushButton::pressed, mainWindow, &WbMainWindow::startAnimationRecording);
  connect(pushButtonAnimation, &QPushButton::pressed, this, &WbShareWindow::close);
}

WbLinkWindow::WbLinkWindow(QWidget *parent) : QDialog(parent) {
  this->setWindowTitle(tr("Share your simulation"));

  mGroupBoxLink = new QGroupBox(this);
  mGroupBoxLink->setTitle(tr("Upload successful"));
  mGroupBoxLink->setStyleSheet(groupBoxStyleSheet);
  mGroupBoxLink->setGeometry(QRect(10, 20, 291, 61));

  QGridLayout *layout = new QGridLayout(mGroupBoxLink);
  mLabelLink = new QLabel(mGroupBoxLink);
  mLabelLink->setAlignment(Qt::AlignCenter);
  mLabelLink->setStyleSheet("border: none;");
  mLabelLink->setOpenExternalLinks(true);
  mLabelLink->setMinimumHeight(15);
  layout->addWidget(mLabelLink, 0, 0, 1, 1);
}

void WbLinkWindow::reject() {
  QDir dir(WbStandardPaths::webotsTmpPath() + "textures/");  // remove tmp files
  dir.removeRecursively();
  const QStringList extensions = {".html", ".x3d", ".json"};
  foreach (QString extension, extensions)
    QFile::remove(WbStandardPaths::webotsTmpPath() + "cloud_export" + extension);

  QDialog::reject();
}

void WbLinkWindow::setLabelLink(QString url) {
  mLabelLink->setText(tr("Link: <a style='color: #5DADE2;' href='%1?upload=webots'>%1</a>").arg(url));
  mGroupBoxLink->adjustSize();
}
