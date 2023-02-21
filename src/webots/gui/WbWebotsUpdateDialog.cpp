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

#include "WbWebotsUpdateDialog.hpp"

#include "WbApplicationInfo.hpp"
#include "WbDesktopServices.hpp"
#include "WbPreferences.hpp"
#include "WbVersion.hpp"
#include "WbWebotsUpdateManager.hpp"

#include <QtWidgets/QtWidgets>

WbWebotsUpdateDialog::WbWebotsUpdateDialog(bool displayCheckBox, QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("Check for updates"));

  QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok, Qt::Horizontal, this);
  connect(buttonBox, &QDialogButtonBox::accepted, this, &WbWebotsUpdateDialog::accept);

  mLabel = new QLabel(tr("Check for updates..."), this);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(mLabel);

  if (displayCheckBox) {
    bool check = WbPreferences::instance()->value("General/checkWebotsUpdateOnStartup").toBool();
    QCheckBox *disableCheckBox =
      new QCheckBox(tr("Don't display this dialog again (you can re-enable it from the preferences)"), this);
    disableCheckBox->setChecked(!check);
    connect(disableCheckBox, &QCheckBox::toggled, this, &WbWebotsUpdateDialog::updatePreferences);
    mainLayout->addWidget(disableCheckBox);
  }

  mainLayout->addWidget(buttonBox);

  setLayout(mainLayout);

  WbWebotsUpdateManager *webotsUpdateManager = WbWebotsUpdateManager::instance();
  if (webotsUpdateManager->isTargetVersionAvailable() || webotsUpdateManager->error().size() > 0)
    updateLabel();
  else
    connect(webotsUpdateManager, &WbWebotsUpdateManager::targetVersionAvailable, this, &WbWebotsUpdateDialog::updateLabel);
}

WbWebotsUpdateDialog::~WbWebotsUpdateDialog() {
}

void WbWebotsUpdateDialog::updateLabel() {
  WbWebotsUpdateManager *webotsUpdateManager = WbWebotsUpdateManager::instance();

  if (webotsUpdateManager->error().size() > 0) {
    mLabel->setText(webotsUpdateManager->error());
    return;
  }

  if (!webotsUpdateManager->isTargetVersionAvailable())
    return;

  const WbVersion &targetVersion = webotsUpdateManager->targetVersion();
  const WbVersion &currentVersion = WbApplicationInfo::version();

  if (currentVersion < targetVersion) {
    connect(mLabel, &QLabel::linkActivated, &WbDesktopServices::openUrl);
    mLabel->setText(tr("<b>A new version of Webots is available.</b><br/><br/>"
                       "Webots %1 is available for download (you are currently using Webots %2).<br/>"
                       "Download the new version: "
                       "<a href=\"https://cyberbotics.com/download\">https://cyberbotics.com/download</a><br/>"
                       "Check the changelog: "
                       "<a "
                       "href=\"https://cyberbotics.com/doc/reference/changelog\">https://cyberbotics.com/doc/"
                       "reference/changelog</a>")
                      .arg(targetVersion.toString())
                      .arg(currentVersion.toString()));
  } else
    mLabel->setText(tr("Your version of Webots (%1) is up-to-date.").arg(currentVersion.toString()));
}

void WbWebotsUpdateDialog::updatePreferences(bool doNotDisplayDialog) {
  WbPreferences::instance()->setValue("General/checkWebotsUpdateOnStartup", !doNotDisplayDialog);
}
