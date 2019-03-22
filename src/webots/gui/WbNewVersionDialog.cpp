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

#include "WbNewVersionDialog.hpp"

#include "WbApplicationInfo.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"
#include "WbVersion.hpp"

#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStyle>
#include <QtWidgets/QVBoxLayout>

static const QString gThemeNamesAndDescription[NUMBER_OF_THEMES][2] = {
  {"Classic",
   QObject::tr("The Classic theme is the theme you are already familiar with, Webots' look for the last few years.")},
  {"Night", QObject::tr("The Night theme features clean lines, flat design, and a subdued color palette with highlights in "
                        "just the right places to make interacting with Webots even more intuitive.")},
  {"Dusk", QObject::tr("The Dusk theme is a second modern dark theme, also featuring the same clean flat design, with a light "
                       "green accent color. It's the developers' favorite!")}};

bool WbNewVersionDialog::run() {
  WbNewVersionDialog dialog;
  return dialog.exec();
}

WbNewVersionDialog::WbNewVersionDialog() {
  style()->polish(this);

  const QString &versionString = WbApplicationInfo::version().toString();
  const WbVersion &version = WbApplicationInfo::version();
  setWindowTitle(tr("Welcome to Webots %1").arg(versionString));

  QVBoxLayout *vBoxLayout = new QVBoxLayout(this);

  QLabel *label = new QLabel(tr("More details about Webots %1 are listed <a style='color: #5DADE2;' "
                                "href='https://cyberbotics.com/doc/blog/Webots-%2-%3-release'>here</a>.")
                               .arg(versionString)
                               .arg(version.majorNumber())
                               .arg(QChar(version.minorNumber() + 'a')));
  label->setOpenExternalLinks(true);
  vBoxLayout->addWidget(label);
  vBoxLayout->addSpacing(10);

  QVBoxLayout *groupBoxLayout = new QVBoxLayout();

  // list of themes
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (i > 0)
      groupBoxLayout->addSpacing(10);
    QHBoxLayout *themeLayout = new QHBoxLayout();
    mRadioButtons[i] = new QRadioButton('&' + gThemeNamesAndDescription[i][0], this);
    mRadioButtons[i]->setMinimumWidth(80);
    mRadioButtons[i]->setObjectName(gThemeNamesAndDescription[i][0].toLower());
    if (i == 0 || WbPreferences::instance()
                    ->value("General/theme")
                    .toString()
                    .contains(gThemeNamesAndDescription[i][0], Qt::CaseInsensitive))
      mRadioButtons[i]->setChecked(true);
    themeLayout->addWidget(mRadioButtons[i]);
    QLabel *themeDescription = new QLabel(gThemeNamesAndDescription[i][1]);
    themeDescription->setWordWrap(true);
    themeLayout->addWidget(themeDescription, 1);
    groupBoxLayout->addLayout(themeLayout);
  }

  for (int i = 0; i < NUMBER_OF_THEMES; ++i)
    connect(mRadioButtons[i], &QRadioButton::toggled, this, &WbNewVersionDialog::updatePreview);

  QGroupBox *groupBox = new QGroupBox(tr("Themes:"));
  groupBox->setLayout(groupBoxLayout);
  vBoxLayout->addWidget(groupBox);

  // preview
  QGroupBox *previewBox = new QGroupBox(tr("Preview:"));
  mPreviewLabel = new QLabel();
  QHBoxLayout *previewLayout = new QHBoxLayout();
  previewLayout->addStretch();
  previewLayout->addWidget(mPreviewLabel);
  previewLayout->addStretch();
  previewBox->setLayout(previewLayout);
  vBoxLayout->addWidget(previewBox);

  // telemetry
  QGroupBox *telemetryBox = new QGroupBox(tr("Telemetry:"));
  QVBoxLayout *telemetryLayout = new QVBoxLayout();
  label = new QLabel(tr("We need your help to continue to improve Webots: more information <a style='color: #5DADE2;' "
                        "href='https://www.cyberbotics.com/doc/guide/telemetry'>here</a>."));
  label->setOpenExternalLinks(true);
  telemetryLayout->addWidget(label);
  telemetryLayout->addStretch();
  mTelemetryCheckBox = new QCheckBox(tr("Allow to send lightweight anonymous technical data to Webots developers."));
  mTelemetryCheckBox->setChecked(true);
  telemetryLayout->addWidget(mTelemetryCheckBox);
  telemetryBox->setLayout(telemetryLayout);
  vBoxLayout->addWidget(telemetryBox);

  // main button
  QPushButton *startButton = new QPushButton(tr("Start Webots with the selected theme."));
  vBoxLayout->addWidget(startButton);

  setLayout(vBoxLayout);
  connect(startButton, &QPushButton::clicked, this, &WbNewVersionDialog::startButtonPressed);
  updatePreview();
}

void WbNewVersionDialog::updatePreview() {
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (mRadioButtons[i]->isChecked()) {
      mPreviewLabel->setPixmap(
        QPixmap(WbStandardPaths::resourcesPath() + "images/themes/" + mRadioButtons[i]->objectName() + ".png"));
      break;
    }
  }
}

void WbNewVersionDialog::startButtonPressed() {
  QString theme;
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (mRadioButtons[i]->isChecked()) {
      theme = "webots_" + mRadioButtons[i]->objectName() + ".qss";
      break;
    }
  }
  WbPreferences::instance()->setValue("General/theme", theme);
  WbPreferences::instance()->setValue("General/telemetry", mTelemetryCheckBox->isChecked());
  // force the sync in case we restart just after quitting the dialog (see #7662)
  WbPreferences::instance()->sync();
  done(QDialog::Accepted);
}
