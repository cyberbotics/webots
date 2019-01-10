// Copyright 1996-2018 Cyberbotics Ltd.
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

  QVBoxLayout *vBoxlayout = new QVBoxLayout(this);

  QLabel *label = new QLabel(
    tr("Thank you for upgrading to Webots %1."
       "<br>We have introduced some brand new themes to customize your experience with Webots."
       "<br>Please select which theme you wish to use (don't worry, you will be able to change it later from the preferences)."
       "<br><br>Read more about the new changes in Webots %1 <a style='color: #5DADE2;' "
       "href='https://www.cyberbotics.com/doc/blog/Webots-%2-%3-release'>here</a>.")
      .arg(versionString)
      .arg(version.majorNumber())
      .arg(QChar(version.minorNumber() + 'a')));
  label->setOpenExternalLinks(true);
  vBoxlayout->addWidget(label);
  vBoxlayout->addSpacing(10);

  QVBoxLayout *groupBoxlayout = new QVBoxLayout();

  // list of themes
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (i > 0)
      groupBoxlayout->addSpacing(10);
    QHBoxLayout *themeLayout = new QHBoxLayout();
    mRadioButtons[i] = new QRadioButton('&' + gThemeNamesAndDescription[i][0], this);
    mRadioButtons[i]->setMinimumWidth(80);
    mRadioButtons[i]->setObjectName(gThemeNamesAndDescription[i][0].toLower());
    if (i == 0)
      mRadioButtons[i]->setChecked(true);
    connect(mRadioButtons[i], &QRadioButton::toggled, this, &WbNewVersionDialog::updatePreview);
    themeLayout->addWidget(mRadioButtons[i]);
    QLabel *themeDescription = new QLabel(gThemeNamesAndDescription[i][1]);
    themeDescription->setWordWrap(true);
    themeLayout->addWidget(themeDescription, 1);
    groupBoxlayout->addLayout(themeLayout);
  }

  QGroupBox *groupBox = new QGroupBox(tr("Themes:"));
  groupBox->setLayout(groupBoxlayout);
  vBoxlayout->addWidget(groupBox);

  // preview
  QGroupBox *previewBox = new QGroupBox(tr("Preview:"));
  mPreviewLabel = new QLabel();
  QHBoxLayout *previewLayout = new QHBoxLayout();
  previewLayout->addStretch();
  previewLayout->addWidget(mPreviewLabel);
  previewLayout->addStretch();
  previewBox->setLayout(previewLayout);
  vBoxlayout->addWidget(previewBox);

  // main button
  mStartButton = new QPushButton(tr("Start Webots with the selected theme."));
  vBoxlayout->addWidget(mStartButton);

  setLayout(vBoxlayout);
  connect(mStartButton, &QPushButton::clicked, this, &WbNewVersionDialog::selectTheme);
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

void WbNewVersionDialog::selectTheme() {
  QString theme;
  for (int i = 0; i < NUMBER_OF_THEMES; ++i) {
    if (mRadioButtons[i]->isChecked()) {
      theme = "webots_" + mRadioButtons[i]->objectName() + ".qss";
      break;
    }
  }

  WbPreferences::instance()->setValue("General/theme", theme);
  // force the sync in case we restart just after quitting the dialog (see #7662)
  WbPreferences::instance()->sync();
  done(QDialog::Accepted);
}
