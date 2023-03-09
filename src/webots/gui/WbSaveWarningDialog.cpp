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

#include "WbSaveWarningDialog.hpp"

#include "WbPreferences.hpp"

#include <QtWidgets/QCheckBox>

WbSaveWarningDialog::WbSaveWarningDialog(const QString &world, bool hideCheckBox, bool reloading, QWidget *parent) :
  QMessageBox(parent) {
  setIcon(QMessageBox::Warning);
  setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
  if (reloading) {
    setWindowTitle(tr("Save before reloading the world?"));
    setText(tr("The '%1' file is not saved. Do you want to save it before reloading the world?").arg(world));
  } else {
    setWindowTitle(tr("Save before closing?"));
    setText(tr("The '%1' file is not saved. Do you want to save it before closing?").arg(world));
  }
  if (!hideCheckBox) {
    bool check = WbPreferences::instance()->value("General/checkWebotsUpdateOnStartup").toBool();
    QCheckBox *disableCheckBox =
      new QCheckBox(tr("Don't display this dialog again (you can re-enable it from the preferences)"), this);
    disableCheckBox->setChecked(!check);
    connect(disableCheckBox, &QCheckBox::toggled, this, &WbSaveWarningDialog::disableSaveWarning);
    setCheckBox(disableCheckBox);
  }
}

void WbSaveWarningDialog::disableSaveWarning(bool disable) {
  WbPreferences::instance()->setValue("General/disableSaveWarning", disable);
}
