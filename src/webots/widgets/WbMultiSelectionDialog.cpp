// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "WbMultiSelectionDialog.hpp"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

WbMultiSelectionDialog::WbMultiSelectionDialog(const QString &description, const QStringList &options, QWidget *parent) :
  QDialog(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);

  // Description label
  QLabel *label = new QLabel(description, this);
  layout->addWidget(label);

  // ok / cancel buttons
  QHBoxLayout *buttonLayout = new QHBoxLayout();
  QPushButton *cancelButton = new QPushButton(tr("cancel"), this);
  cancelButton->setFocus(Qt::TabFocusReason);
  connect(cancelButton, &QPushButton::pressed, this, &QDialog::reject);
  buttonLayout->addWidget(cancelButton);
  QPushButton *okButton = new QPushButton(tr("ok"), this);
  connect(okButton, &QPushButton::pressed, this, &WbMultiSelectionDialog::validate);
  buttonLayout->addWidget(okButton);
  layout->addLayout(buttonLayout);

  setMinimumHeight(sizeHint().height());
}

WbMultiSelectionDialog::~WbMultiSelectionDialog() {
}

void WbMultiSelectionDialog::validate() {
  // TODO
  accept();
}
