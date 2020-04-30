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

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QVBoxLayout>

WbMultiSelectionDialog::WbMultiSelectionDialog(const QString &description, const QStringList &options,
                                               const QStringList &enabledOptions, QWidget *parent) :
  QDialog(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);

  // Description label
  QLabel *label = new QLabel(description, this);
  layout->addWidget(label);

  // checkboxes
  QScrollArea *scrollArea = new QScrollArea(this);
  scrollArea->setWidgetResizable(true);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  QWidget *scrollAreaWidget = new QWidget(scrollArea);
  QVBoxLayout *scrollAreaLayout = new QVBoxLayout(scrollAreaWidget);
  scrollAreaWidget->setLayout(scrollAreaLayout);
  scrollArea->setWidget(scrollAreaWidget);

  for (int i = 0; i < options.size(); ++i) {
    QCheckBox *checkbox = new QCheckBox(options.at(i), this);
    scrollAreaLayout->addWidget(checkbox);
    if (enabledOptions.contains(options.at(i)))
      checkbox->setChecked(true);
    mCheckboxes.append(checkbox);
  }
  layout->addWidget(scrollArea);

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

void WbMultiSelectionDialog::validate() {
  mEnabledOptions.clear();
  for (int i = 0; i < mCheckboxes.size(); ++i) {
    if (mCheckboxes.at(i)->isChecked())
      mEnabledOptions.append(mCheckboxes.at(i)->text());
  }
  accept();
}
