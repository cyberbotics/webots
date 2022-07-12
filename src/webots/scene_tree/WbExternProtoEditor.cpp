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

#include "WbExternProtoEditor.hpp"

#include "WbActionManager.hpp"
#include "WbInsertExternProtoDialog.hpp"
#include "WbProtoManager.hpp"

#include <QtCore/QEvent>
#include <QtGui/QAction>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTextEdit>

WbExternProtoEditor::WbExternProtoEditor(QWidget *parent) : WbValueEditor(parent) {
  connect(this, &WbExternProtoEditor::changed, WbActionManager::instance()->action(WbAction::SAVE_WORLD), &QAction::setEnabled);
  connect(WbActionManager::instance()->action(WbAction::SAVE_WORLD), &QAction::triggered, this,
          &WbExternProtoEditor::updateContents, Qt::UniqueConnection);
  updateContents();
}

WbExternProtoEditor::~WbExternProtoEditor() {
}

void WbExternProtoEditor::updateContents(bool isChecked) {
  if (!isChecked)
    WbProtoManager::instance()->refreshExternProtoList();  // refresh the list only on world saves

  // clear layout
  for (int i = mLayout->count() - 1; i >= 0; --i) {
    QWidget *const widget = mLayout->itemAt(i)->widget();
    if (widget) {
      layout()->removeWidget(widget);
      delete widget;
    }
  }

  QTextEdit *const info = new QTextEdit("PROTO that may be imported during the execution must be declared");

  info->setWordWrapMode(QTextOption::WordWrap);
  info->setReadOnly(true);
  info->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  info->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  info->setTextInteractionFlags(Qt::NoTextInteraction);
  info->setAlignment(Qt::AlignCenter);
  info->setMinimumHeight(40);
  info->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  info->setFont(QFont(info->font().family(), 10));
  info->setStyleSheet("background-color: transparent;");
  mLayout->addWidget(info, 0, 0, 1, 2);

  mInsertButton = new QPushButton("Insert new", this);
  mInsertButton->setToolTip(tr("Declare additional Ephemeral EXTERNPROTO."));
  mInsertButton->setMaximumWidth(125);
  mLayout->addWidget(mInsertButton, 1, 0, 1, 2, Qt::AlignCenter);
  mLayout->setRowStretch(1, 1);
  mLayout->setColumnStretch(1, 1);
  connect(mInsertButton, &QPushButton::pressed, this, &WbExternProtoEditor::insertExternProto);

  const QVector<WbExternProtoInfo *> &externProto = WbProtoManager::instance()->externProto();
  int row = 2;
  for (int i = 0; i < externProto.size(); ++i) {
    if (!externProto[i]->isEphemeral())
      continue;

    QLabel *const label = new QLabel(this);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setObjectName("externProtoEditor");
    label->setToolTip(externProto[i]->url());
    label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    label->setText(externProto[i]->name());

    mLayout->addWidget(label, row, 0);
    mLayout->setRowStretch(row, 1);

    QIcon icon = QIcon();
    icon.addFile("enabledIcons:delete_button.png", QSize(), QIcon::Normal);
    QPushButton *const removeButton = new QPushButton(this);
    removeButton->setIcon(QIcon(icon));
    removeButton->setToolTip(tr("Remove."));
    removeButton->setMaximumWidth(40);
    connect(removeButton, &QPushButton::pressed, this, &WbExternProtoEditor::removeExternProto);
    mLayout->addWidget(removeButton, row, 1);

    row++;
  }

  QSpacerItem *spacer = new QSpacerItem(0, 1000, QSizePolicy::Expanding, QSizePolicy::Expanding);
  mLayout->addItem(spacer, row, 0, 1, 2);
}

void WbExternProtoEditor::insertExternProto() {
  WbInsertExternProtoDialog dialog(this);

  if (dialog.exec() == QDialog::Accepted) {
    updateContents();  // regenerate panel
    emit changed(true);
  }
}

void WbExternProtoEditor::removeExternProto() {
  const QPushButton *const caller = qobject_cast<QPushButton *>(sender());
  const int index = caller ? mLayout->indexOf(caller) : -1;
  if (index != -1 && index > 1) {
    assert(mLayout->itemAt(index - 1)->widget());  // must be preceeded by a QLabel widget
    const QLabel *label = qobject_cast<QLabel *>(mLayout->itemAt(index - 1)->widget());
    if (label) {
      const QString proto = label->text();
      WbProtoManager::instance()->removeExternProto(proto, true);
      updateContents();  // regenerate panel

      emit changed(true);
    }
  }
}
