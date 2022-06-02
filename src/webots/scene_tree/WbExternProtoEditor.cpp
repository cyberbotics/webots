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
#include "WbInsertExternProtoDialog.hpp"
#include "WbProtoList.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>

WbExternProtoEditor::WbExternProtoEditor(QWidget *parent) : WbValueEditor(parent) {
  updateContents();
}

WbExternProtoEditor::~WbExternProtoEditor() {
}

void WbExternProtoEditor::updateContents() {
  printf("updating externproto pane contents\n");
  // QStringList items;
  // items << "car.proto"
  //      << "road.proto"
  //      << "other very long adsako sodk aosd kaosd kaso dsadasd das.proto";

  // clear layout
  for (int i = mLayout->count() - 1; i >= 0; --i) {
    QWidget *const widget = mLayout->itemAt(i)->widget();
    if (widget) {
      layout()->removeWidget(widget);
      delete widget;
    }
  }

  // TODO: add name lister for this vector as done in protowizard (nameList()) ??
  const QVector<QPair<QString, QString>> &externProto = WbProtoList::instance()->externProto();

  // Vector<bool> locked{true, false, true};

  mInsertButton = new QPushButton("Insert new", this);
  mInsertButton->setToolTip(tr("Declare additional EXTERNPROTO."));
  mInsertButton->setMaximumWidth(125);
  mLayout->addWidget(mInsertButton, 0, 0, 1, 2, Qt::AlignCenter);
  connect(mInsertButton, &QPushButton::pressed, this, &WbExternProtoEditor::insertExternProto);

  for (int i = 0; i < externProto.size(); ++i) {
    QLabel *const label = new QLabel(this);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setStyleSheet("border: 1px solid black");
    // note: since the label text might be elided based on the available space, the tooltip MUST contain the full name of the
    // proto, this information is used by removeExternProto to know what to remove
    label->setToolTip(externProto[i].first);
    label->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    setElidedText(label, externProto[i].first);

    mLayout->addWidget(label, i + 1, 0);

    if (true) {
      QPushButton *const removeButton = new QPushButton("-", this);
      removeButton->setToolTip(tr("Remove."));
      removeButton->setMaximumWidth(40);
      connect(removeButton, &QPushButton::pressed, this, &WbExternProtoEditor::removeExternProto);
      mLayout->addWidget(removeButton, i + 1, 1);
    }
  }

  QSpacerItem *spacer = new QSpacerItem(0, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
  mLayout->addItem(spacer, static_cast<int>(externProto.size()) + 1, 0, 1, 2);
  mLayout->setColumnStretch(0, 1);
}

void WbExternProtoEditor::insertExternProto() {
  printf("insertButtonCallback\n");
  WbInsertExternProtoDialog dialog(this);

  if (dialog.exec() == QDialog::Accepted)
    updateContents();  // regenerate panel
}

void WbExternProtoEditor::removeExternProto() {
  printf("removeExternProto\n");

  const QPushButton *const caller = dynamic_cast<QPushButton *>(sender());
  const int index = caller ? mLayout->indexOf(caller) : -1;
  if (index != -1 && index > 1) {
    assert(mLayout->itemAt(index - 1)->widget());  // must be preceeded by a QLabel widget
    const QString proto = mLayout->itemAt(index - 1)->widget()->toolTip();
    printf("removing: %s\n", proto.toUtf8().constData());
    WbProtoList::instance()->removeExternProto(proto);
    updateContents();  // regenerate panel
  }
}

void WbExternProtoEditor::setElidedText(QLabel *label, const QString &text) {
  QFontMetrics metrics(label->font());
  label->setText(metrics.elidedText(text, Qt::ElideRight, label->width() - 2));
}