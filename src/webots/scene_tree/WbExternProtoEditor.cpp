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

#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

WbExternProtoEditor::WbExternProtoEditor(QWidget *parent) : WbValueEditor(parent) {
  QWidget *pane = new QWidget(this);
  pane->setObjectName("NodeEditorBackground");  // TODO: css stuff

  QVBoxLayout *const layout = new QVBoxLayout(pane);

  QStringList items;
  items << "car.proto"
        << "road.proto"
        << "other.proto";

  foreach (const QString &item, items) { layout->addWidget(new QLabel(item, this)); }

  QPushButton *editButton = new QPushButton(tr("Edit"), this);
  editButton->setStatusTip(tr("Something"));
  editButton->setToolTip(editButton->statusTip());
  connect(editButton, &QPushButton::pressed, this, &WbExternProtoEditor::buttonCallback);
  layout->addWidget(editButton);

  mLayout->addLayout(layout, 1, 1);
  /*
  QGridLayout *const layout = new QGridLayout(pane);
  layout->addWidget(new QLabel("DEF:", this), 0, 0);
  layout->addWidget(mDefEdit, 0, 1);
  layout->addWidget(mUseCount, 1, 1);
  layout->addWidget(mNbTriangles, 2, 1);

  layout->addWidget(mShowResizeHandlesLabel, 4, 0);
  layout->addWidget(mShowResizeHandlesCheckBox, 4, 1);

  // setup layout size policy in order to put all the widgets top - left
  // vertically
  QWidget *vStretch = new QWidget(this);
  layout->addWidget(vStretch, 4, 0);
  layout->setRowStretch(4, 1);
  // horizontally
  QWidget *hStretch = new QWidget(this);
  layout->addWidget(hStretch, 0, 2);
  layout->setColumnStretch(2, 1);

  // Main layout
  mStackedWidget->addWidget(nodePane);
  mStackedWidget->addWidget(new QWidget(this));  // empty pane
  mLayout->addWidget(mStackedWidget, 1, 1);
  */
}

WbExternProtoEditor::~WbExternProtoEditor() {
}

void WbExternProtoEditor::buttonCallback() {
}
