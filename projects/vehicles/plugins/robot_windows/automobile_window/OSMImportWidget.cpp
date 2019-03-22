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

#include "OSMImportWidget.hpp"

#include <QtCore/QProcess>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

OSMImportWidget::OSMImportWidget(QWidget *parent) : QWidget(parent) {
  mLayout = new QVBoxLayout(this);
  mPushButton = new QPushButton("Start OpenStreetMap importer graphical user interface", this);
  mLayout->addWidget(mPushButton);
  connect(mPushButton, &QPushButton::pressed, this, &OSMImportWidget::launchExecutable);
}

OSMImportWidget::~OSMImportWidget() {
}

void OSMImportWidget::launchExecutable() {
  // TODO, need to handle Windows
  QString gui_executable = qgetenv("WEBOTS_HOME") + QString("/resources/osm_importer/osm_gui/osm_gui");
  const QStringList arguments = QStringList(qgetenv("WEBOTS_HOME") + QString("/resources/osm_importer/importer.py"));
  QProcess::startDetached(gui_executable, arguments);
}
