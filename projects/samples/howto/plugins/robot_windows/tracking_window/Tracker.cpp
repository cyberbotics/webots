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

#include "Tracker.hpp"

#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsItem>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>

#include <webots/supervisor.h>

using namespace webotsQtUtils;

static const int DIMENSION = 400;
static const int MARGIN = 30;
static const int X = 0;
static const int Z = 2;

Tracker::Tracker() : MainWindow() {
  QGraphicsScene *scene = new QGraphicsScene(this);

  // outline
  scene->addRect(-DIMENSION / 2 + MARGIN, -DIMENSION / 2 + MARGIN, DIMENSION - 2 * MARGIN, DIMENSION - 2 * MARGIN);

  // robot representation
  for (int i = 0; i < NROBOTS; i++)
    mRobotRepresentation[i] = static_cast<QGraphicsItem *>(scene->addEllipse(0, 0, 15, 15));

  QGraphicsView *view = new QGraphicsView(scene);
  setCentralWidget(view);
}

Tracker::~Tracker() {
}

QSize Tracker::sizeHint() const {
  static QSize size(DIMENSION, DIMENSION);
  return size;
}

void Tracker::readSensors() {
  for (int i = 0; i < NROBOTS; i++) {
    QString robotName = QString("ROBOT%1").arg(i + 1);
    WbNodeRef r = wb_supervisor_node_get_from_def(robotName.toLatin1().data());
    WbFieldRef f = wb_supervisor_node_get_field(r, "translation");
    const double *position = wb_supervisor_field_get_sf_vec3f(f);
    mRobotRepresentation[i]->setPos(position[X] * (DIMENSION - 2 * MARGIN), position[Z] * (DIMENSION - 2 * MARGIN));
  }
}

void Tracker::writeActuators() {
}
