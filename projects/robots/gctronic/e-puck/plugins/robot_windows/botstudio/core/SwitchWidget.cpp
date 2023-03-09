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

/*
 * Description:  Implementation of the SwitchWidget.hpp functions
 */

#include "SwitchWidget.hpp"

#include "Automaton.hpp"
#include "Model.hpp"
#include "RobotActuatorCommand.hpp"
#include "RobotConditionWidget.hpp"
#include "RobotObjectFactory.hpp"
#include "RobotSensorCondition.hpp"
#include "RobotStateWidget.hpp"
#include "RobotViewWidget.hpp"
#include "State.hpp"
#include "Transition.hpp"

#include <QtWidgets/QStackedLayout>

SwitchWidget::SwitchWidget(QWidget *parent) : QWidget(parent) {
  mModel = Model::instance();

  mStateWidget = RobotObjectFactory::instance()->createRobotStateWidget(this);
  mConditionWidget = RobotObjectFactory::instance()->createRobotConditionWidget(this);
  mViewWidget = RobotObjectFactory::instance()->createRobotViewWidget(this);

  mStackedLayout = new QStackedLayout;
  mStackedLayout->addWidget(mStateWidget);
  mStackedLayout->addWidget(mConditionWidget);
  mStackedLayout->addWidget(mViewWidget);

  setLayout(mStackedLayout);

  selectWidget();
  connect(mModel->automaton(), SIGNAL(selectionChanged()), this, SLOT(selectWidget()));
}

SwitchWidget::~SwitchWidget() {
  delete mStateWidget;
  delete mConditionWidget;
  delete mViewWidget;
  delete mStackedLayout;
}

void SwitchWidget::selectWidget() {
  AutomatonObject *object = mModel->automaton()->findSelectedObject();
  State *state = dynamic_cast<State *>(object);
  Transition *transition = dynamic_cast<Transition *>(object);

  if (state) {
    RobotActuatorCommand *actuatorCommand = state->actuatorCommand();
    mStateWidget->setActuatorCommand(actuatorCommand);
    mStackedLayout->setCurrentIndex(StateWidget);
  } else if (transition) {
    RobotSensorCondition *sensorCondition = transition->sensorCondition();
    mConditionWidget->setSensorCondition(sensorCondition);
    mStackedLayout->setCurrentIndex(TransitionWidget);
  } else
    mStackedLayout->setCurrentIndex(RobotWidget);
}

void SwitchWidget::updateValues() {
  mViewWidget->updateValues();
}
