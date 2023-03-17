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

#include "WbSimulationStateIndicator.hpp"

#include "WbApplication.hpp"
#include "WbGuiRefreshOracle.hpp"
#include "WbSimulationState.hpp"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QScrollArea>

#include <cmath>

WbSimulationStateIndicator::WbSimulationStateIndicator(QWidget *parent) : QFrame(parent) {
  setObjectName("simulationIndicatorFrame");
  setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  mTimeLabel = new QLabel(this);
  mSpeedLabel = new QLabel(this);
  QLabel *separator = new QLabel("-", this);

  mTimeLabel->setObjectName("timeLabel");
  separator->setObjectName("separator");
  mSpeedLabel->setObjectName("speedLabel");

  // Intuitively, `mTimeLabel` and `mSpeedLabel` could be inserted directly into the `QHBoxLayout`.
  // Doing so, the extension menu of the parent tool bar blinks each time `QLabel::setText()` is called on these objects.
  // This is certainly due to the fact that the QToolBar is monitoring somehow modifications on its children and recreates
  // entirely the extension menu in this case. The present workaround adds an dummy QScrollArea (transparent and without scroll
  // bars) inbetween.
  QScrollArea *timeContainer = new QScrollArea(this);
  timeContainer->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  timeContainer->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  timeContainer->setObjectName("timeLabelContainer");
  timeContainer->setContentsMargins(0, 0, 0, 0);
  timeContainer->setWidget(mTimeLabel);
  QScrollArea *speedContainer = new QScrollArea(this);
  speedContainer->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  speedContainer->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  speedContainer->setObjectName("speedLabelContainer");
  speedContainer->setContentsMargins(0, 0, 0, 0);
  speedContainer->setWidget(mSpeedLabel);

  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->addWidget(timeContainer);
  layout->addStretch();
  layout->addWidget(separator);
  layout->addStretch();
  layout->addWidget(speedContainer);
  layout->setContentsMargins(0, 0, 0, 0);

  connect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbSimulationStateIndicator::update);
  connect(WbApplication::instance(), &WbApplication::postWorldLoaded, this, &WbSimulationStateIndicator::reset);

  mLastSpeedIndicatorTime = WbSimulationState::instance()->time();

  update();

  mSpeedLabel->setText("0.00x");
  mTimeLabel->setText("0:00:00:000");
}

WbSimulationStateIndicator::~WbSimulationStateIndicator() {
}

void WbSimulationStateIndicator::update() {
  // don't refresh the rendering at each step
  if (WbGuiRefreshOracle::instance()->canRefreshNow()) {
    // get current virtual time
    const double time = WbSimulationState::instance()->time();
    QString str;

    // Display "N/A" if we have just stepped and are now
    // paused, or we are currently in step mode itself
    if ((WbSimulationState::instance()->isPaused() &&
         WbSimulationState::instance()->previousMode() == WbSimulationState::STEP) ||
        WbSimulationState::instance()->isStep())
      str = "N/A";
    else if (time < mLastSpeedIndicatorTime)
      str = "0.00x";
    else {
      // update speed indicator
      int elapsed = WbGuiRefreshOracle::instance()->elapsed();
      double speed = (time - mLastSpeedIndicatorTime) / elapsed;

      // If the speed < 10, we want two decimal places i.e 0.00 - 9.99
      // Following this, remove a decimal place as we climb an order of magnitude
      // This gives us room up to 9999x without resizing the container.
      int accuracy;
      if (speed < 10.0)
        accuracy = 2;
      else if (speed < 100.0)
        accuracy = 1;
      else
        accuracy = 0;

      str = QString::asprintf("%.*fx", accuracy, speed);
    }

    if (mSpeedLabel->text() != str)
      mSpeedLabel->setText(str);

    // update virtual time display
    int h = floor(time / 3600000.0);            // hours
    int m = fmod(floor(time / 60000.0), 60.0);  // minutes
    int s = fmod(floor(time / 1000.0), 60.0);   // seconds
    int ms = fmod(time, 1000.0);                // milliseconds
    str = QString::asprintf("%d:%02d:%02d:%03d", h, m, s, ms);

    if (mTimeLabel->text() != str)
      mTimeLabel->setText(str);

    mLastSpeedIndicatorTime = time;
  }
}

void WbSimulationStateIndicator::reset() {
  mTimeLabel->setText("0:00:00:000");
  mSpeedLabel->setText("0.00x");
  mLastSpeedIndicatorTime = 0.0;
}
