// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef WB_SIMULATION_STATE_INDICATOR_HPP
#define WB_SIMULATION_STATE_INDICATOR_HPP

//
// Description: elapsed time, speedometer, and recording (REC) indicator
//

#include <QtWidgets/QFrame>

class QLabel;

class WbSimulationStateIndicator : public QFrame {
  Q_OBJECT

public:
  explicit WbSimulationStateIndicator(QWidget *parent = NULL);
  virtual ~WbSimulationStateIndicator();

private:
  QLabel *mTimeLabel, *mSpeedLabel;
  double mLastSpeedIndicatorTime;

private slots:
  void update();
  void reset();
};

#endif
