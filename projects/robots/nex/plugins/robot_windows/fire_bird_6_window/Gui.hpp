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

/*
 * Description:  Class defining the robot window of the Firebird6
 */

#ifndef GUI_HPP
#define GUI_HPP

#include <gui/MainWindow.hpp>

class QGridLayout;
class QWidget;
class RemoteControlWidget;
class MainWidget;
class AccelerometerGroupBox;
class GyroGroupBox;
class MagnetometerGroupBox;
class EncoderGroupBox;

class Gui : public webotsQtUtils::MainWindow {
public:
  Gui();
  virtual ~Gui();
  virtual void updateValues();

private:
  QGridLayout *mLayout;
  MainWidget *mMainWidget;
  RemoteControlWidget *mRemoteControlWidget;
  AccelerometerGroupBox *mAccelerometerGroupBox;
  GyroGroupBox *mGyroGroupBox;
  MagnetometerGroupBox *mMagnetometerGroupBox;
  EncoderGroupBox *mEncoderGroupBox;
};

#endif  // GUI_HPP
