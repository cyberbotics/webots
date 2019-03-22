# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include


CXX_SOURCES = \
  core/MainApplication.cpp \
  core/StandardPaths.cpp \
  devices/Device.cpp \
  devices/Motor.cpp \
  graph2d/Graph2D.cpp \
  gui/GenericWindow.cpp \
  gui/MainWindow.cpp \
  motion_editor/AddStateDialog.cpp \
  motion_editor/ListWidget.cpp \
  motion_editor/Motion.cpp \
  motion_editor/MotionEditor.cpp \
  motion_editor/MotionGlobalSettings.cpp \
  motion_editor/MotionPlayer.cpp \
  motion_editor/MotionWidget.cpp \
  motion_editor/Pose.cpp \
  motion_editor/PoseEditor.cpp \
  motion_editor/PoseWidget.cpp \
  motion_editor/MotorTargetState.cpp \
  motion_editor/MotorTargetStateWidget.cpp \
  widgets/AccelerometerWidget.cpp \
  widgets/CameraWidget.cpp \
  widgets/CategoryWidget.cpp \
  widgets/CompassWidget.cpp \
  widgets/DeviceWidget.cpp \
  widgets/DeviceWidgetFactory.cpp \
  widgets/DifferentialWheelsWidget.cpp \
  widgets/DistanceSensorWidget.cpp \
  widgets/GPSWidget.cpp \
  widgets/GyroWidget.cpp \
  widgets/InertialUnitWidget.cpp \
  widgets/LidarWidget.cpp \
  widgets/LightSensorWidget.cpp \
  widgets/MotorWidget.cpp \
  widgets/PositionSensorWidget.cpp \
  widgets/RadarWidget.cpp \
  widgets/RangeFinderWidget.cpp \
  widgets/ScalarSensorWidget.cpp \
  widgets/SensorWidget.cpp \
  widgets/TouchSensorScalarWidget.cpp \
  widgets/TouchSensorVectorialWidget.cpp \
  widgets/VectorialSensorWidget.cpp

HPP_FILES_TO_MOC = \
  core/MainApplication.hpp \
  gui/GenericWindow.hpp \
  motion_editor/AddStateDialog.hpp \
  motion_editor/ListWidget.hpp \
  motion_editor/Motion.hpp \
  motion_editor/MotionEditor.hpp \
  motion_editor/MotionPlayer.hpp \
  motion_editor/MotionWidget.hpp \
  motion_editor/Pose.hpp \
  motion_editor/PoseEditor.hpp \
  motion_editor/PoseWidget.hpp \
  motion_editor/MotorTargetState.hpp \
  motion_editor/MotorTargetStateWidget.hpp \
  widgets/AccelerometerWidget.hpp \
  widgets/CameraWidget.hpp \
  widgets/CompassWidget.hpp \
  widgets/DifferentialWheelsWidget.hpp \
  widgets/DistanceSensorWidget.hpp \
  widgets/GPSWidget.hpp \
  widgets/GyroWidget.hpp \
  widgets/InertialUnitWidget.hpp \
  widgets/LidarWidget.hpp \
  widgets/LightSensorWidget.hpp \
  widgets/MotorWidget.hpp \
  widgets/PositionSensorWidget.hpp \
  widgets/RadarWidget.hpp \
  widgets/RangeFinderWidget.hpp \
  widgets/SensorWidget.hpp \
  widgets/TouchSensorScalarWidget.hpp \
  widgets/TouchSensorVectorialWidget.hpp \
  widgets/VectorialSensorWidget.hpp

QT = core gui widgets
USE_C_API = true

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
