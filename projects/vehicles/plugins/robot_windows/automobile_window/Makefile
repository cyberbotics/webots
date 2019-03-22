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

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

CXX_SOURCES = entry_points.cpp Viewer.cpp AbstractWidget.cpp GeneralInformationWidget.cpp SpeedWidget.cpp RPMWidget.cpp SteeringWidget.cpp EncodersWidget.cpp ThrottleWidget.cpp BrakeWidget.cpp OSMImportWidget.cpp
HPP_FILES_TO_MOC = GeneralInformationWidget.hpp AbstractWidget.hpp OSMImportWidget.hpp

QT = core gui widgets
USE_C_API = true

QT_UTILS = $(WEBOTS_HOME)/resources/projects/libraries/qt_utils
INCLUDE = -I"$(QT_UTILS)"
LIBRARIES = -L"$(QT_UTILS)" -lqt_utils -ldriver -lcar

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
