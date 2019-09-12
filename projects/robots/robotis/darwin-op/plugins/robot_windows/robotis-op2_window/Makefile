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

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

CXX_SOURCES = entry_points.cpp Viewer.cpp TransferWidget.cpp SSH.cpp ZIP.cpp
HPP_FILES_TO_MOC = TransferWidget.hpp SSH.hpp

QT = core gui widgets concurrent
USE_C_API = true

QT_UTILS = $(WEBOTS_HOME)/resources/projects/libraries/qt_utils
LIBRARIES = -L"$(QT_UTILS)" -lqt_utils
INCLUDE = -I"$(QT_UTILS)"

ifeq ($(OSTYPE),windows)
LIBRARIES += -L"$(WEBOTS_LIB_PATH)" -lssh -lpthread -lzip
INCLUDE += -I/mingw64/include -I/mingw64/lib/libzip/include
endif

ifeq ($(OSTYPE),darwin)
LIBRARIES += -L"$(WEBOTS_HOME)/projects/robots/robotis/darwin-op/libraries/libzip" -lnihzip -L"$(WEBOTS_HOME)/projects/robots/robotis/darwin-op/libraries/libssh" -lssh
INCLUDE += -I"$(WEBOTS_HOME)/projects/robots/robotis/darwin-op/libraries/libzip/include/libzip" -I"$(WEBOTS_HOME)/projects/robots/robotis/darwin-op/libraries/libssh/include/libssh"
endif

ifeq ($(OSTYPE),linux)
LIBRARIES += -lssh -lzip
endif

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
