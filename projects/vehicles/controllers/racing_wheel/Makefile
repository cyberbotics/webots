# Copyright 1996-2022 Cyberbotics Ltd.
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

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

LIBRARIES = -ldriver -lcar -lCppDriver

ifeq ($(OSTYPE),linux)
CFLAGS = -fPIC
LIBRARIES += -Wl,-rpath,$(WEBOTS_LIB_PATH)
endif

CXX_SOURCES = $(wildcard *.cpp)

# Do not modify the following: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Qt linking

CFLAGS += -std=c++17

ifeq ($(OSTYPE),windows)
 CFLAGS += -D_USE_MATH_DEFINES
 QT_INCLUDE_DIR = $(WEBOTS_HOME)/include/qt
 DYNAMIC_LIBRARIES += -L"$(WEBOTS_HOME)/msys64/mingw64/bin" -lQt6Core
 INCLUDE += -isystem "$(QT_INCLUDE_DIR)/QtCore"
endif

ifeq ($(OSTYPE),linux)
 QT_INCLUDE_DIR = $(WEBOTS_HOME)/include/qt
 QT_LIB_DIR = $(WEBOTS_HOME)/lib/webots
 DYNAMIC_LIBRARIES += -L"$(QT_LIB_DIR)" -lQt6Core
 INCLUDE += -isystem "$(QT_INCLUDE_DIR)/QtCore"
endif

ifeq ($(OSTYPE),darwin)
 WBCFLAGS += -Wno-unknown-pragmas
 FRAMEWORKS_DIR = $(WEBOTS_HOME)/Contents/Frameworks
 INCLUDE += -F"$(FRAMEWORKS_DIR)"
 FRAMEWORKS += -F"$(FRAMEWORKS_DIR)"
 DYNAMIC_LIBRARIES += -bind_at_load $(FRAMEWORKS)
 FRAMEWORKS += -framework QtCore
endif
