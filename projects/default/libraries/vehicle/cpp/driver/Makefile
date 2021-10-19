# Copyright 1996-2021 Cyberbotics Ltd.
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

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

WEBOTS_LIBRARY = 1

CXX_SOURCES = $(wildcard src/*.cpp)
INCLUDE = -I"$(WEBOTS_HOME)/include/controller/c" -I$(WEBOTS_HOME_PATH)/include/controller/cpp
LIBRARIES = -ldriver -lCppController
MAIN_TARGET = $(LIB_PREFIX)CppDriver$(SHARED_LIB_EXTENSION)


ifeq ($(OSTYPE),linux)
LINK_DEPENDENCIES = $(WEBOTS_CONTROLLER_LIB_PATH)/libdriver.so
LFLAGS = -shared -Wl,-rpath,'$$ORIGIN'
endif

ifeq ($(OSTYPE),darwin)
LINK_DEPENDENCIES = $(WEBOTS_CONTROLLER_LIB_PATH)/libdriver.dylib
LFLAGS = -dynamiclib -compatibility_version 1.0 -current_version 1.0.0 -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
endif

ifeq ($(OSTYPE),windows)
LINK_DEPENDENCIES = $(WEBOTS_CONTROLLER_LIB_PATH)/driver.dll
LFLAGS = -shared -Wl,--enable-auto-import
endif

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
