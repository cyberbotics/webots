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
C_SOURCES = $(wildcard src/*.c)

LIBRARIES = -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lController
MAIN_TARGET = $(LIB_PREFIX)car$(SHARED_LIB_EXTENSION)

INCLUDE = -I$(WEBOTS_HOME_PATH)/include/controller/c

ifeq ($(OSTYPE),linux)
LFLAGS = -shared -Wl,-soname, $(MAIN_TARGET)
LINK_DEPENDENCIES = $(WEBOTS_CONTROLLER_LIB_PATH)/libController.so
endif

ifeq ($(OSTYPE),darwin)
LFLAGS = -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -dynamiclib -install_name @rpath/lib/controller/libcar.dylib -compatibility_version 1.0 -current_version 1.0.0
LINK_DEPENDENCIES = $(WEBOTS_CONTROLLER_LIB_PATH)/libController.dylib
endif

ifeq ($(OSTYPE),windows)
LFLAGS = -shared -mwindows --def Controller.def
LINK_DEPENDENCIES = $(WEBOTS_CONTROLLER_LIB_PATH)/Controller.dll
endif


include $(WEBOTS_HOME_PATH)/resources/Makefile.include
