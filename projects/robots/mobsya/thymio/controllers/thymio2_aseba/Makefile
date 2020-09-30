# Copyright 1996-2020 Cyberbotics Ltd.
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
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

DASHEL_HOME = $(WEBOTS_HOME)/projects/robots/mobsya/thymio/libraries/dashel/

C_SOURCES = \
  thymio2_definitions.c \
  thymio2_natives.c \
  aseba/transport/buffer/vm-buffer.c \
  aseba/vm/natives.c \
  aseba/vm/vm.c
CXX_SOURCES = $(wildcard *.cpp)

ifneq ($(OSTYPE),darwin)
CFLAGS = -fno-strict-aliasing
endif

ifeq ($(OSTYPE),windows)
CFLAGS = -Wno-address -Wno-unused-function -DDISABLE_WEAK_CALLBACKS
endif

INCLUDE = -I"$(DASHEL_HOME)" -Iaseba
LIBRARIES = -L"$(DASHEL_HOME)" -ldashel

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
