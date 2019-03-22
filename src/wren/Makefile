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

WEBOTS_HOME ?= ../..

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

WEBOTS_DEPENDENCY_PATH ?= $(WEBOTS_HOME_PATH)/dependencies

STATIC_LIBRARY = true
CFLAGS = -std=c++11 -DGLM_ENABLE_EXPERIMENTAL
INCLUDE = -I$(WEBOTS_HOME_PATH)/include -I$(WEBOTS_HOME_PATH)/include/glad -I../glm -I../webots/external/siphash/

ifeq ($(OSTYPE),darwin)
INCLUDE += -I$(WEBOTS_DEPENDENCY_PATH)/freetype-2.9/include -I$(WEBOTS_DEPENDENCY_PATH)/assimp/include -I$(WEBOTS_DEPENDENCY_PATH)/assimp/build/include
else ifeq ($(OSTYPE),linux)
INCLUDE += -I/usr/include/freetype2
else ifeq ($(OSTYPE),windows)
INCLUDE += -I/mingw64/include/freetype2 -I/mingw64/include
endif

CXX_SOURCES = $(wildcard *.cpp) $(wildcard mesh/*.cpp)

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
