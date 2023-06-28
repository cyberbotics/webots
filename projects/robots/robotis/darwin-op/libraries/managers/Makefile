# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

MANAGERS_SOURCES_PATH = ./src
MANAGERS_INCLUDE_PATH = ./include
ROBOTISOP2_PATH = ../robotis-op2
ROBOTISOP2_FRAMEWORK_PATH = $(ROBOTISOP2_PATH)/robotis/Framework

CXX_SOURCES = \
  $(MANAGERS_SOURCES_PATH)/RobotisOp2DirectoryManager.cpp \
  $(MANAGERS_SOURCES_PATH)/RobotisOp2MotionManager.cpp \
  $(MANAGERS_SOURCES_PATH)/RobotisOp2GaitManager.cpp \
  $(MANAGERS_SOURCES_PATH)/RobotisOp2VisionManager.cpp

INCLUDE = -I"$(ROBOTISOP2_FRAMEWORK_PATH)/include" -I"$(MANAGERS_INCLUDE_PATH)"
CFLAGS = -DWEBOTS
LIBRARIES = -L"$(ROBOTISOP2_PATH)" -lrobotis-op2

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
