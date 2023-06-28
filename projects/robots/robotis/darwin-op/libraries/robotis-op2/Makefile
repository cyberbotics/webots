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

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

ROBOTISOP2_FRAMEWORK_PATH = ./robotis/Framework

CXX_SOURCES = \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/math/Matrix.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/math/Vector.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/math/Point.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/MX28.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/motion/JointData.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/motion/MotionStatus.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/motion/Kinematics.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/motion/modules/Action.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/motion/modules/Walking.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/vision/ImgProcess.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/vision/ColorFinder.cpp \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/vision/Image.cpp
C_SOURCES = \
  $(ROBOTISOP2_FRAMEWORK_PATH)/src/minIni/minIni.c

INCLUDE = -I"$(ROBOTISOP2_FRAMEWORK_PATH)/include"
CFLAGS = -DWEBOTS

ifeq ($(OSTYPE),darwin)
CFLAGS += -Wno-unused-private-field
else
CFLAGS += -Wno-stringop-truncation
endif

USE_C_API = True

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
