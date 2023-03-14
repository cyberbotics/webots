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

###############################################################
#
# Purpose: Makefile for building a wrapper between the Webots
#          API and the robotis-op2 real robot framework
#
###############################################################

# -------------------------------------------------------------
# TO MODIFY:

# paths
ROBOTISOP2_ROOT ?= /robotis

# -------------------------------------------------------------

TARGET = wrapper.a
CXX_SOURCES = \
  ../src/Robot.cpp \
  ../src/Motion.cpp \
  ../src/Motor.cpp \
  ../src/PositionSensor.cpp \
  ../src/LED.cpp \
  ../src/Accelerometer.cpp \
  ../src/Gyro.cpp \
  ../src/Camera.cpp \
  ../src/Keyboard.cpp \
  ../src/Speaker.cpp
OBJECTS = $(CXX_SOURCES:.cpp=.o)
INCLUDE_DIRS = -I$(ROBOTISOP2_ROOT)/Linux/include -I$(ROBOTISOP2_ROOT)/Framework/include -I../include -I../keyboard

AR = ar
ARFLAGS = cr
CXX = g++
CXXFLAGS += -c -O2 -DLINUX -Wall $(INCLUDE_DIRS)
LIBS += ../keyboard/keyboardInterface.a
LINK_DEPENDENCIES = ../keyboard/keyboardInterface.a
ROBOTISOP2_STATIC_LIBRARY = $(ROBOTISOP2_ROOT)/Linux/lib/darwin.a

all: $(TARGET)

clean:
	rm -f $(TARGET) $(OBJECTS)

$(ROBOTISOP2_STATIC_LIBRARY):
	make -C $(ROBOTISOP2_ROOT)/Linux/build

$(TARGET): $(ROBOTISOP2_STATIC_LIBRARY) $(OBJECTS)
	$(AR) $(ARFLAGS) $(TARGET) $(OBJECTS) $(ROBOTISOP2_STATIC_LIBRARY) $(LIBS) $(LINK_DEPENDENCIES)
	chmod 755 $(TARGET)
