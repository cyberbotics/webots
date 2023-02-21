# Copyright 1996-2018 Cyberbotics Ltd.
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
# Purpose: Makefile for "DARwIn Linux Framework"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

CC = g++
AR = ar
ARFLAGS = cr

TARGET = ../lib/darwin.a

INCLUDE_DIRS = -I../include -I../../Framework/include

CXXFLAGS += -O2 -DLINUX -Wall -shared $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall -shared $(INCLUDE_DIRS)
LFLAGS += -lpthread -ldl

OBJS =  ../../Framework/src/MX28.o     	\
        ../../Framework/src/CM730.o     	\
        ../../Framework/src/math/Matrix.o   \
        ../../Framework/src/math/Plane.o    \
        ../../Framework/src/math/Point.o    \
        ../../Framework/src/math/Vector.o   \
        ../../Framework/src/motion/JointData.o  	\
        ../../Framework/src/motion/Kinematics.o 	\
        ../../Framework/src/motion/MotionManager.o  \
        ../../Framework/src/motion/MotionStatus.o   \
        ../../Framework/src/motion/modules/Action.o \
        ../../Framework/src/motion/modules/Head.o   \
        ../../Framework/src/motion/modules/Walking.o\
        ../../Framework/src/vision/BallFollower.o   \
        ../../Framework/src/vision/BallTracker.o    \
        ../../Framework/src/vision/ColorFinder.o    \
        ../../Framework/src/vision/Image.o  		\
        ../../Framework/src/vision/ImgProcess.o 	\
        ../../Framework/src/vision/Camera.o	\
        ../../Framework/src/minIni/minIni.o	\
        streamer/httpd.o           \
        streamer/jpeg_utils.o      \
        streamer/mjpg_streamer.o   \
        LinuxActionScript.o   \
        LinuxCamera.o   \
        LinuxCM730.o    \
        LinuxMotionTimer.o    \
        LinuxNetwork.o

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) $(TARGET) $(OBJS)
#	$(CC) -o $(TARGET) $(OBJS) $(LFLAGS)

all: $(TARGET)

clean:
	rm -f $(OBJS) ../lib/$(TARGET)
