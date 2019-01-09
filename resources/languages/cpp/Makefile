# Copyright 1996-2018 Cyberbotics Ltd.
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

ifeq ($(WEBOTS_HOME),)
WEBOTS_HOME_PATH = ../../..
else
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
endif

include $(WEBOTS_HOME_PATH)/resources/Makefile.include

CPPSRC        = Accelerometer.cpp \
                Brake.cpp \
                Camera.cpp \
                Compass.cpp \
                Connector.cpp \
                Device.cpp \
                DifferentialWheels.cpp \
                Display.cpp \
                DistanceSensor.cpp \
                Emitter.cpp \
                Field.cpp \
                GPS.cpp \
                Gyro.cpp \
                InertialUnit.cpp \
                Joystick.cpp \
                Keyboard.cpp \
                LED.cpp \
                Lidar.cpp \
                LightSensor.cpp \
                Motion.cpp \
                Motor.cpp \
                Mouse.cpp \
                Node.cpp \
                Pen.cpp \
                PositionSensor.cpp \
                Radar.cpp \
                RangeFinder.cpp \
                Receiver.cpp \
                Robot.cpp \
                Skin.cpp \
                Speaker.cpp \
                Supervisor.cpp \
                TouchSensor.cpp

CPPOBJS        = $(CPPSRC:.cpp=.o)
DEPENDENCIES   = $(CPPSRC:.cpp=.d)

ifeq ($(OSTYPE),windows)
LDFLAGS       = -shared -Wl,--out-implib,$(WEBOTS_HOME_PATH)/msys64/mingw64/lib/libCppController.a
LIBCONTROLLER = $(WEBOTS_HOME_PATH)/msys64/mingw64/bin/Controller.dll
SHAREDLIBS    = $(LIBCONTROLLER)
CPPFLAGS      = -c -Wall -mwindows
CPPINCLUDES   = -I$(WEBOTS_HOME_PATH)/include/controller/c -I$(WEBOTS_HOME_PATH)/include/controller/cpp
TARGET        = $(WEBOTS_HOME_PATH)/msys64/mingw64/bin/CppController.dll
endif

ifeq ($(OSTYPE),darwin)
LDFLAGS       = -dynamiclib -compatibility_version 1.0 -current_version 1.0.0 -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
SHAREDLIBS    = -L$(WEBOTS_HOME_PATH)/lib -lController
CPPFLAGS      = -c -fPIC -Wall -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
CPPINCLUDES   = -I$(WEBOTS_HOME_PATH)/include/controller/c -I$(WEBOTS_HOME_PATH)/include/controller/cpp
LIBCONTROLLER = $(WEBOTS_HOME_PATH)/lib/libController.dylib
LDFLAGS      += -install_name @rpath/lib/libCppController.dylib
TARGET        = $(WEBOTS_HOME_PATH)/lib/libCppController.dylib
endif

ifeq ($(OSTYPE),linux)
LDFLAGS       = -shared
SHAREDLIBS    = -L$(WEBOTS_HOME_PATH)/lib -lController
CPPFLAGS      = -c -fpic -Wall
CPPINCLUDES   = -I$(WEBOTS_HOME_PATH)/include/controller/c -I$(WEBOTS_HOME_PATH)/include/controller/cpp
LIBCONTROLLER = $(WEBOTS_HOME_PATH)/lib/libController.so
TARGET        = $(WEBOTS_HOME_PATH)/lib/libCppController.so
endif

ifeq ($(TREAT_WARNINGS_AS_ERRORS),1)
CPPFLAGS += -Werror
endif

all debug profile release: $(TARGET)

$(TARGET):$(CPPOBJS) $(LIBCONTROLLER)
	$(CXX) $(LDFLAGS) $(CPPOBJS) $(SHAREDLIBS) -o "$@"

$(LIBCONTROLLER):
	@echo "Warning: $(LIBCONTROLLER) doesn't exist, rebuild it first."

-include $(DEPENDENCIES)

# cpp compilation rule
%.o:%.cpp
	$(CXX) $(CPPFLAGS) $(CPPINCLUDES) -o $@ $<

# dependency rule
%.d:%.cpp
	$(CXX) $(CPPINCLUDES) -MM $< > $@

ifeq ($(OSTYPE),windows)
FILES_TO_REMOVE = $(WEBOTS_HOME_PATH)/msys64/mingw64/bin/CppController.dll $(WEBOTS_HOME_PATH)/msys64/mingw64/lib/libCppController.a *.def *.exp
else
FILES_TO_REMOVE = $(WEBOTS_HOME_PATH)/lib/*Cpp*
endif
FILES_TO_REMOVE += $(CPPOBJS) $(DEPENDENCIES)
