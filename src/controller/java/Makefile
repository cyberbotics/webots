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

ifeq ($(WEBOTS_HOME),)
WEBOTS_HOME_PATH = ../../..
else
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
endif

default:
	@echo "Type one of the following (OSTYPE is \""$(OSTYPE)"\"):"
	@echo "  make debug                 for debug (with gdb symbols)"
	@echo "  make profile               for profiling (with gprof information)"
	@echo "  make release               for finale release"
	@echo "  make clean                 remove build directory"

include $(WEBOTS_HOME_PATH)/resources/Makefile.include

.DEFAULT_GOAL := release

#directories
SWIG            = swig
SWIG_OUTPUT_PATH= SWIG_generated_files
CONTROLLERPATH  = com/cyberbotics/webots/controller
OBJDIR          = build/$(MAKECMDGOALS)

#files
INTERFACE       = controller.i
WRAPPER         = $(addprefix $(SWIG_OUTPUT_PATH)/,$(INTERFACE:.i=_wrap.cpp))
WRAPPER_OBJECT  = $(addprefix $(OBJDIR)/,$(INTERFACE:.i=_wrap.o))
JAR_FILE        = $(WEBOTS_CONTROLLER_LIB_PATH)/java/Controller.jar
JAVA_FILES      = Accelerometer.java \
                  Altimeter.java \
                  AnsiCodes.java \
                  Brake.java \
                  Camera.java \
                  Compass.java \
                  Connector.java \
                  Device.java \
                  Display.java \
                  DistanceSensor.java \
                  Emitter.java \
                  Field.java \
                  GPS.java \
                  Gyro.java \
                  InertialUnit.java \
                  ImageRef.java \
                  Joystick.java \
                  Keyboard.java \
                  LED.java \
                  Lidar.java \
                  LidarPoint.java \
                  LightSensor.java \
                  Motion.java \
                  Motor.java \
                  Node.java \
                  Pen.java \
                  PositionSensor.java \
                  Propeller.java \
                  Radar.java \
                  RadarTarget.java \
                  RangeFinder.java \
                  Receiver.java \
                  Robot.java \
                  Skin.java \
                  Speaker.java \
                  Supervisor.java \
                  TouchSensor.java \
                  VacuumGripper.java \
                  wrapper.java \
                  wrapperJNI.java
LIBNAME         = JavaController

#compilation options
WEBOTS_INCLUDES = -I$(WEBOTS_HOME_PATH)/include/controller/cpp -I$(WEBOTS_HOME_PATH)/include/controller/c

SWIG_EXISTS = $(shell which $(SWIG) 2> /dev/null)

ifeq ($(CC),cc)
CC=gcc
endif

# variables depending on the OS
ifeq ($(OSTYPE),windows)
CFLAGS1         = -I$(WEBOTS_HOME_PATH)/windows/include
PLATFORM_INCLUDE= -I"$(JAVA_HOME)/include/win32"
PLATFORM_LIBS   =
TARGET          = $(WEBOTS_CONTROLLER_LIB_PATH)/java/$(LIBNAME).dll
CFLAGS2         = -c -Wall -Wno-unused-function -O
JNILIB_FLAGS    = -bundle
JAVA_INCLUDES   = -I"$(JAVA_HOME)/include" -I"$(JAVA_HOME)/include/win32"
LIBCONTROLLER   = $(WEBOTS_CONTROLLER_LIB_PATH)/Controller.dll
LIBCPPCONTROLLER= $(WEBOTS_CONTROLLER_LIB_PATH)/CppController.dll
else

ifeq ($(OSTYPE),darwin)
WRAPPER_OBJECT_X86_64 = $(addprefix $(OBJDIR)/x86_64/,$(INTERFACE:.i=.o))
WRAPPER_OBJECT_ARM64  = $(addprefix $(OBJDIR)/arm64/,$(INTERFACE:.i=.o))
CFLAGS1               = -isysroot $(MACOSX_SDK_PATH) -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
PLATFORM_INCLUDE      = -I"$(MACOSX_SDK_PATH)/System/Library/Frameworks/JavaVM.framework/Versions/Current/Headers"
TARGET                = $(WEBOTS_CONTROLLER_LIB_PATH)/java/lib$(LIBNAME).jnilib
CFLAGS2               = -c -Wall -isysroot $(MACOSX_SDK_PATH) -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -Wno-unused-function
JNILIB_FLAGS          = -dynamiclib -install_name @rpath/Contents/lib/controller/java/libJavaController.jnilib -Wl,-rpath,@loader_path/../../../..
JAVA_INCLUDES         = -I"$(JAVA_HOME)/include" -I"$(JAVA_HOME)/include/darwin"
LIBCONTROLLER         = $(WEBOTS_CONTROLLER_LIB_PATH)/libController.dylib
LIBCPPCONTROLLER      = $(WEBOTS_CONTROLLER_LIB_PATH)/libCppController.dylib

else #Linux
CFLAGS1         = -fPIC
PLATFORM_INCLUDE= -I"$(JAVA_HOME)/include/linux" -I"/usr/X11R6/include"
PLATFORM_LIBS   =
TARGET          = $(WEBOTS_CONTROLLER_LIB_PATH)/java/lib$(LIBNAME).so
CFLAGS2         = -c -Wall -Wno-unused-function -fPIC
JNILIB_FLAGS    = -bundle
JAVA_INCLUDES   = -I"$(JAVA_HOME)/include" -I"$(JAVA_HOME)/include/linux"
LIBCONTROLLER   = $(WEBOTS_CONTROLLER_LIB_PATH)/libController.so
LIBCPPCONTROLLER= $(WEBOTS_CONTROLLER_LIB_PATH)/libCppController.so
endif
endif

# rules
ifeq ($(SWIG_EXISTS),)
release debug profile:
	@$(ECHO) "# \033[0;33mSWIG not installed, skipping Java API\033[0m"
else ifeq ($(JAVA_HOME),)
release debug profile:
	@$(ECHO) "# \033[0;33mJava not installed or 'JAVA_HOME' not set, skipping Java API\033[0m"
else
release debug profile: $(JAR_FILE) $(TARGET)

$(WRAPPER_OBJECT): $(WRAPPER)
	@echo "# compiling" $(notdir $^)
	$(CXX) $(CFLAGS2) $(WEBOTS_INCLUDES) $(JAVA_INCLUDES) $< -o $@

$(WRAPPER_OBJECT_ARM64): $(WRAPPER)
	@echo "# compiling" $(notdir $^) "(arm64)"
	$(CXX) -target arm64-apple-macos11 $(CFLAGS2) $(WEBOTS_INCLUDES) $(JAVA_INCLUDES) $< -o $@

$(WRAPPER_OBJECT_X86_64): $(WRAPPER)
	@echo "# compiling" $(notdir $^) "(x86_64)"
	$(CXX) -target x86_64-apple-macos11 $(CFLAGS2) $(WEBOTS_INCLUDES) $(JAVA_INCLUDES) $< -o $@

$(OBJDIR)/arm64/lib$(LIBNAME).jnilib: $(WRAPPER_OBJECT_ARM64)
	@echo "# linking" $(notdir $^) "(arm64)"
	$(CXX) -target arm64-apple-macos11 $(CFLAGS1) $(JNILIB_FLAGS) -L$(WEBOTS_CONTROLLER_LIB_PATH) -lController -lCppController $^ -o "$@"

$(OBJDIR)/x86_64/lib$(LIBNAME).jnilib: $(WRAPPER_OBJECT_X86_64)
	@echo "# linking" $(notdir $^) "(x86_64)"
	$(CXX) -target x86_64-apple-macos11 $(CFLAGS1) $(JNILIB_FLAGS) -L$(WEBOTS_CONTROLLER_LIB_PATH) -lController -lCppController $^ -o "$@"

$(WEBOTS_CONTROLLER_LIB_PATH)/java/lib$(LIBNAME).jnilib: $(OBJDIR)/arm64/lib$(LIBNAME).jnilib $(OBJDIR)/x86_64/lib$(LIBNAME).jnilib
	@echo "# creating fat" $^
	@mkdir -p $(WEBOTS_CONTROLLER_LIB_PATH)/java
	lipo -create -output "$@" $(OBJDIR)/x86_64/lib$(LIBNAME).jnilib $(OBJDIR)/arm64/lib$(LIBNAME).jnilib
	codesign -s - "$@"

$(WEBOTS_CONTROLLER_LIB_PATH)/java/lib$(LIBNAME).so: $(WRAPPER_OBJECT)
	@echo "# compiling" $^
	@mkdir -p $(WEBOTS_CONTROLLER_LIB_PATH)/java
	$(CXX) -shared $^ -L$(WEBOTS_CONTROLLER_LIB_PATH) -lController -lCppController -o $@
	@chmod a-x $@

$(WEBOTS_CONTROLLER_LIB_PATH)/java/$(LIBNAME).dll: $(WRAPPER_OBJECT)
	@echo "# linking" $@
	@mkdir -p $(WEBOTS_CONTROLLER_LIB_PATH)/java
	$(CXX) -shared -mwindows -Wl,--add-stdcall-alias -Wl,--enable-auto-import -O \
	$^ -L$(WEBOTS_CONTROLLER_LIB_PATH) -lController -lCppController -lm -o "$@"

$(JAR_FILE): $(WRAPPER)
	@echo "# compiling java sources"
	javac --release 8 $(SWIG_OUTPUT_PATH)/*.java -d .
	@echo "# creating   "$@
	@mkdir -p $(WEBOTS_CONTROLLER_LIB_PATH)/java
	jar cfv $(JAR_FILE) $(CONTROLLERPATH)/*.class > /dev/null

$(WRAPPER): $(INTERFACE) $(LIBCONTROLLER) $(LIBCPPCONTROLLER)
	@echo "# generating "$(notdir $@)
	test -d $(SWIG_OUTPUT_PATH) || mkdir -p $(SWIG_OUTPUT_PATH)
	$(SWIG) -c++ -java -package $(subst /,.,$(CONTROLLERPATH)) -outdir $(SWIG_OUTPUT_PATH) $(WEBOTS_INCLUDES) -o "$@" $<

$(LIBCONTROLLER):
	@echo "$(LIBCONTROLLER) doesn't exist, build it first."

$(LIBCPPCONTROLLER):
	@echo "$(LIBCPPCONTROLLER) doesn't exist, build it first."
endif

clean:
ifneq ($(SWIG_EXISTS),) # otherwise a user may break the distribution
FILES_TO_REMOVE = $(SWIG_OUTPUT_PATH)
endif
FILES_TO_REMOVE += $(TARGET) $(JAR_FILE) *.o com
EMPTY_DIRECTORIES_TO_REMOVE = $(WEBOTS_CONTROLLER_LIB_PATH)/java
