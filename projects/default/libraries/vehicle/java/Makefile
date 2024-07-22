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
WEBOTS_HOME_PATH_NO_ESCAPE=$(subst \,/,$(WEBOTS_HOME))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

WEBOTS_LIBRARY = 1
MODULE_NAME = vehicle

INTERFACE = $(MODULE_NAME).i
SOURCE_HEADERS = $(substr $(WEBOTS_HOME_PATH_NO_ESCAPE),$(WEBOTS_HOME_PATH),$(wildcard $(WEBOTS_HOME_PATH)/include/controller/cpp/webots/*.hpp))
JAVA_FILES = Driver.java Car.java vehicle.java vehicleJNI.java

WRAPPER = $(MODULE_NAME).cpp
WRAPPER_OBJECT = $(WRAPPER:.cpp=.o)
JAR = $(WEBOTS_CONTROLLER_LIB_PATH)/java/$(MODULE_NAME).jar
INCLUDES = -I$(WEBOTS_HOME_PATH)/include/controller/c/ -I$(WEBOTS_HOME_PATH)/include/controller/cpp/

SWIG_OUTPUT  = output
MODULE_PATH  = com/cyberbotics/webots/controller/vehicle
JAVA_SOURCES = $(addprefix $(SWIG_OUTPUT)/,$(JAVA_FILES))

SWIG_EXISTS = $(shell which swig 2> /dev/null)

ifeq ($(OSTYPE),linux)
JAVA_INCLUDES = -I"$(JAVA_HOME)/include" -I"$(JAVA_HOME)/include/linux"
CFLAGS2       = -c -Wall -Wno-unused-function -fPIC
LIBRARY       = $(WEBOTS_CONTROLLER_LIB_PATH)/java/libvehicle.so
endif

ifeq ($(OSTYPE),windows)
JAVA_INCLUDES = -I"$(JAVA_HOME)/include" -I"$(JAVA_HOME)/include/win32"
CFLAGS2       = -c -Wall -Wno-unused-function -O
LIBRARY       = $(WEBOTS_CONTROLLER_LIB_PATH)/java/vehicle.dll
endif

ifeq ($(OSTYPE),darwin)
WRAPPER_OBJECT_ARM64 = $(WRAPPER:.cpp=-arm64.o)
WRAPPER_OBJECT_X86_64 = $(WRAPPER:.cpp=-x86_64.o)
JAVA_INCLUDES = -I"$(JAVA_HOME)/include" -I"$(JAVA_HOME)/include/darwin" -I"$(MACOSX_SDK_PATH)/System/Library/Frameworks/JavaVM.framework/Versions/Current/Headers"
CFLAGS1 = -isysroot $(MACOSX_SDK_PATH) -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -stdlib=libc++ -Wl,-rpath,@loader_path/../../..
CFLAGS2 = -c -Wall -isysroot $(MACOSX_SDK_PATH) -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -stdlib=libc++ -Wno-unused-function
LIBRARY = $(WEBOTS_CONTROLLER_LIB_PATH)/java/libvehicle.jnilib
endif

LIB += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lCppController -lCppCar -lCppDriver

.PHONY: JAVA_CLASSES all

# rules
ifeq ($(SWIG_EXISTS),)
all release debug profile:
	@$(ECHO) "# \033[0;33mSWIG not installed, skipping Java API\033[0m"
else ifeq ($(JAVA_HOME),)
all release debug profile:
	@$(ECHO) "# \033[0;33mJava not installed or 'JAVA_HOME' not set, skipping Java API\033[0m"
else
all release debug profile: $(JAR) $(LIBRARY)

$(JAR): JAVA_CLASSES
	@mkdir -p $(WEBOTS_CONTROLLER_LIB_PATH)/java
	@jar cfv "$@" $(MODULE_PATH)/*.class > /dev/null

JAVA_CLASSES: $(JAVA_SOURCES)
	@javac --release 8 -classpath $(WEBOTS_CONTROLLER_LIB_PATH)/java/Controller.jar $(JAVA_SOURCES) -d .

$(JAVA_SOURCES): $(WRAPPER)

$(WRAPPER): $(INTERFACE) $(SOURCE_HEADERS)
	@mkdir -p $(SWIG_OUTPUT)
	@swig -c++ -java -package $(subst /,.,$(MODULE_PATH)) -outdir $(SWIG_OUTPUT) -I$(WEBOTS_HOME_PATH)/src/controller/java/ $(INCLUDES) -o $@ $<

$(WRAPPER_OBJECT): $(WRAPPER)
	@$(CXX) $(CFLAGS2) $(INCLUDES) $(JAVA_INCLUDES) $< -o "$@"

$(WRAPPER_OBJECT_ARM64): $(WRAPPER)
	@$(CXX) -target arm64-apple-macos12 $(CFLAGS2) $(INCLUDES) $(JAVA_INCLUDES) $< -o "$@"

$(WRAPPER_OBJECT_X86_64): $(WRAPPER)
	@$(CXX) -target x86_64-apple-macos12 $(CFLAGS2) $(INCLUDES) $(JAVA_INCLUDES) $< -o "$@"

%.so: $(WRAPPER_OBJECT)
	@mkdir -p $(WEBOTS_CONTROLLER_LIB_PATH)/java
	@$(CXX) -shared $< $(LIB) -o "$@"

%.dll: $(WRAPPER_OBJECT)
	@mkdir -p $(WEBOTS_CONTROLLER_LIB_PATH)/java
	@$(CXX) -shared -mwindows -Wl,--add-stdcall-alias -Wl,--enable-auto-import -O -lm $< $(LIB) -o "$@"

%.jnilib: $(WRAPPER_OBJECT_ARM64) $(WRAPPER_OBJECT_X86_64)
	@$(CXX) -target arm64-apple-macos12 $(CFLAGS1) -bundle $(WRAPPER_OBJECT_ARM64) $(LIB) -o lib$(MODULE_NAME)-arm64.jnilib
	@$(CXX) -target x86_64-apple-macos12 $(CFLAGS1) -bundle $(WRAPPER_OBJECT_X86_64) $(LIB) -o lib$(MODULE_NAME)-x86_64.jnilib
	@mkdir -p $(WEBOTS_CONTROLLER_LIB_PATH)/java
	@lipo -create -output "$@" lib$(MODULE_NAME)-arm64.jnilib lib$(MODULE_NAME)-x86_64.jnilib
	@codesign -s - "$@"
endif

clean:
	@rm -fr com $(SWIG_OUTPUT) $(JAR) $(WRAPPER) $(LIBRARY) *.o *.jnilib
	@rm -d $(WEBOTS_CONTROLLER_LIB_PATH)/java > /dev/null 2>&1 || :
