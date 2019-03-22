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

# Makefile to compile the ODE_MT project
# Written by Abd-ur-Rehman Mustafa (Cyberbotics)
# 18/02/2014

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

ifeq ($(MAKECMDGOALS),)
default:
	@echo "Type one of the following:"
	@echo "  make debug                 for debug (with gdb symbols)"
	@echo "  make profile               for profiling (with gprof information)"
	@echo "  make release               for release"
	@echo "  make clean                 remove build directory"
else

CFLAGS = -DdTRIMESH_ENABLED -DdTRIMESH_OPCODE -DODE_MT -DODE_LIB -DODE_THREADMODE_PTHREAD
ifneq ($(OSTYPE),darwin)
  CFLAGS += -Wno-misleading-indentation -Wno-nonnull-compare
endif
INCLUDE = -I../../include/ode -I../../include/ode/ode/fluid_dynamics -Iode/src -Iode/src/fluid_dynamics -Ilibccd/src -IOPCODE
LIBRARIES = -lpthread

# define the source files

C_SOURCES = $(wildcard libccd/src/*.c) $(wildcard ode/src/*.c)

CXX_SOURCES = $(wildcard ode/src/*.cpp) $(wildcard ode/src/joints/*.cpp) $(wildcard ode/src/ode_MT/*.cpp) $(wildcard ode/src/ode_MT/*/*.cpp) $(wildcard OPCODE/*.cpp) $(wildcard OPCODE/Ice/*.cpp) $(wildcard ode/src/fluid_dynamics/*.cpp) $(wildcard ode/src/fluid_dynamics/immersion_link/*.cpp)

ifeq ($(OSTYPE),windows)
	FILES_TO_REMOVE = ../../msys64/mingw64/bin/ode.dll
endif
ifeq ($(OSTYPE),linux)
	FILES_TO_REMOVE = ../../lib/libode.so
endif
ifeq ($(OSTYPE),darwin)
	FILES_TO_REMOVE = ../../lib/libode.3.dylib ../../lib/libode.dylib
endif

ifeq ($(OSTYPE),windows)
TARGET_ODE_LIBRARY_PATH=../../msys64/mingw64/bin
TARGET_ODE_LIBRARY_FILES=$(TARGET_ODE_LIBRARY_PATH)/ode.dll
endif
ifeq ($(OSTYPE),linux)
TARGET_ODE_LIBRARY_PATH=../../lib
TARGET_ODE_LIBRARY_FILES=$(TARGET_ODE_LIBRARY_PATH)/libode.so $(TARGET_ODE_LIBRARY_PATH)/libode.so.3
endif
ifeq ($(OSTYPE),darwin)
TARGET_ODE_LIBRARY_PATH=../../lib
TARGET_ODE_LIBRARY_FILES=$(TARGET_ODE_LIBRARY_PATH)/libode.dylib $(TARGET_ODE_LIBRARY_PATH)/libode.3.dylib
endif
FILES_TO_REMOVE += $(TARGET_ODE_LIBRARY_FILES)
WEBOTS_DISABLE_BINARY_COPY=true

USER_POSTBUILD: $(TARGET_ODE_LIBRARY_FILES)

$(TARGET_ODE_LIBRARY_PATH)/ode.dll: $(TARGET)
	@echo "#" copying $(@F)
	@mv $(TARGET) $@

$(TARGET_ODE_LIBRARY_PATH)/libode.so: $(TARGET)
	@echo "#" copying $(@F)
	@mv $(TARGET) $@

$(TARGET_ODE_LIBRARY_PATH)/libode.so.3:
	@echo "#" creating $(@F)
	@ln -s libode.so $@

$(TARGET_ODE_LIBRARY_PATH)/libode.3.dylib: $(TARGET)
	@echo "#" copying $(@F)
	@install_name_tool -id @rpath/lib/libode.dylib $(TARGET)
	@cp $(TARGET) $@

$(TARGET_ODE_LIBRARY_PATH)/libode.dylib:
	@echo "#" creating $(@F)
	@ln -s libode.3.dylib $@

# finally, include the Makefile.include to perform magic

EXCLUDE_CONTROLLERS = true
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
endif
