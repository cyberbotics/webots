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

ifeq ($(OSTYPE),darwin)
INSTALL_NAME = @rpath/Contents/lib/webots/libode.dylib
endif

ifeq ($(MAKECMDGOALS),)
default:
	@echo "Type one of the following:"
	@echo "  make debug      for debug (with gdb symbols)"
	@echo "  make profile    for profiling (with gprof information)"
	@echo "  make release    for release"
	@echo "  make clean      remove build directory"
endif

ifeq ($(MAKECMDGOALS),profile)
  STATIC_LIBRARY = true
else
  TARGET_LIB_DIR=$(WEBOTS_LIB_PATH)
endif

FILES_TO_REMOVE = $(MAIN_TARGET_COPY) libode.a ode.a

CFLAGS = -DdTRIMESH_ENABLED -DdTRIMESH_OPCODE -DODE_MT -DODE_LIB -DODE_THREADMODE_PTHREAD
ifneq ($(OSTYPE),darwin)
  CFLAGS += -Wno-misleading-indentation -Wno-nonnull-compare
endif

ifeq ($(MAKECMDGOALS),release)
  CFLAGS += -DdNODEBUG
endif

CXXFLAGS = -std=c++11
INCLUDE = -I../../include/ode -I../../include/ode/ode/fluid_dynamics -Iode/src -Iode/src/fluid_dynamics -Ilibccd/src -Ilibccd/src/custom -IOPCODE

# define the source files
C_SOURCES = $(wildcard libccd/src/*.c) $(wildcard ode/src/*.c)
CXX_SOURCES = $(wildcard ode/src/*.cpp) $(wildcard ode/src/joints/*.cpp) $(wildcard ode/src/ode_MT/*.cpp) $(wildcard ode/src/ode_MT/*/*.cpp) $(wildcard OPCODE/*.cpp) $(wildcard OPCODE/Ice/*.cpp) $(wildcard ode/src/fluid_dynamics/*.cpp) $(wildcard ode/src/fluid_dynamics/immersion_link/*.cpp)

# finally, include the Makefile.include to perform magic
EXCLUDE_CONTROLLERS = true

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
