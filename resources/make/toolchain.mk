# Copyright 1996-2025 Cyberbotics Ltd.
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

# Compiler defaults, source discovery, target naming, and build directory setup.

ifeq ($(CC),cc)
 CC = gcc
endif

ifndef CURDIR
 CURDIR = $(shell pwd)
endif

BAD_NAME = $(basename $(notdir $(CURDIR)))
NAME = $(word $(words $(BAD_NAME)),$(BAD_NAME))

SPLIT_PATH = $(subst /, ,$(dir $(CURDIR)))
BUILD_TYPE = $(word $(words $(SPLIT_PATH)),$(SPLIT_PATH))

ifdef VERBOSE
  SILENT =
else
  SILENT = @
endif

ifdef CC_SOURCES
 $(warning Please use CXX_SOURCES instead of CC_SOURCES to define the C++ sources)
endif
ifdef CPP_SOURCES
 $(warning Please use CXX_SOURCES instead of CPP_SOURCES to define the C++ sources)
endif
CXX_SOURCES += $(CC_SOURCES) $(CPP_SOURCES)

ifeq ($(C_SOURCES),)
 ifeq ($(strip $(CXX_SOURCES)),)
  C_SOURCES = $(shell ls $(NAME).c 2> /dev/null)
  CXX_SOURCES = $(shell ls $(NAME).cpp 2> /dev/null)
  ifeq ($(CXX_SOURCES),)
   CXX_SOURCES = $(shell ls $(NAME).cc 2> /dev/null)
   ifeq ($(CXX_SOURCES),)
    CXX_SOURCES = $(shell ls $(NAME).c++ 2> /dev/null)
   endif
  endif
 endif
endif

ifneq ($(strip $(CXX_SOURCES)),)
 USE_CXX = true
endif
ifneq ($(strip $(C_SOURCES)),)
 USE_C = true
endif

SOURCES_DIRECTORIES = $(sort $(dir $(C_SOURCES) $(CXX_SOURCES)))
OBJECTS_LIST = $(patsubst %.c,%.o, $(patsubst %.cc,%.o, $(patsubst %.c++,%.o, $(patsubst %.cpp,%.o, $(notdir $(C_SOURCES) $(CXX_SOURCES))))))

ifeq ($(BUILD_TYPE),physics)
 USE_ODE = true
 EXCLUDE_CONTROLLERS = true
endif
ifeq ($(BUILD_TYPE),controllers)
 BUILD_EXECUTABLE = true
else
 ifdef STATIC_LIBRARY
  BUILD_STATIC_LIBRARY = true
 else
  BUILD_SHARED_LIBRARY = true
 endif
endif

ifneq ($(C_SOURCES)$(CXX_SOURCES),)
 ifdef BUILD_EXECUTABLE
  MAIN_TARGET = $(NAME)$(EXE_EXTENSION)
  X86_64_LINKER = -target x86_64-apple-macos12
  ARM64_LINKER = -target arm64-apple-macos12
 else
  ifdef BUILD_STATIC_LIBRARY
   MAIN_TARGET ?= $(LIB_PREFIX)$(NAME)$(STATIC_LIB_EXTENSION)
  else
   MAIN_TARGET ?= $(LIB_PREFIX)$(NAME)$(SHARED_LIB_EXTENSION)
   X86_64_LINKER = -target x86_64-apple-macos12
   ARM64_LINKER = -target arm64-apple-macos12
  endif
 endif
endif

ifdef WREN
 WREN_TARGET = wrenjs.js
endif

TARGETS += $(MAIN_TARGET) $(EXTRA_TARGETS) $(WREN_TARGET)

SUPPORTED_TARGETS = all release debug profile
ifneq ($(filter all,$(MAKECMDGOALS)),)
 GOAL = release
else ifneq ($(filter $(MAIN_TARGET),$(MAKECMDGOALS)),)
 GOAL = release
else ifneq ($(filter $(SUPPORTED_TARGETS),$(MAKECMDGOALS)),)
 GOAL = $(firstword $(filter $(SUPPORTED_TARGETS),$(MAKECMDGOALS)))
else ifeq ($(MAKECMDGOALS),)
 GOAL = release
 GOAL_EMCC = emcc
endif

BUILD_DIR = build
BUILD_GOAL_DIR = $(BUILD_DIR)/$(GOAL)
BUILD_GOAL_DIR_EMCC = $(BUILD_DIR)/$(GOAL_EMCC)

BUILD_DIRECTORIES =
ifdef GOAL
 BUILD_DIRECTORIES += $(BUILD_DIR) $(BUILD_GOAL_DIR)
 ifdef FAT_BINARY
  BUILD_DIRECTORIES += $(BUILD_GOAL_DIR)/x86_64 $(BUILD_GOAL_DIR)/arm64
 endif
endif

ifdef GOAL_EMCC
 BUILD_DIRECTORIES += $(BUILD_DIR) $(BUILD_GOAL_DIR_EMCC)
 OBJECTS_EMCC = $(addprefix $(BUILD_GOAL_DIR_EMCC)/,$(OBJECTS_LIST))
endif
BUILD_DIRECTORIES := $(sort $(BUILD_DIRECTORIES))

FILES_TO_REMOVE += $(MAIN_TARGET_COPY)
ifdef TARGET_LIB_DIR
 FILES_TO_REMOVE += $(TARGET_LIB_DIR)/$(MAIN_TARGET)
endif

WRENJS_DIR = $(WEBOTS_HOME_PATH)/resources/web/wwi
