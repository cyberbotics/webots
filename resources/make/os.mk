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

# OS, architecture, distribution, extensions, and default library locations.

ifeq ($(OS),Windows_NT)
 OSTYPE = windows
endif

ifndef OSTYPE
 OSTYPE := $(shell uname)
endif

ifeq ($(OSTYPE),Linux)
 OSTYPE = linux
endif

ifeq ($(OSTYPE),linux-gnu)
 OSTYPE = linux
endif

ifeq ($(OSTYPE),)
 OSTYPE = windows
endif

ifeq ($(OSTYPE),msys)
 OSTYPE = windows
endif

ifneq ($(findstring MINGW,$(OSTYPE)),)
 OSTYPE = windows
endif

ifneq ($(findstring CYGWIN,$(OSTYPE)),)
 OSTYPE = windows
endif

ifeq ($(OSTYPE),Darwin)
 OSTYPE = darwin
endif

ifeq ($(OSTYPE),linux)
  ifeq ($(filter x86_64, $(shell uname -a)),)
    OSARCH = i386
  else
    OSARCH = x86_64
  endif
  OSDIST=$(shell grep ^ID= /etc/os-release | cut -d= -f2)
  ifeq ($(SNAP_NAME), webots)
    UBUNTU_VERSION=22.04
  else
    ifneq (, $(wildcard /etc/os-release))
      UBUNTU_VERSION:=$(shell grep VERSION_ID /etc/os-release | cut -d= -f2 | tr -d '"')
    endif
  endif
else
  OSDIST=$(OSTYPE)
endif

ifeq ($(OSTYPE),darwin)
 SUPPORTED_SDK = 12 13 14
 MACOSX_MIN_SDK_VERSION = 12
 ECHO = echo
 MACOSX_SDK_PATH = $(shell xcrun --sdk macosx --show-sdk-path)
 ifeq ($(wildcard $(MACOSX_SDK_PATH)),)
  $(error The macOS SDK is not found. Please check that XCode and its command line tools are correctly installed.)
 endif
 MACOSX_SDK_VERSION = $(shell egrep -o '10.[0-9][0-9]' <<< $(MACOSX_SDK_PATH) )
 ifneq ($(MACOSX_SDK_VERSION),$(filter $(MACOSX_SDK_VERSION),$(SUPPORTED_SDK)))
  $(warning Your macOS SDK ("$(MACOSX_SDK_PATH)") is $(MACOSX_SDK_VERSION) while Webots supports the $(SUPPORTED_SDK) SDKs.)
 endif
else
 ECHO = echo -e
endif

STATIC_LIB_EXTENSION = .a
ifeq ($(OSTYPE),windows)
 EXE_EXTENSION = .exe
 SHARED_LIB_EXTENSION = .dll
 WEBOTS_LIB_PATH?=$(subst \,/,$(WEBOTS_HOME))/msys64/mingw64/bin
 WEBOTS_CONTROLLER_LIB_PATH?=$(subst \,/,$(WEBOTS_HOME))/lib/controller
else
 LIB_PREFIX = lib
 ifeq ($(OSTYPE),linux)
  SHARED_LIB_EXTENSION = .so
  WEBOTS_LIB_PATH?=$(WEBOTS_HOME)/lib/webots
  WEBOTS_CONTROLLER_LIB_PATH?=$(WEBOTS_HOME)/lib/controller
 else
  SHARED_LIB_EXTENSION = .dylib
  WEBOTS_LIB_PATH?=$(WEBOTS_HOME)/Contents/lib/webots
  WEBOTS_CONTROLLER_LIB_PATH?=$(WEBOTS_HOME)/Contents/lib/controller
 endif
endif
