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

# Webots SDK include and library flags, preferring pkg-config on Linux.

INCLUDE += -I.
WBCFLAGS += -Wall

WEBOTS_PKG_CONFIG ?= pkg-config
ifeq ($(OSTYPE),linux)
 webots_pkg_config_exists = $(shell $(WEBOTS_PKG_CONFIG) --exists $(1) >/dev/null 2>&1 && echo true)
 webots_pkg_config_cflags = $(shell $(WEBOTS_PKG_CONFIG) --cflags $(1) 2>/dev/null)
 webots_pkg_config_libs = $(shell $(WEBOTS_PKG_CONFIG) --libs $(1) 2>/dev/null)
endif

ifeq ($(GOAL),debug)
 WBCFLAGS += -ggdb
endif

ifeq ($(TREAT_WARNINGS_AS_ERRORS),1)
CFLAGS += -Werror
endif

ifeq ($(GOAL),release)
 WBCFLAGS += -O3 -DNDEBUG
 ifneq ($(OSTYPE),darwin)
  DYNAMIC_LINK_FLAGS += -s
 endif
endif

ifeq ($(GOAL),profile)
 WBCFLAGS += -pg
 ifneq ($(OSTYPE),darwin)
  ifndef BUILD_STATIC_LIBRARY
   LFLAGS += -pg
  endif
 endif
endif

ifeq ($(OSTYPE),windows)
 WBCFLAGS += -mwindows -Wl,-subsystem,windows -D_GLIBCXX_USE_CXX11_ABI=1
endif

ifeq ($(OSTYPE),linux)
 DYNAMIC_LIBRARIES += -lm
endif

ifeq ($(OSTYPE),darwin)
ifdef NO_FAT_BINARY
 WBCFLAGS += -target $(PROCESSOR)-apple-macos12
 DYNAMIC_LINK_FLAGS += -target $(PROCESSOR)-apple-macos12
endif
 WBCFLAGS += -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
 DYNAMIC_LINK_FLAGS += -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
endif

ifdef USE_CXX
 ifdef BUILD_STATIC_LIBRARY
  LINKER = ar
 else
  LINKER = $(CXX)
 endif
 ifndef EXCLUDE_CONTROLLERS
  ifeq ($(OSTYPE),linux)
   ifeq ($(call webots_pkg_config_exists,webots-controller),true)
    INCLUDE += $(call webots_pkg_config_cflags,webots-controller)
    DYNAMIC_LIBRARIES += $(call webots_pkg_config_libs,webots-controller)
   else
    INCLUDE += -I"$(WEBOTS_HOME)/include/controller/c"
    DYNAMIC_LIBRARIES += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lController
   endif
   ifdef USE_C_API
   else
    ifeq ($(call webots_pkg_config_exists,webots-cpp-controller),true)
     INCLUDE += $(call webots_pkg_config_cflags,webots-cpp-controller)
     DYNAMIC_LIBRARIES += $(call webots_pkg_config_libs,webots-cpp-controller)
    else
     INCLUDE += -I"$(WEBOTS_HOME)/include/controller/cpp"
     DYNAMIC_LIBRARIES += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lCppController
    endif
   endif
  else
   DYNAMIC_LIBRARIES += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lController
   ifdef USE_C_API
    ifeq ($(OSTYPE),darwin)
     INCLUDE += -I"$(WEBOTS_HOME)/Contents/include/controller/c"
    else
     INCLUDE += -I"$(WEBOTS_HOME)/include/controller/c"
    endif
   else
    DYNAMIC_LIBRARIES += -lCppController
    ifeq ($(OSTYPE),darwin)
     INCLUDE += -I"$(WEBOTS_HOME)/Contents/include/controller/cpp"
    else
     INCLUDE += -I"$(WEBOTS_HOME)/include/controller/cpp"
    endif
   endif
   ifeq ($(OSTYPE),windows)
    DYNAMIC_LINK_FLAGS += -Wl,--enable-auto-import
   endif
  endif
 endif
 ifeq ($(OSTYPE),darwin)
  WBCXXFLAGS += -stdlib=libc++
  DYNAMIC_LINK_FLAGS += -stdlib=libc++
 endif
else
 ifdef BUILD_STATIC_LIBRARY
  LINKER = ar
 else
  LINKER = $(CC)
 endif
 ifndef EXCLUDE_CONTROLLERS
  ifeq ($(OSTYPE),linux)
   ifeq ($(call webots_pkg_config_exists,webots-controller),true)
    INCLUDE += $(call webots_pkg_config_cflags,webots-controller)
    DYNAMIC_LIBRARIES += $(call webots_pkg_config_libs,webots-controller)
   else
    INCLUDE += -I"$(WEBOTS_HOME)/include/controller/c"
    DYNAMIC_LIBRARIES += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lController
   endif
  else
   ifeq ($(OSTYPE),darwin)
    INCLUDE += -I"$(WEBOTS_HOME)/Contents/include/controller/c"
   else
    INCLUDE += -I"$(WEBOTS_HOME)/include/controller/c"
   endif
   DYNAMIC_LIBRARIES += -L"$(WEBOTS_CONTROLLER_LIB_PATH)" -lController
  endif
 endif
endif

ifdef BUILD_SHARED_LIBRARY
 DYNAMIC_LINK_FLAGS += -shared
 ifeq ($(OSTYPE),linux)
  WBCFLAGS += -fPIC
 endif
 ifeq ($(OSTYPE),darwin)
  DYNAMIC_LINK_FLAGS += -dynamiclib -compatibility_version 1.0 -current_version 1.0.0
 endif
endif

ifdef BUILD_STATIC_LIBRARY
 ifeq ($(OSTYPE),linux)
  WBCFLAGS += -fPIC
 endif
 STATIC_LINK_FLAGS += rvs
endif

ifeq ($(OSTYPE),darwin)
 ifdef WEBOTS_LIBRARY
  INSTALL_NAME ?= @rpath/Contents/lib/controller/$(MAIN_TARGET)
  LFLAGS += -Xlinker -rpath -Xlinker @loader_path/.. -install_name $(INSTALL_NAME)
 else
  CALLING_MAKEFILE_DIR := $(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
  WEBOTS_RELATIVE_PATH = $(call computeRelativePath,"$(WEBOTS_HOME_PATH)","$(CALLING_MAKEFILE_DIR)")
  ifneq (,$(findstring ..,$(WEBOTS_RELATIVE_PATH)))
   ifeq ($(TREAT_WARNINGS_AS_ERRORS),1)
    DYNAMIC_LINK_FLAGS += -Xlinker -rpath -Xlinker @loader_path/$(WEBOTS_RELATIVE_PATH)../
   else
    DYNAMIC_LINK_FLAGS += -Xlinker -rpath -Xlinker @loader_path/$(WEBOTS_RELATIVE_PATH)
   endif
   ifdef BUILD_SHARED_LIBRARY
    INSTALL_NAME ?= @rpath$(subst $(WEBOTS_HOME_PATH),,$(CALLING_MAKEFILE_DIR))/$(MAIN_TARGET)
    DYNAMIC_LINK_FLAGS += -install_name $(INSTALL_NAME)
   endif
  endif
 endif
endif

ifdef USE_ODE
 ifeq ($(OSTYPE),linux)
  ifeq ($(call webots_pkg_config_exists,webots-ode),true)
   INCLUDE += $(call webots_pkg_config_cflags,webots-ode)
   DYNAMIC_LIBRARIES += $(call webots_pkg_config_libs,webots-ode)
  else
   DYNAMIC_LIBRARIES += -L"$(WEBOTS_LIB_PATH)" -lode
   INCLUDE += -I"$(WEBOTS_HOME)/include/ode" -I"$(WEBOTS_HOME)/include"
  endif
 else
  DYNAMIC_LIBRARIES += -L"$(WEBOTS_LIB_PATH)" -lode
 endif
 ifeq ($(GOAL),profile)
  DYNAMIC_LIBRARIES += -lstdc++
 endif
 ifeq ($(OSTYPE),darwin)
  DYNAMIC_LINK_FLAGS += -flat_namespace -undefined suppress
  DYNAMIC_LIBRARIES += "$(WEBOTS_HOME)/Contents/Resources/projects/plugins/physics/physics.a"
  INCLUDE += -I"$(WEBOTS_HOME)/Contents/include/ode" -I"$(WEBOTS_HOME)/Contents/include"
 else
  DYNAMIC_LIBRARIES += "$(WEBOTS_HOME)/resources/projects/plugins/physics/physics.o"
 endif
endif
