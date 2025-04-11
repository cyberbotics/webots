# Copyright 1996-2024 Cyberbotics Ltd.
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
	@echo "  make release               for final release"
	@echo "  make clean                 remove build directory"

include $(WEBOTS_HOME_PATH)/resources/Makefile.include

ifeq ($(OSTYPE),windows)
CONTROLLER_LAUNCHER = $(WEBOTS_HOME_PATH)/msys64/mingw64/bin/webots-controller.exe
else ifeq ($(OSTYPE),darwin)
CONTROLLER_LAUNCHER = $(WEBOTS_HOME_PATH)/Contents/MacOS/webots-controller
else
CONTROLLER_LAUNCHER = $(WEBOTS_HOME_PATH)/webots-controller
endif

.PHONY: release debug profile

release debug profile: $(CONTROLLER_LAUNCHER)

ifeq ($(OSTYPE),darwin)

OBJDIR = build/$(MAKECMDGOALS)
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR)/x86_64 $(OBJDIR)/arm64)

$(CONTROLLER_LAUNCHER): $(OBJDIR)/x86_64/webots-controller $(OBJDIR)/arm64/webots-controller
	@echo "# creating fat " $(notdir $@)
	@lipo -create -output $@ $(OBJDIR)/x86_64/webots-controller $(OBJDIR)/arm64/webots-controller
	@codesign -s - $@

$(OBJDIR)/x86_64/webots-controller: webots_controller.c
	@echo "# compiling " $@ "(x64_64)"
	@gcc $(CFLAGS) -target x86_64-apple-macos12 webots_controller.c -o $@

$(OBJDIR)/arm64/webots-controller: webots_controller.c
	@echo "# compiling " $@ "(arm64)"
	@gcc $(CFLAGS) -target arm64-apple-macos12 webots_controller.c -o $@

else

$(CONTROLLER_LAUNCHER): webots_controller.c
	@echo "# compiling" $<
	@gcc $(CFLAGS) webots_controller.c -o $@

endif

FILES_TO_REMOVE += $(CONTROLLER_LAUNCHER)
