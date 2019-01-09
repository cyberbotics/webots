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

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

CFLAGS = -c -O3

ifeq ($(OSTYPE), darwin)
PLATFORM_FLAGS = -fPIC -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
endif
ifeq ($(OSTYPE), windows)
PLATFORM_FLAGS =
endif
ifeq ($(OSTYPE), linux)
PLATFORM_FLAGS = -fPIC
endif

physics.o: physics.c $(WEBOTS_HOME_PATH)/include/plugins/physics.h
	@gcc $(CFLAGS) $(PLATFORM_FLAGS) physics.c -I$(WEBOTS_HOME_PATH)/include -I$(WEBOTS_HOME_PATH)/include/ode
release debug profile: physics.o
clean:
	@rm -f physics.o
