# Copyright 1996-2022 Cyberbotics Ltd.
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
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
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

ifeq ($(OSTYPE), darwin)

release debug profile: physics.a

physics-x86_64.o: physics.c $(WEBOTS_HOME_PATH)/include/plugins/physics.h
	@gcc -target x86_64-apple-macos11 $(CFLAGS) $(PLATFORM_FLAGS) physics.c -I$(WEBOTS_HOME_PATH)/include -I$(WEBOTS_HOME_PATH)/include/ode -o $@

physics-arm64.o: physics.c $(WEBOTS_HOME_PATH)/include/plugins/physics.h
	@gcc -target arm64-apple-macos11 $(CFLAGS) $(PLATFORM_FLAGS) physics.c -I$(WEBOTS_HOME_PATH)/include -I$(WEBOTS_HOME_PATH)/include/ode -o $@

physics.a: physics-x86_64.o physics-arm64.o
	@ar rcs physics-x86_64.a physics-x86_64.o
	@ar rcs physics-arm64.a physics-arm64.o
	@lipo -create -output $@ physics-arm64.a physics-x86_64.a

else

release debug profile: physics.o

physics.o: physics.c $(WEBOTS_HOME_PATH)/include/plugins/physics.h
	@gcc $(CFLAGS) $(PLATFORM_FLAGS) physics.c -I$(WEBOTS_HOME_PATH)/include -I$(WEBOTS_HOME_PATH)/include/ode

endif

clean:
	@rm -f physics*.o physics*.a
