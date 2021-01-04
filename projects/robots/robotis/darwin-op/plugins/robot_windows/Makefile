# Copyright 1996-2021 Cyberbotics Ltd.
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

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

TARGETS = robotis-op2_window.Makefile

ifneq ($(MAKECMDGOALS),clean)
ifeq ($(OSTYPE),windows)
ifeq (,$(wildcard /mingw64/include/libssh))
NO_LIBSSH = 1
endif
ifeq (,$(wildcard /mingw64/include/zip.h))
NO_LIBZIP = 1
endif
endif
endif

.PHONY: release debug profile clean

release debug profile clean: $(TARGETS)

%.Makefile:
ifeq ($(NO_LIBSSH),1)
	+@echo -e "# \033[0;33mlibssh not installed, skipping Robotis OP2 robot window\033[0m"
else
ifeq ($(NO_LIBZIP),1)
	+@echo -e "# \033[0;33mlibzip not installed, skipping Robotis OP2 robot window\033[0m"
else
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)
endif
endif
