# Copyright 1996-2020 Cyberbotics Ltd.
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

ifeq ($(OSTYPE),windows)
CMAKE_FLAGS = -G "MSYS Makefiles"
endif

ifeq ($(OSTYPE),linux)
unexport LD_LIBRARY_PATH
endif

release debug profile:
ifneq (, $(shell which cmake 2> /dev/null))
	@mkdir -p dashel/dashel
	@cmake -Bdashel -Hdashel-src $(CMAKE_FLAGS) > /dev/null
	@cp dashel-src/dashel/dashel.h dashel/dashel/
	@+make -C dashel > /dev/null
else
	@echo -e "# \033[0;33mcmake not installed, skipping dashel library\033[0m"
endif

clean:
	@rm -fr dashel
