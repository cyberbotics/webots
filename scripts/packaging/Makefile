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

# WEBOTS_HOME is a sine qua non condition to run this Makefile
ifndef WEBOTS_HOME
 $(error The WEBOTS_HOME environment variable is not defined.)
endif

ifndef WEBOTS_HOME_PATH
 null :=
 space := $(null) $(null)
 WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
endif

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

ifneq ($(SNAPCRAFT_PROJECT_NAME),)
ifeq ($(DISPLAY),)
export DISPLAY = :99
XVFB = xvfb
endif
endif

all: $(WEBOTS_DISTRO_EXE) $(XVFB)
	+@echo "# removing files and folders not in git repository"
	+@cd $(WEBOTS_HOME_PATH) ; git clean -fdf | sed -e 's/^/# - /' ; cd scripts/packaging
	+@echo "# checking submodules update"
	+@./check_submodules_update.sh
	+@echo "# generating package"
	+@python3 webots_distro.py
	+@echo "# done for" $(OSTYPE)

install: all
	./webots.snap

xvfb:
	+@echo "# starting Xvfb: X Virtual Frame Buffer"
	+@Xvfb :99 -screen 0 1024x768x16 &

clean:
	+@rm -rf $(WEBOTS_DISTRO_EXE) webots.snap webots.iss
	+@find $(WEBOTS_HOME_PATH)/projects $(WEBOTS_HOME_PATH)/resources -name .*.cache | xargs rm -f
	+@rm -rf __pycache__ node_modules package-lock.json
