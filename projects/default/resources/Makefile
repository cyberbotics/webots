# Copyright 1996-2019 Cyberbotics Ltd.
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

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

SUMO_TARGET=sumo
SUMO_VERSION=0.30

ifeq ($(OSTYPE),linux)
 HOSTTYPE ?= $(shell uname -m)
 OS_TAG=linux64
 SUMO_PACKAGE=sumo-$(SUMO_VERSION)-linux64.tar.gz
 PREVIOUS_SUMO_PACKAGE=$(filter-out $(SUMO_PACKAGE),$(wildcard sumo-*-linux64.tar.gz))
 EXTRACT=tar xfz
 WGET=wget --quiet
endif

ifeq ($(OSTYPE),darwin)
 OS_TAG=mac
 SUMO_PACKAGE=sumo-$(SUMO_VERSION)-mac.tar.gz
 PREVIOUS_SUMO_PACKAGE=$(filter-out $(SUMO_PACKAGE),$(wildcard sumo-*-mac.tar.gz))
 EXTRACT=tar xfz
 WGET=curl -O --silent
endif

ifeq ($(OSTYPE),windows)
 OS_TAG=windows
 SUMO_PACKAGE=sumo-$(SUMO_VERSION)-windows.zip
 PREVIOUS_SUMO_PACKAGE=$(filter-out $(SUMO_PACKAGE),$(wildcard sumo-*-windows.zip))
 EXTRACT=unzip -q
 WGET=wget --quiet
endif

ifeq ($(OSTYPE),windows)
 # remove .zip
 SUMO_SOURCE=$(basename $(SUMO_PACKAGE))
else
 # remove .tar.gz
 SUMO_SOURCE=$(basename $(basename $(SUMO_PACKAGE)))
endif

release debug profile: $(SUMO_TARGET)

$(SUMO_PACKAGE):
	@if ! $(WGET) -h > /dev/null ; then echo "You need to download the" $(SUMO_PACKAGE) "package and copy it in this directory"; fi
	@echo "#" downloading $(SUMO_PACKAGE)
	@$(WGET) https://www.cyberbotics.com/files/repository/dependencies/$(OS_TAG)/release/$(SUMO_PACKAGE)
	@echo "#" extracting $(SUMO_PACKAGE)
	@$(EXTRACT) $(SUMO_PACKAGE)
	@echo "#" removing old $(SUMO_TARGET)
	@rm -rf $(SUMO_TARGET)
ifneq ($(PREVIOUS_SUMO_PACKAGE),)
	@rm -f $(PREVIOUS_SUMO_PACKAGE)
endif
	@echo "#" installing $(SUMO_PACKAGE)
	@mv $(SUMO_SOURCE) $(SUMO_TARGET)
	@touch $(SUMO_TARGET) # update modification date on Linux and Mac

$(SUMO_TARGET): $(SUMO_PACKAGE)

clean:
	rm -rf $(SUMO_TARGET)
	rm -rf $(SUMO_PACKAGE)
