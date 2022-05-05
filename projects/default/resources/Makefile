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

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

SUMO_TARGET=sumo
SUMO_VERSION=1.10
VER_UBUNTU = 20.04

ifeq ($(OSTYPE),linux)
 VER_UBUNTU = $(shell lsb_release -sr)
 HOSTTYPE ?= $(shell uname -m)
 OS_TAG=linux64
 EXTRACT=tar xfm
ifeq ($(VER_UBUNTU),18.04)
 SUMO_VERSION=1.11
 SUMO_PACKAGE=sumo-$(SUMO_VERSION)-linux64.tar.gz
 SUMO_MD5SUM="aee08d40dfa26ea1167e6a6fd99b5ced"
 EXTRACT=tar xfz
else ifeq ($(VER_UBUNTU),22.04)
 SUMO_VERSION=1.13
 SUMO_MD5SUM="3485da3c7a4e157e0f3811783828bb84"
 SUMO_PACKAGE=sumo-$(SUMO_VERSION)-Ubuntu_22.04.tar.bz2
else
 SUMO_PACKAGE=sumo-$(SUMO_VERSION)-Ubuntu_20.04.tar.bz2
 SUMO_MD5SUM="dc4283d9c98da59e944c01ffb61dbf16"
endif
 PREVIOUS_SUMO_PACKAGE=$(filter-out $(SUMO_PACKAGE),$(wildcard sumo-*-Ubuntu_*.tar.bz2))
 # remove .tar.gz
 SUMO_SOURCE=$(basename $(basename $(SUMO_PACKAGE)))
 WGET=wget --quiet
endif

ifeq ($(OSTYPE),darwin)
 OS_TAG=mac
 SUMO_PACKAGE=sumo-$(SUMO_VERSION)-mac.tar.gz
 SUMO_MD5SUM="a8900ed025fb930cb78609f08b6cd204"
 PREVIOUS_SUMO_PACKAGE=$(filter-out $(SUMO_PACKAGE),$(wildcard sumo-*-mac.tar.gz))
 EXTRACT=tar xfz
 # remove .tar.gz
 SUMO_SOURCE=$(basename $(basename $(SUMO_PACKAGE)))
 WGET=LANG=en_US.UTF-8 wget --quiet
endif

ifeq ($(OSTYPE),windows)
 OS_TAG=windows
 SUMO_PACKAGE=sumo-$(SUMO_VERSION)-windows.zip
 SUMO_MD5SUM="2fd97eeb01fb11d4e95c3e95fd2bb4a3"
 PREVIOUS_SUMO_PACKAGE=$(filter-out $(SUMO_PACKAGE),$(wildcard sumo-*-windows.zip))
 EXTRACT=unzip -q
 # remove .zip
 SUMO_SOURCE=$(basename $(SUMO_PACKAGE))
 WGET=wget --quiet
endif

release debug profile: $(SUMO_TARGET)

$(SUMO_TARGET):
	@if ! $(WGET) -h > /dev/null ; then echo "You need to download the" $(SUMO_PACKAGE) "package and copy it in this directory"; fi
	@echo "#" downloading $(SUMO_PACKAGE)
	@$(WGET) https://www.cyberbotics.com/files/repository/dependencies/$(OS_TAG)/release/$(SUMO_PACKAGE)
ifeq ($(OSTYPE),darwin)
	@if [ `md5 -q $(SUMO_PACKAGE)` != $(SUMO_MD5SUM) ]; then echo "MD5 checksum failed for $(SUMO_PACKAGE)"; exit 1; fi
else
	@if [ "$$(md5sum $(SUMO_PACKAGE) | awk '{print $$1;}')" != $(SUMO_MD5SUM) ]; then echo "MD5 checksum failed for $(SUMO_PACKAGE)"; exit 1; fi
endif
	@echo "#" extracting $(SUMO_PACKAGE)
	@$(EXTRACT) $(SUMO_PACKAGE)
	@echo "#" removing old $(SUMO_TARGET)
	@rm -rf $(SUMO_TARGET)
	@rm -rf $(SUMO_PACKAGE)
ifneq ($(PREVIOUS_SUMO_PACKAGE),)
	@rm -f $(PREVIOUS_SUMO_PACKAGE)
endif
	@echo "#" installing $(SUMO_PACKAGE)
	@mv $(SUMO_SOURCE) $(SUMO_TARGET)
	@touch $(SUMO_TARGET) # update modification date on Linux and Mac

clean:
	rm -rf $(SUMO_TARGET)
	rm -rf $(SUMO_PACKAGE)
