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

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

XC16_TARGET=xc16
XC16_VERSION=1.24

ifeq ($(OSTYPE),linux)
 HOSTTYPE ?= $(shell uname -m)
 OS_TAG=linux64
 XC16_PACKAGE=$(XC16_TARGET)-$(XC16_VERSION).zip
 PREVIOUS_XC16_PACKAGE=$(filter-out $(XC16_PACKAGE),$(wildcard XC16-*.zip))
 EXTRACT=unzip -q
 WGET=wget --quiet
endif

ifeq ($(OSTYPE),darwin)
 OS_TAG=mac
 XC16_PACKAGE=$(XC16_TARGET)-$(XC16_VERSION).zip
 PREVIOUS_XC16_PACKAGE=$(filter-out $(XC16_PACKAGE),$(wildcard XC16-*zip))
 EXTRACT=unzip -q
 WGET=curl -O --silent
endif

ifeq ($(OSTYPE),windows)
 OS_TAG=windows
 XC16_PACKAGE=$(XC16_TARGET)-$(XC16_VERSION).zip
 PREVIOUS_XC16_PACKAGE=$(filter-out $(XC16_PACKAGE),$(wildcard XC16-*.zip))
 EXTRACT=unzip -q
 WGET=wget --quiet
endif

release debug profile: $(XC16_TARGET)

$(XC16_TARGET):
	@if ! $(WGET) -h > /dev/null ; then echo "You need to download the" $(XC16_PACKAGE) "package and copy it in this directory"; fi
	@echo "#" downloading $(XC16_PACKAGE)
	@$(WGET) https://www.cyberbotics.com/files/repository/dependencies/$(OS_TAG)/release/$(XC16_PACKAGE)
	@echo "#" removing old $(XC16_TARGET)
	@rm -rf $(XC16_TARGET)
	@echo "#" extracting $(XC16_PACKAGE)
	@$(EXTRACT) $(XC16_PACKAGE)
	@rm -rf $(XC16_PACKAGE)
ifneq ($(PREVIOUS_XC16_PACKAGE),)
	@rm -f $(PREVIOUS_XC16_PACKAGE)
endif
	@touch $(XC16_TARGET) # update modification date on Linux and Mac

clean:
	rm -rf $(XC16_TARGET)
	rm -rf $(XC16_PACKAGE)
