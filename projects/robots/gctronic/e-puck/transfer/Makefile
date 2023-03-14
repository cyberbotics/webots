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

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

XC16_TARGET=xc16
XC16_VERSION=1.24
XC16_PACKAGE=$(XC16_TARGET)-$(XC16_VERSION).zip
PREVIOUS_XC16_PACKAGE=$(filter-out $(XC16_PACKAGE),$(wildcard XC16-*.zip))
EXTRACT=unzip -q

ifeq ($(OSTYPE),linux)
 OS_TAG=linux64
 XC16_MD5SUM="01f1fbde40335014e9c60b38fd5b1bcb"
 WGET=wget --quiet
endif

ifeq ($(OSTYPE),darwin)
 OS_TAG=mac
 XC16_MD5SUM="409c5feadbe09984c6d377c8509731eb"
 WGET=LANG=en_US.UTF-8 wget --quiet
endif

ifeq ($(OSTYPE),windows)
 OS_TAG=windows
 XC16_MD5SUM="8187594d8356b84bf852649a13380362"
 WGET=wget --quiet
endif

release debug profile: $(XC16_TARGET)

$(XC16_TARGET):
	@if ! $(WGET) -h > /dev/null ; then echo "You need to download the" $(XC16_PACKAGE) "package and copy it in this directory"; fi
	@echo "#" downloading $(XC16_PACKAGE)
	@$(WGET) https://www.cyberbotics.com/files/repository/dependencies/$(OS_TAG)/release/$(XC16_PACKAGE)
ifeq ($(OSTYPE),darwin)
	@if [ `md5 -q $(XC16_PACKAGE)` != $(XC16_MD5SUM) ]; then echo "MD5 checksum failed for $(XC16_PACKAGE)"; exit 1; fi
else
	@if [ "$$(md5sum $(XC16_PACKAGE) | awk '{print $$1;}')" != $(XC16_MD5SUM) ]; then echo "MD5 checksum failed for $(XC16_PACKAGE)"; exit 1; fi
endif
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
