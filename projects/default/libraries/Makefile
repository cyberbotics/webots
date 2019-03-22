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

ROS_TARGET=ros

ifeq ($(OSTYPE),linux)
 HOSTTYPE ?= $(shell uname -m)
 OS_TAG=linux64
 ROS_PACKAGE=$(ROS_TARGET)_kinetic_ubuntu16.04.tar.gz
 EXTRACT=tar xfz
 WGET=wget --quiet
endif

ifeq ($(OSTYPE),darwin)
 OS_TAG=mac
 ROS_PACKAGE=$(ROS_TARGET)_mac.zip
 EXTRACT=unzip -q
 WGET=curl -O --silent
endif

ifeq ($(OSTYPE),windows)
 OS_TAG=windows
 ROS_PACKAGE=$(ROS_TARGET)_windows.zip
 EXTRACT=unzip -q
 WGET=wget --quiet
 # remove .zip
 ROS_SOURCE=$(basename $(ROS_PACKAGE))
else
 # remove .tar.gz
 ROS_SOURCE=$(basename $(basename $(ROS_PACKAGE)))
endif

release debug profile: $(ROS_TARGET)

$(ROS_PACKAGE):
	@if ! $(WGET) -h > /dev/null ; then echo "You need to download the" $(ROS_PACKAGE) "package and copy it in this directory"; fi
	@echo "#" downloading $(ROS_PACKAGE)
	@$(WGET) https://www.cyberbotics.com/files/repository/dependencies/$(OS_TAG)/release/$(ROS_PACKAGE)
	@echo "#" extracting $(ROS_PACKAGE)
	@$(EXTRACT) $(ROS_PACKAGE)
	@echo "#" removing old $(ROS_TARGET)
	@rm -rf $(ROS_TARGET)/*$(SHARED_LIB_EXTENSION).* $(ROS_TARGET)/*$(SHARED_LIB_EXTENSION)
	@echo "#" installing $(ROS_PACKAGE)
ifeq ($(OSTYPE),windows)
	@mv *.dll $(ROS_TARGET)
else ifeq ($(OSTYPE),darwin)
	@mv *.dylib $(ROS_TARGET)
else
	@mv $(ROS_SOURCE)/* $(ROS_TARGET)
endif
	@rm -rf $(ROS_SOURCE)
	@touch $(ROS_TARGET) # update modification date on Linux and Mac

$(ROS_TARGET): $(ROS_PACKAGE)

clean:
	rm -rf $(ROS_TARGET)/*$(SHARED_LIB_EXTENSION).* $(ROS_TARGET)/*$(SHARED_LIB_EXTENSION)
	rm -rf $(ROS_PACKAGE)
