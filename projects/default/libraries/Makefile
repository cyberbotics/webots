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

WGET=wget --quiet

ifeq ($(OSTYPE),linux)
 OS_TAG=linux64
 ROS_PACKAGE=ros_kinetic_ubuntu16.04.tar.gz
 EXTRACT=tar xfz
endif

ifeq ($(OSTYPE),darwin)
 OS_TAG=mac
 ROS_PACKAGE=ros_mac.zip
 EXTRACT=unzip -q
endif

ROS_TARGET=ros/libroscpp$(SHARED_LIB_EXTENSION)

# remove .tar.gz
ROS_SOURCE=$(basename $(basename $(ROS_PACKAGE)))

release debug profile: $(ROS_TARGET)

$(ROS_TARGET):
	@if ! $(WGET) -h > /dev/null ; then echo "You need to download the" $(ROS_PACKAGE) "package and copy it in this directory"; fi
	@echo "#" downloading $(ROS_PACKAGE)
	@$(WGET) https://www.cyberbotics.com/files/repository/dependencies/$(OS_TAG)/release/$(ROS_PACKAGE)
	@echo "#" extracting $(ROS_PACKAGE)
	@$(EXTRACT) $(ROS_PACKAGE)
	@echo "#" removing old ros
	@rm -rf $(ROS_PACKAGE)
	@rm -rf ros/*$(SHARED_LIB_EXTENSION).* ros/*
	@echo "#" installing $(ROS_PACKAGE)
ifeq ($(OSTYPE),darwin)
	@mv *.dylib ros
else ifeq ($(OSTYPE),linux)
	@mv $(ROS_SOURCE)/* ros
endif
	@rm -rf $(ROS_SOURCE)
	@touch ros/* # update modification date on Linux and Mac

clean:
	rm -rf ros/*$(SHARED_LIB_EXTENSION).* ros/*$(SHARED_LIB_EXTENSION)
	rm -rf $(ROS_PACKAGE)
