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

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

ROS_TARGET=ros
ROS_HEADERS=geometry_msgs log4cxx ros rosconsole rosgraph_msgs sensor_msgs std_msgs
EXTRACT=unzip -q
WGET=wget --quiet

ifeq ($(OSTYPE),linux)
 OS_TAG=linux64
 ROS_PACKAGE=$(ROS_TARGET)_kinetic_headers.zip
 ROS_PACKAGE_PATH=common/$(ROS_PACKAGE)
 ROS_HEADERS += boost XmlRpcDecl.h XmlRpcValue.h
endif

ifeq ($(OSTYPE),darwin)
 OS_TAG=mac
 ROS_PACKAGE=$(ROS_TARGET)_headers.zip
 ROS_PACKAGE_PATH=mac/release/$(ROS_PACKAGE)
 ROS_HEADERS += boost xmlrpcpp
endif

ifeq ($(OSTYPE),windows)
 OS_TAG=windows
 ROS_PACKAGE=$(ROS_TARGET)_headers_windows.zip
 ROS_PACKAGE_PATH=windows/release/$(ROS_PACKAGE)
 # remove .zip
 ROS_SOURCE=$(basename $(ROS_PACKAGE))
 ROS_HEADERS += XmlRpcDecl.h XmlRpcValue.h
else
 # remove .tar.gz
 ROS_SOURCE=$(basename $(basename $(ROS_PACKAGE)))
endif

release debug profile: $(ROS_TARGET)

$(ROS_TARGET):
	@if ! $(WGET) -h > /dev/null ; then echo "You need to download the" $(ROS_PACKAGE) "package and copy it in this directory"; fi
	@echo "#" downloading $(ROS_PACKAGE)
	@$(WGET) https://cyberbotics.com/files/repository/dependencies/$(ROS_PACKAGE_PATH)
	@echo "#" removing old $(ROS_TARGET)
	@rm -rf $(ROS_HEADERS)
	@echo "#" extracting $(ROS_PACKAGE)
	@$(EXTRACT) $(ROS_PACKAGE)
	@rm -rf $(ROS_PACKAGE)
	@echo "#" installing $(ROS_PACKAGE)
ifeq ($(OSTYPE),linux)
	@mv $(ROS_SOURCE)/* .
	@rm -rf $(ROS_SOURCE)
endif
	@touch $(ROS_HEADERS) # update modification date on Linux and Mac

clean:
	rm -rf $(ROS_HEADERS) $(ROS_PACKAGE)
