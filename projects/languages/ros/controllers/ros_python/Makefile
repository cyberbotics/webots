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

ROS_PYTHON_ZIP = ros_python.zip

ifeq ($(OSTYPE),windows)
 ROS_PYTHON_DEPENDENCY_URL = https://cyberbotics.com/files/repository/dependencies/windows/release/$(ROS_PYTHON_ZIP)
else  # Linux64 version works for macOS too
 ROS_PYTHON_DEPENDENCY_URL = https://cyberbotics.com/files/repository/dependencies/linux64/release/$(ROS_PYTHON_ZIP)
endif

release debug profile: kinetic python

kinetic: python

python:
	@echo "# downloading and installing rospy dependencies"
	@wget -q $(ROS_PYTHON_DEPENDENCY_URL)
	@unzip -q -o $(ROS_PYTHON_ZIP)
	@rm $(ROS_PYTHON_ZIP)

clean:
	@rm -fr kinetic python
