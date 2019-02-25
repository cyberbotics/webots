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

ROS_PYTHON_ZIP = ros_python.zip
ROS_PYTHON_DEPENDENCY_URL = https://cyberbotics.com/files/repository/dependencies/common/$(ROS_PYTHON_ZIP)

release debug profile: kinetic python

kinetic python:
	@echo "# downloading and installing rospy dependencies"
	@wget -q $(ROS_PYTHON_DEPENDENCY_URL)
	@unzip -q -o $(ROS_PYTHON_ZIP)
	@rm $(ROS_PYTHON_ZIP)

clean:
	@rm -fr kinetic python
