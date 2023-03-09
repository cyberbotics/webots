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

# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

SERVICE_FILES = $(wildcard $(WEBOTS_HOME_PATH)/resources/webots_ros/srv/*.srv)
SERVICE_HEADERS = $(SERVICE_FILES:$(WEBOTS_HOME_PATH)/resources/webots_ros/srv/%.srv=include/services/%.h)

MESSAGE_FILES = $(wildcard $(WEBOTS_HOME_PATH)/resources/webots_ros/msg/*.msg)
MESSAGE_HEADERS = $(MESSAGE_FILES:$(WEBOTS_HOME_PATH)/resources/webots_ros/msg/%.msg=include/messages/%.h)

GENERATED_HEADERS = $(MESSAGE_HEADERS:include/messages/%.h=include/webots_ros/%.h)
GENERATED_HEADERS += $(SERVICE_HEADERS:include/services/%.h=include/webots_ros/%.h)

ifeq ($(OSTYPE),linux)
 CFLAGS = -std=c++11
endif

PYTHON_COMMAND ?= python3
ifeq (, $(ROS_DISTRO))
release debug profile clean:
	@$(ECHO) "# \033[0;33mROS not installed or 'ROS_DISTRO' not defined\033[0m"
else
ifeq (2, $(ROS_VERSION))
release debug profile clean:
	@$(ECHO) "# \033[0;33mROS_DISTRO should not be a ROS2 distribution\033[0m"
else
CXX_SOURCES = $(wildcard *.cpp)
CXX_SOURCES += $(wildcard highlevel/*.cpp)

ifeq ($(OSTYPE),windows)
 ros.exe: $(CXX_SOURCES:.cxx=.d)
else
 ros: $(CXX_SOURCES:.cxx=.d)
endif

$(CXX_SOURCES:.cxx=.d): $(GENERATED_HEADERS) include/templateHeader.h include/templateRequest.h include/templateResponse.h

$(GENERATED_HEADERS): $(SERVICE_HEADERS) $(MESSAGE_HEADERS)

include/messages/%.h: $(WEBOTS_HOME_PATH)/resources/webots_ros/msg/%.msg include/templateHeader.h include/templateRequest.h include/templateResponse.h headersFromMSG.py headersGenerator.py
	@echo "# generating message header" $(notdir $<)
	$(SILENT)$(PYTHON_COMMAND) headersFromMSG.py $<

include/services/%.h: $(WEBOTS_HOME_PATH)/resources/webots_ros/srv/%.srv include/templateHeader.h include/templateRequest.h include/templateResponse.h headersFromSRV.py headersGenerator.py
	@echo "# generating service header" $(notdir $<)
	$(SILENT)$(PYTHON_COMMAND) headersFromSRV.py $<

INCLUDE = -isystem $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include -isystem /opt/ros/$(ROS_DISTRO)/include

# include ros libraries

LIBRARIES += -L/opt/ros/$(ROS_DISTRO)/lib -Wl,-rpath-link=/opt/ros/$(ROS_DISTRO)/lib -lxmlrpcpp -lcpp_common -lrosconsole_backend_interface -lroscpp -lrosconsole -lrosconsole_log4cxx -lroscpp_serialization -lrostime -lcontroller_manager -lboost_system
ifeq ($(OSTYPE),windows)
 LIBRARIES += -lws2_32
 ifeq ($(MAKECMDGOALS),debug)
  # The following option is fixing the following error
  # appearing only on Windows 64 bits in debug mode
  # (probably that the number of templates is causing this)
  # RosSupervisor.o: too many sections
  CFLAGS += -Wa,-mbig-obj
 endif
endif

ifeq ($(OSTYPE),linux)
 LIBRARIES += -Wl,-rpath,$(WEBOTS_HOME_PATH)/projects/default/controllers/ros/lib/ros
endif

ifeq ($(OSTYPE),darwin)
# Hide Boost warnings
CFLAGS += -Wno-unused-local-typedefs
# Fix warnings about the 'override' keyword.
CFLAGS += -std=c++11
endif

FILES_TO_REMOVE += include/webots_ros include/messages include/services headersGenerator.pyc include/XmlRpcDecl.h include/XmlRpcValue.h include/geometry_msgs include/log4cxx include/ros include/rosconsole include/rosgraph_msgs include/sensor_msgs include/std_msgs include/hardware_interface include/controller_manager include/boost __pycache__ $(wildcard include/*.zip)

### Do not modify: this includes Webots global Makefile.include

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
endif
endif
