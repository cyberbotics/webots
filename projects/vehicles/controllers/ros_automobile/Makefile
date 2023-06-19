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
CXX_SOURCES += $(wildcard $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/Ros*.cpp)
CXX_SOURCES += $(wildcard $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/highlevel/*.cpp)

ifeq ($(OSTYPE),windows)
 ros_automobile.exe: $(CXX_SOURCES:.cxx=.d)
else
 ros_automobile: $(CXX_SOURCES:.cxx=.d)
endif

INCLUDE = -I$(WEBOTS_HOME_PATH)/projects/default/controllers/ros -isystem $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include -isystem /opt/ros/$(ROS_DISTRO)/include
LIBRARIES += -lCppDriver -lCppCar -ldriver -lcar

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

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
endif
endif
