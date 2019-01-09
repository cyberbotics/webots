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

# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

SERVICE_FILES = $(wildcard include/srv/*.srv)
SERVICE_HEADERS = $(SERVICE_FILES:include/srv/%.srv=include/webots_ros/%.h)

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

ifeq ($(OSTYPE),linux)
 CFLAGS = -std=c++11
endif

PYTHON_COMMAND ?= python
ifeq (, $(shell which $(PYTHON_COMMAND) 2> /dev/null))
release debug profile clean:
	@echo -e "# \033[0;33m$(PYTHON_COMMAND) not installed, impossible to generate services/messages required by the ros controller, skipping ros controller\033[0m"
else
CXX_SOURCES = $(wildcard *.cpp)
CXX_SOURCES += $(wildcard $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/Ros*.cpp)

ifeq ($(OSTYPE),windows)
 ros_automobile.exe: $(CXX_SOURCES:.cxx=.d)
else
 ros_automobile: $(CXX_SOURCES:.cxx=.d)
endif

$(CXX_SOURCES:.cxx=.d): $(SERVICE_HEADERS) $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include/templateHeader.h $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include/templateRequest.h $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include/templateResponse.h

include/webots_ros/%.h : include/srv/%.srv $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include/templateHeader.h $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include/templateRequest.h $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include/templateResponse.h $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/headersFromSRV.py $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/headersGenerator.py
	@echo "# generating ROS header" $(notdir $<)
	$(SILENT)python $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/headersFromSRV.py $< $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include

INCLUDE = -I$(WEBOTS_HOME_PATH)/projects/default/controllers/ros -isystem $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include -Iinclude
LIBRARIES += -lCppDriver -lCppCar -ldriver -lcar

# include ros libraries

LIBRARIES += -L$(WEBOTS_HOME_PATH)/projects/default/libraries/ros -lroscpp -lrosconsole -lroscpp_serialization -lrostime
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
 LIBRARIES += -Wl,-rpath,$(WEBOTS_HOME_PATH)/projects/default/libraries/ros
endif

ifeq ($(OSTYPE),darwin)
# Hide Boost warnings
CFLAGS += -Wno-unused-local-typedefs
# Fix warnings about the 'override' keyword.
CFLAGS += -std=c++11
endif

FILES_TO_REMOVE += include/webots_ros

### Do not modify: this includes Webots global Makefile.include

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
endif
