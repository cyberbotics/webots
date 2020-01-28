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

# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

USE_C_API = true

# define the opencv installation directory if not set in an environment variable (tested with OpenCV 2.4)
#OPENCV_DIR =

ifneq ($(OPENCV_DIR),)
INCLUDE = -I$(OPENCV_DIR)/include/opencv4
ifeq ($(OSTYPE),windows)
OPENCV_VERSION := $(subst .,,$(shell $(OPENCV_DIR)/bin/opencv_version.exe))
LIBRARIES = -L$(OPENCV_DIR)/lib -lopencv_core$(OPENCV_VERSION) -lopencv_imgproc$(OPENCV_VERSION)
else
LIBRARIES = -L"$(OPENCV_DIR)/lib" -lopencv_core -lopencv_imgproc
endif
# Do not modify the following: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
else
release debug profile clean:
	@+echo "# opencv not installed, skipping vision controller."
endif
